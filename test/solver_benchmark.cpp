/*
 * Benchmark for ScanSolver plugins on standard 2D pose-graph datasets
 * (g2o format). Feeds the graph through the real solver interface and
 * times Compute().
 *
 *   solver_benchmark <file.g2o> [--reps N] [--every K]
 *
 * Default is one batch solve per rep for each plugin, interleaved.
 * --every K additionally re-solves after every K constraints while the
 * graph grows, printing a time-vs-size curve (one pass, first rep only).
 * Cost is reported with the Ceres convention (0.5 * sum of squared
 * sqrt-information-weighted residuals) so quality is visible next to
 * time.
 */

#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <glog/logging.h>

#include <Eigen/Cholesky>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "karto_sdk/Mapper.h"
#include "../solvers/ceres_solver.hpp"
#include "../solvers/arael_solver.hpp"

namespace
{

struct G2oEdge
{
  int a, b;
  double dx, dy, dth;
  Eigen::Matrix3d info;
};

struct G2oGraph
{
  std::vector<karto::Pose2> poses;  // index = id (must be contiguous)
  std::vector<G2oEdge> edges;
};

karto::Pose2 ComposePose(const karto::Pose2 & a, const G2oEdge & e)
{
  const double ca = std::cos(a.GetHeading()), sa = std::sin(a.GetHeading());
  return karto::Pose2(
    a.GetX() + ca * e.dx - sa * e.dy,
    a.GetY() + sa * e.dx + ca * e.dy,
    a.GetHeading() + e.dth);
}

bool LoadG2o(const char * path, G2oGraph & g)
{
  std::ifstream f(path);
  if (!f) {
    return false;
  }
  std::string line, tag;
  int max_id = -1;
  while (std::getline(f, line)) {
    std::istringstream s(line);
    s >> tag;
    if (tag == "VERTEX_SE2") {
      int id;
      double x, y, th;
      s >> id >> x >> y >> th;
      if (id != static_cast<int>(g.poses.size())) {
        std::fprintf(stderr, "non-contiguous vertex id %d\n", id);
        return false;
      }
      g.poses.push_back(karto::Pose2(x, y, th));
    } else if (tag == "EDGE_SE2") {
      G2oEdge e;
      double i11, i12, i13, i22, i23, i33;
      s >> e.a >> e.b >> e.dx >> e.dy >> e.dth >>
      i11 >> i12 >> i13 >> i22 >> i23 >> i33;
      e.info << i11, i12, i13, i12, i22, i23, i13, i23, i33;
      g.edges.push_back(e);
      max_id = std::max(max_id, std::max(e.a, e.b));
    }
  }
  if (g.poses.empty() && max_id >= 0) {
    // edges-only file: initialize by chaining the odometry edges, the
    // same initial guess karto itself would build
    g.poses.assign(max_id + 1, karto::Pose2(0.0, 0.0, 0.0));
    for (const auto & e : g.edges) {
      if (e.b == e.a + 1) {
        g.poses[e.b] = ComposePose(g.poses[e.a], e);
      }
    }
    std::printf("(edges-only file: odometry-chained initialization)\n");
  }
  return !g.poses.empty() && !g.edges.empty();
}

// Whole-process peak RSS so far (graph + solver + ROS); each run is a
// fresh forked child, so this is per-run. Linux reports KB.
double MaxRssMb()
{
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return ru.ru_maxrss / 1024.0;
}

double WrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// Ceres-convention cost of the current corrections
double Cost(const std::map<int, karto::Pose2> & poses, const G2oGraph & g)
{
  double cost = 0.0;
  for (const auto & e : g.edges) {
    const karto::Pose2 & pa = poses.at(e.a);
    const karto::Pose2 & pb = poses.at(e.b);
    Eigen::Matrix3d sqrt_info = e.info.llt().matrixU();
    const double ca = std::cos(pa.GetHeading()), sa = std::sin(pa.GetHeading());
    Eigen::Vector3d r;
    r(0) = ca * (pb.GetX() - pa.GetX()) + sa * (pb.GetY() - pa.GetY()) - e.dx;
    r(1) = -sa * (pb.GetX() - pa.GetX()) + ca * (pb.GetY() - pa.GetY()) - e.dy;
    r(2) = WrapAngle(pb.GetHeading() - pa.GetHeading() - e.dth);
    r = sqrt_info * r;
    cost += 0.5 * r.squaredNorm();
  }
  return cost;
}

// Owns the karto objects for one solver run (leaked; karto
// default-constructed scans are not safely destructible)
class Rig
{
public:
  Rig(const std::string & plugin, bool verbose)
  {
    if (plugin == "ceres") {
      solver_.reset(new solver_plugins::CeresSolver());
    } else {
      solver_.reset(new solver_plugins::AraelSolver());
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("bench_" + plugin);
    node_->declare_parameter("debug_logging", rclcpp::ParameterValue(verbose));
    solver_->Configure(node_);
  }

  ~Rig()
  {
    // a LifecycleNode destroyed unconfigured warns; finalize it properly
    node_->shutdown();
  }

  void AddNode(int id, const karto::Pose2 & pose)
  {
    auto scan = new karto::LocalizedRangeScan();
    scan->SetUniqueId(id);
    scan->SetCorrectedPose(pose);
    auto vertex = new karto::Vertex<karto::LocalizedRangeScan>(scan);
    solver_->AddNode(vertex);
    vertices_[id] = vertex;
  }

  void AddConstraint(const G2oEdge & e)
  {
    auto edge = new karto::Edge<karto::LocalizedRangeScan>(
      vertices_[e.a], vertices_[e.b]);
    karto::Matrix3 cov;
    Eigen::Matrix3d c = e.info.inverse();
    for (int r = 0; r < 3; ++r) {
      for (int col = 0; col < 3; ++col) {
        cov(r, col) = c(r, col);
      }
    }
    edge->SetLabel(new karto::LinkInfo(
        karto::Pose2(), karto::Pose2(e.dx, e.dy, e.dth), cov));
    solver_->AddConstraint(edge);
  }

  // returns wall milliseconds of Compute()
  double Solve()
  {
    auto t0 = std::chrono::steady_clock::now();
    solver_->Compute();
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  }

  std::map<int, karto::Pose2> Corrections() const
  {
    std::map<int, karto::Pose2> out;
    for (const auto & c : solver_->GetCorrections()) {
      out[c.first] = c.second;
    }
    return out;
  }

  int Iterations() const
  {
    if (auto * c = dynamic_cast<solver_plugins::CeresSolver *>(solver_.get())) {
      return c->last_iterations();
    }
    if (auto * a = dynamic_cast<solver_plugins::AraelSolver *>(solver_.get())) {
      return a->last_iterations();
    }
    return 0;
  }

  // the cost the solver itself saw (after karto's covariance round
  // trip), as opposed to the reference cost from the raw file
  double InnerCost() const
  {
    if (auto * c = dynamic_cast<solver_plugins::CeresSolver *>(solver_.get())) {
      return c->last_cost();
    }
    if (auto * a = dynamic_cast<solver_plugins::AraelSolver *>(solver_.get())) {
      return a->last_cost();
    }
    return 0.0;
  }

private:
  std::unique_ptr<karto::ScanSolver> solver_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::map<int, karto::Vertex<karto::LocalizedRangeScan> *> vertices_;
};

void RunBatch(const std::string & plugin, const G2oGraph & g, bool verbose)
{
  Rig rig(plugin, verbose);
  for (size_t i = 0; i < g.poses.size(); ++i) {
    rig.AddNode(static_cast<int>(i), g.poses[i]);
  }
  for (const auto & e : g.edges) {
    rig.AddConstraint(e);
  }
  std::map<int, karto::Pose2> init;
  for (size_t i = 0; i < g.poses.size(); ++i) {
    init[static_cast<int>(i)] = g.poses[i];
  }
  const double cost0 = Cost(init, g);
  const double ms = rig.Solve();
  const double cost1 = Cost(rig.Corrections(), g);
  std::printf(
    "batch %-6s nodes=%zu edges=%zu solve_ms=%.1f iters=%d maxrss_mb=%.1f"
    " cost %.3f -> %.6f inner=%.6f\n",
    plugin.c_str(), g.poses.size(), g.edges.size(), ms, rig.Iterations(),
    MaxRssMb(), cost0, cost1, rig.InnerCost());
  std::fflush(stdout);
}

void RunIncremental(const std::string & plugin, const G2oGraph & g, int every, bool verbose)
{
  Rig rig(plugin, verbose);
  size_t next_pose = 0;
  int since_solve = 0;
  double total_ms = 0.0;
  int solves = 0;
  int total_iters = 0;
  std::printf("curve %s: nodes:ms", plugin.c_str());
  for (const auto & e : g.edges) {
    const size_t need = static_cast<size_t>(std::max(e.a, e.b)) + 1;
    while (next_pose < need) {
      rig.AddNode(static_cast<int>(next_pose), g.poses[next_pose]);
      ++next_pose;
    }
    rig.AddConstraint(e);
    if (++since_solve >= every) {
      const double ms = rig.Solve();
      total_ms += ms;
      total_iters += rig.Iterations();
      ++solves;
      since_solve = 0;
      std::printf(" %zu:%.1f", next_pose, ms);
    }
  }
  const double ms = rig.Solve();
  total_ms += ms;
  total_iters += rig.Iterations();
  ++solves;
  std::printf(" %zu:%.1f\n", next_pose, ms);
  const double cost1 = Cost(rig.Corrections(), g);
  std::printf(
    "curve %-6s total: solves=%d total_ms=%.1f iters=%d maxrss_mb=%.1f"
    " final_cost=%.6f inner=%.6f\n",
    plugin.c_str(), solves, total_ms, total_iters, MaxRssMb(), cost1,
    rig.InnerCost());
  std::fflush(stdout);
}

}  // namespace

// Each measurement runs in a forked child so peak RSS is per-run
// instead of accumulating across solvers; rclcpp/DDS is initialized
// only inside children so the fork is safe.
template<typename Fn>
void RunForked(const std::string & label, Fn fn)
{
  pid_t pid = fork();
  if (pid == 0) {
    rclcpp::init(0, nullptr);
    fn();
    rclcpp::shutdown();
    std::_Exit(0);
  }
  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::fprintf(stderr, "%s: child failed\n", label.c_str());
  }
}

void PrintHelp(const char * prog)
{
  std::printf(
    "usage: %s <file.g2o> [options]\n"
    "\n"
    "Benchmark the Ceres and Arael ScanSolver plugins on a 2D pose\n"
    "graph (VERTEX_SE2/EDGE_SE2). Runs interleave the two solvers; each\n"
    "run executes in a forked child so peak memory is measured per run.\n"
    "\n"
    "options:\n"
    "  --reps N     interleaved batch-solve pairs (default 3)\n"
    "  --every K    also grow the graph and re-solve every K constraints,\n"
    "               printing a nodes:ms curve per solver (default off)\n"
    "  --verbose    solver debug output (Ceres full report, Arael\n"
    "               iteration summaries)\n"
    "  --help       this text\n"
    "\n"
    "Output: 'batch <solver> ... solve_ms=... maxrss_mb=... cost <initial>\n"
    "-> <final>' (cost is 0.5 * sum of squared sqrt-information-weighted\n"
    "residuals; maxrss is the run's whole-process peak), and\n"
    "'curve <solver> ...' rows for --every.\n", prog);
}

int main(int argc, char ** argv)
{
  int reps = 3, every = 0;
  bool verbose = false;
  const char * path = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
      PrintHelp(argv[0]);
      return 0;
    } else if (!std::strcmp(argv[i], "--verbose")) {
      verbose = true;
    } else if (!std::strcmp(argv[i], "--reps") && i + 1 < argc) {
      reps = std::atoi(argv[++i]);
    } else if (!std::strcmp(argv[i], "--every") && i + 1 < argc) {
      every = std::atoi(argv[++i]);
    } else if (argv[i][0] != '-' && !path) {
      path = argv[i];
    } else {
      std::fprintf(stderr, "unknown option: %s (see --help)\n", argv[i]);
      return 1;
    }
  }
  if (!path) {
    PrintHelp(argv[0]);
    return 1;
  }

  G2oGraph g;
  if (!LoadG2o(path, g)) {
    std::fprintf(stderr, "failed to load %s\n", path);
    return 1;
  }

  // Quiet Ceres' per-process warnings (num_threads clamping) and the
  // uninitialized-logging banner; children inherit the initialized state
  FLAGS_minloglevel = google::GLOG_ERROR;
  google::InitGoogleLogging(argv[0]);

  for (int r = 0; r < reps; ++r) {
    RunForked("ceres", [&] {RunBatch("ceres", g, verbose);});
    RunForked("arael", [&] {RunBatch("arael", g, verbose);});
  }
  if (every > 0) {
    RunForked("ceres", [&] {RunIncremental("ceres", g, every, verbose);});
    RunForked("arael", [&] {RunIncremental("arael", g, every, verbose);});
  }
  return 0;
}

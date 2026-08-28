/*
 * Equivalence tests between the Ceres and Arael solver plugins: both
 * optimize the identical cost (see solvers/ceres_utils.h
 * PoseGraph2dErrorTerm), so on the same graph they must reach the same
 * optimum. The reference cost here is computed independently of both.
 */

#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "karto_sdk/Mapper.h"
#include "../solvers/ceres_solver.hpp"
#include "../solvers/arael_solver.hpp"

namespace
{

rclcpp_lifecycle::LifecycleNode::SharedPtr MakeNode(const std::string & name)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name);
  // SOLVER_TEST_DEBUG=1 turns on both solvers' debug output
  node->declare_parameter(
    "debug_logging",
    rclcpp::ParameterValue(std::getenv("SOLVER_TEST_DEBUG") != nullptr));
  return node;
}

struct Measurement
{
  int a, b;
  karto::Pose2 delta;
  karto::Matrix3 cov;
};

// The graph as data plus the karto objects fed to a solver
struct TestGraph
{
  std::vector<karto::Pose2> initial;
  std::vector<Measurement> measurements;
};

// Feeds one solver; owns the karto objects backing it
class SolverRig
{
public:
  SolverRig(karto::ScanSolver * solver, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : solver_(solver)
  {
    solver_->Configure(node);
  }

  // The karto objects are leaked deliberately, like in
  // lifelong_metrics_test.cpp: default-constructed scans are not safely
  // destructible (the serialization constructors leave members
  // uninitialized)
  void AddNode(int id, const karto::Pose2 & pose)
  {
    auto scan = new karto::LocalizedRangeScan();
    scan->SetUniqueId(id);
    scan->SetCorrectedPose(pose);
    auto vertex = new karto::Vertex<karto::LocalizedRangeScan>(scan);
    solver_->AddNode(vertex);
    vertices_[id] = vertex;
  }

  void AddConstraint(const Measurement & m)
  {
    // Edge registers itself with its vertices and owns its label
    auto edge = new karto::Edge<karto::LocalizedRangeScan>(
      vertices_[m.a], vertices_[m.b]);
    edge->SetLabel(new karto::LinkInfo(karto::Pose2(), m.delta, m.cov));
    solver_->AddConstraint(edge);
  }

  void Feed(const TestGraph & g)
  {
    for (size_t i = 0; i < g.initial.size(); ++i) {
      AddNode(static_cast<int>(i), g.initial[i]);
    }
    for (const auto & m : g.measurements) {
      AddConstraint(m);
    }
  }

  std::map<int, karto::Pose2> Solve()
  {
    solver_->Compute();
    std::map<int, karto::Pose2> out;
    for (const auto & c : solver_->GetCorrections()) {
      out[c.first] = c.second;
    }
    return out;
  }

  karto::ScanSolver * operator->() {return solver_;}

private:
  karto::ScanSolver * solver_;
  std::map<int, karto::Vertex<karto::LocalizedRangeScan> *> vertices_;
};

double WrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// Reference cost with the Ceres convention (0.5 * sum of squared
// sqrt-information-scaled residuals), computed from the raw data
double ReferenceCost(
  const std::map<int, karto::Pose2> & poses,
  const std::vector<Measurement> & measurements)
{
  double cost = 0.0;
  for (const auto & m : measurements) {
    if (poses.count(m.a) == 0 || poses.count(m.b) == 0) {
      continue;
    }
    const karto::Pose2 & pa = poses.at(m.a);
    const karto::Pose2 & pb = poses.at(m.b);

    karto::Matrix3 information = m.cov.Inverse();
    Eigen::Matrix3d info;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        info(r, c) = information(r, c);
      }
    }
    Eigen::Matrix3d sqrt_info = info.llt().matrixU();

    const double ca = std::cos(pa.GetHeading()), sa = std::sin(pa.GetHeading());
    const double ddx = pb.GetX() - pa.GetX(), ddy = pb.GetY() - pa.GetY();
    Eigen::Vector3d r;
    r(0) = ca * ddx + sa * ddy - m.delta.GetX();
    r(1) = -sa * ddx + ca * ddy - m.delta.GetY();
    r(2) = WrapAngle(pb.GetHeading() - pa.GetHeading() - m.delta.GetHeading());
    r = sqrt_info * r;
    cost += 0.5 * r.squaredNorm();
  }
  return cost;
}

// A correlated SPD covariance, varied per edge
karto::Matrix3 MakeCovariance(std::mt19937 & rng)
{
  std::uniform_real_distribution<double> u(0.5, 1.5);
  // C = L L^T with a fixed correlated lower factor scaled per edge
  Eigen::Matrix3d L;
  L << 0.10 * u(rng), 0.0, 0.0,
    0.02, 0.12 * u(rng), 0.0,
    0.01, 0.015, 0.06 * u(rng);
  Eigen::Matrix3d C = L * L.transpose();
  karto::Matrix3 cov;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      cov(r, c) = C(r, c);
    }
  }
  return cov;
}

karto::Pose2 Compose(const karto::Pose2 & a, const karto::Pose2 & d)
{
  const double ca = std::cos(a.GetHeading()), sa = std::sin(a.GetHeading());
  return karto::Pose2(
    a.GetX() + ca * d.GetX() - sa * d.GetY(),
    a.GetY() + sa * d.GetX() + ca * d.GetY(),
    WrapAngle(a.GetHeading() + d.GetHeading()));
}

karto::Pose2 RelativePose(const karto::Pose2 & a, const karto::Pose2 & b)
{
  const double ca = std::cos(a.GetHeading()), sa = std::sin(a.GetHeading());
  const double dx = b.GetX() - a.GetX(), dy = b.GetY() - a.GetY();
  return karto::Pose2(
    ca * dx + sa * dy, -sa * dx + ca * dy,
    WrapAngle(b.GetHeading() - a.GetHeading()));
}

// Manhattan-style walk with noisy odometry measurements and loop
// closures on revisits; the initial guess integrates the noisy odometry
TestGraph MakeManhattanGraph(unsigned seed, int n)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> tn(0.0, 0.03), rn(0.0, 0.01);
  std::uniform_int_distribution<int> turn(0, 3);

  TestGraph g;
  std::vector<karto::Pose2> truth;
  truth.push_back(karto::Pose2(0.0, 0.0, 0.0));
  g.initial.push_back(truth[0]);

  for (int i = 1; i < n; ++i) {
    double dth = 0.0;
    if (turn(rng) == 0) {
      dth = (turn(rng) < 2) ? M_PI / 2.0 : -M_PI / 2.0;
    }
    // turn, then move 1m along the new heading; turn further when the
    // step would leave the arena, so the walk revisits places and loop
    // closures actually happen
    karto::Pose2 next;
    for (int tries = 0; tries < 4; ++tries) {
      next = Compose(
        Compose(truth[i - 1], karto::Pose2(0.0, 0.0, dth)),
        karto::Pose2(1.0, 0.0, 0.0));
      if (std::abs(next.GetX()) < 4.0 && std::abs(next.GetY()) < 4.0) {
        break;
      }
      dth = WrapAngle(dth + M_PI / 2.0);
    }
    truth.push_back(next);

    karto::Pose2 rel = RelativePose(truth[i - 1], truth[i]);
    karto::Pose2 measured(
      rel.GetX() + tn(rng), rel.GetY() + tn(rng),
      WrapAngle(rel.GetHeading() + rn(rng)));
    g.initial.push_back(Compose(g.initial[i - 1], measured));
    g.measurements.push_back({i - 1, i, measured, MakeCovariance(rng)});
  }

  // loop closures between nearby non-consecutive poses
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < i - 10; ++j) {
      const double dx = truth[i].GetX() - truth[j].GetX();
      const double dy = truth[i].GetY() - truth[j].GetY();
      if (dx * dx + dy * dy < 0.25) {
        karto::Pose2 rel = RelativePose(truth[j], truth[i]);
        karto::Pose2 measured(
          rel.GetX() + tn(rng), rel.GetY() + tn(rng),
          WrapAngle(rel.GetHeading() + rn(rng)));
        g.measurements.push_back({j, i, measured, MakeCovariance(rng)});
        break;
      }
    }
  }
  return g;
}

// The Ceres plugin runs with deliberately loose production tolerances
// and can stop well short of the optimum in one Compute(); every
// Compute() warm-starts from the last state, so iterating converges.
// Used for both solvers so the comparison is at the shared optimum.
std::map<int, karto::Pose2> SolveToConvergence(
  SolverRig & rig, const std::vector<Measurement> & measurements)
{
  std::map<int, karto::Pose2> out = rig.Solve();
  double cost = ReferenceCost(out, measurements);
  for (int i = 0; i < 20; ++i) {
    std::map<int, karto::Pose2> next = rig.Solve();
    double next_cost = ReferenceCost(next, measurements);
    const bool done = std::abs(cost - next_cost) <= 1e-10 * cost;
    out = std::move(next);
    cost = next_cost;
    if (done) {
      break;
    }
  }
  return out;
}

std::map<int, karto::Pose2> AsMap(const std::vector<karto::Pose2> & poses)
{
  std::map<int, karto::Pose2> out;
  for (size_t i = 0; i < poses.size(); ++i) {
    out[static_cast<int>(i)] = poses[i];
  }
  return out;
}

void ExpectPosesNear(
  const std::map<int, karto::Pose2> & a,
  const std::map<int, karto::Pose2> & b, double tol)
{
  ASSERT_EQ(a.size(), b.size());
  for (const auto & p : a) {
    ASSERT_TRUE(b.count(p.first) == 1) << "missing id " << p.first;
    const karto::Pose2 & q = b.at(p.first);
    EXPECT_NEAR(p.second.GetX(), q.GetX(), tol) << "id " << p.first;
    EXPECT_NEAR(p.second.GetY(), q.GetY(), tol) << "id " << p.first;
    EXPECT_NEAR(
      WrapAngle(p.second.GetHeading() - q.GetHeading()), 0.0,
      tol) << "id " << p.first;
  }
}

}  // namespace

// A single constraint from a fixed pose has an exact-fit optimum: the
// composed measurement. Catches any frame/sign/weighting mismatch.
TEST(SolverEquivalence, TwoPosesExactOptimum)
{
  TestGraph g;
  g.initial.push_back(karto::Pose2(0.2, -0.1, 0.05));
  g.initial.push_back(karto::Pose2(1.3, 0.4, 0.4));
  std::mt19937 rng(7);
  karto::Pose2 delta(0.9, 0.1, 0.15);
  g.measurements.push_back({0, 1, delta, MakeCovariance(rng)});

  const karto::Pose2 expect = Compose(g.initial[0], delta);

  solver_plugins::CeresSolver ceres;
  SolverRig ceres_rig(&ceres, MakeNode("ceres_two"));
  ceres_rig.Feed(g);
  auto ceres_out = ceres_rig.Solve();

  solver_plugins::AraelSolver arael;
  SolverRig arael_rig(&arael, MakeNode("arael_two"));
  arael_rig.Feed(g);
  auto arael_out = arael_rig.Solve();

  // Ceres runs with deliberately loose tolerances (1e-3); arael
  // converges tighter
  struct { std::map<int, karto::Pose2> * out; double tol; } checks[] = {
    {&ceres_out, 1e-3}, {&arael_out, 1e-8},
  };
  for (auto & c : checks) {
    EXPECT_NEAR(c.out->at(0).GetX(), g.initial[0].GetX(), 1e-9);
    EXPECT_NEAR(c.out->at(0).GetY(), g.initial[0].GetY(), 1e-9);
    EXPECT_NEAR(c.out->at(1).GetX(), expect.GetX(), c.tol);
    EXPECT_NEAR(c.out->at(1).GetY(), expect.GetY(), c.tol);
    EXPECT_NEAR(WrapAngle(c.out->at(1).GetHeading() - expect.GetHeading()), 0.0, c.tol);
  }
}

// Full graph: both plugins must reach the same optimum of the same cost
TEST(SolverEquivalence, ManhattanGraph)
{
  TestGraph g = MakeManhattanGraph(42, 80);
  // enough loop closures for a redundant graph with a nonzero optimum
  ASSERT_GE(g.measurements.size(), g.initial.size() + 5);

  const double initial_cost = ReferenceCost(AsMap(g.initial), g.measurements);

  solver_plugins::CeresSolver ceres;
  SolverRig ceres_rig(&ceres, MakeNode("ceres_manhattan"));
  ceres_rig.Feed(g);
  auto ceres_first = ceres_rig.Solve();
  auto ceres_out = SolveToConvergence(ceres_rig, g.measurements);

  solver_plugins::AraelSolver arael;
  SolverRig arael_rig(&arael, MakeNode("arael_manhattan"));
  arael_rig.Feed(g);
  auto arael_first = arael_rig.Solve();
  auto arael_out = SolveToConvergence(arael_rig, g.measurements);

  const double ceres_cost = ReferenceCost(ceres_out, g.measurements);
  const double arael_cost = ReferenceCost(arael_out, g.measurements);

  EXPECT_LT(ceres_cost, initial_cost);
  EXPECT_LT(arael_cost, initial_cost);
  // a single arael Compute() must never land above a single Ceres
  // Compute() on Ceres' own cost
  EXPECT_LE(
    ReferenceCost(arael_first, g.measurements),
    ReferenceCost(ceres_first, g.measurements) * (1.0 + 1e-6));
  // Same functional: arael must never land above Ceres on Ceres' own
  // cost, and both agree within Ceres' convergence floor (its plugin
  // config stops at 1e-3 relative decrease, so it plateaus ~0.1% above
  // the optimum even when iterated)
  ASSERT_GT(ceres_cost, 1e-6);  // the ratio is meaningless near a zero optimum
  EXPECT_LE(arael_cost, ceres_cost * (1.0 + 1e-6));
  EXPECT_NEAR(arael_cost / ceres_cost, 1.0, 5e-3);
  // cost is the equivalence criterion; near the plateau a flat valley
  // moves poses by centimeters at 0.1% cost, so poses only get a
  // same-basin sanity bound
  ExpectPosesNear(ceres_out, arael_out, 0.05);
}

// Localization-style removal: drop constraints and nodes, re-solve
TEST(SolverEquivalence, RemovalEquivalence)
{
  TestGraph g = MakeManhattanGraph(1234, 60);
  ASSERT_GE(g.measurements.size(), g.initial.size() + 5);

  solver_plugins::CeresSolver ceres;
  SolverRig ceres_rig(&ceres, MakeNode("ceres_removal"));
  solver_plugins::AraelSolver arael;
  SolverRig arael_rig(&arael, MakeNode("arael_removal"));
  ceres_rig.Feed(g);
  arael_rig.Feed(g);
  ceres_rig.Solve();
  arael_rig.Solve();

  // remove the last 10 nodes and every constraint touching them,
  // constraints first as the localization flow does
  const int cut = static_cast<int>(g.initial.size()) - 10;
  std::vector<Measurement> kept;
  for (const auto & m : g.measurements) {
    if (m.a >= cut || m.b >= cut) {
      ceres_rig->RemoveConstraint(m.a, m.b);
      arael_rig->RemoveConstraint(m.a, m.b);
    } else {
      kept.push_back(m);
    }
  }
  for (int id = cut; id < static_cast<int>(g.initial.size()); ++id) {
    ceres_rig->RemoveNode(id);
    arael_rig->RemoveNode(id);
  }

  auto ceres_out = SolveToConvergence(ceres_rig, kept);
  auto arael_out = SolveToConvergence(arael_rig, kept);
  ASSERT_EQ(ceres_out.size(), static_cast<size_t>(cut));
  ASSERT_EQ(arael_out.size(), static_cast<size_t>(cut));

  const double ceres_cost = ReferenceCost(ceres_out, kept);
  const double arael_cost = ReferenceCost(arael_out, kept);
  ASSERT_GT(ceres_cost, 1e-6);  // the ratio is meaningless near a zero optimum
  EXPECT_LE(arael_cost, ceres_cost * (1.0 + 1e-6));
  EXPECT_NEAR(arael_cost / ceres_cost, 1.0, 5e-3);
  // cost is the equivalence criterion; near the plateau a flat valley
  // moves poses by centimeters at 0.1% cost, so poses only get a
  // same-basin sanity bound
  ExpectPosesNear(ceres_out, arael_out, 0.05);
}

// ModifyNode applies a manual pose with additive yaw on both solvers
TEST(SolverEquivalence, ModifyNodeEquivalence)
{
  TestGraph g = MakeManhattanGraph(99, 60);
  ASSERT_GE(g.measurements.size(), g.initial.size() + 5);

  solver_plugins::CeresSolver ceres;
  SolverRig ceres_rig(&ceres, MakeNode("ceres_modify"));
  solver_plugins::AraelSolver arael;
  SolverRig arael_rig(&arael, MakeNode("arael_modify"));
  ceres_rig.Feed(g);
  arael_rig.Feed(g);

  // small enough to stay in the optimum's basin for both solvers
  Eigen::Vector3d moved(0.2, -0.1, 0.05);
  ceres_rig->ModifyNode(20, moved);
  arael_rig->ModifyNode(20, moved);

  auto ceres_out = SolveToConvergence(ceres_rig, g.measurements);
  auto arael_out = SolveToConvergence(arael_rig, g.measurements);
  const double ceres_cost = ReferenceCost(ceres_out, g.measurements);
  const double arael_cost = ReferenceCost(arael_out, g.measurements);
  ASSERT_GT(ceres_cost, 1e-6);  // the ratio is meaningless near a zero optimum
  EXPECT_LE(arael_cost, ceres_cost * (1.0 + 1e-6));
  EXPECT_NEAR(arael_cost / ceres_cost, 1.0, 5e-3);
  // cost is the equivalence criterion; near the plateau a flat valley
  // moves poses by centimeters at 0.1% cost, so poses only get a
  // same-basin sanity bound
  ExpectPosesNear(ceres_out, arael_out, 0.05);
}

#include <Eigen/Cholesky>

#include <unordered_map>
#include <string>
#include <utility>

#include <graph.hpp>
#include "arael_solver.hpp"

namespace solver_plugins
{

// Collision-free key for a directed node pair
static std::size_t ConstraintHash(const int & x, const int & y)
{
  return ((std::size_t)(uint32_t)x << 32) | (std::size_t)(uint32_t)y;
}

/*****************************************************************************/
AraelSolver::AraelSolver()
: first_node_id_(-1), debug_logging_(false), last_iterations_(0),
  last_cost_(0.0)
/*****************************************************************************/
{
}

/*****************************************************************************/
void AraelSolver::Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
/*****************************************************************************/
{
  logger_ = node->get_logger();
  debug_logging_ = node->get_parameter("debug_logging").as_bool();

  corrections_.clear();
  nodes_.clear();
  constraints_.clear();
  first_node_id_ = -1;
}

/*****************************************************************************/
AraelSolver::~AraelSolver()
/*****************************************************************************/
{
}

/*****************************************************************************/
void AraelSolver::Compute()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (nodes_.size() == 0) {
    RCLCPP_WARN(
      logger_,
      "AraelSolver: Compute was called when there are no nodes."
      " This shouldn't happen.");
    return;
  }

  // Constraints whose node was removed die with it, like Ceres'
  // RemoveParameterBlock dropping dependent residual blocks
  for (auto it = constraints_.begin(); it != constraints_.end(); ) {
    if (nodes_.find(it->second.a) == nodes_.end() ||
      nodes_.find(it->second.b) == nodes_.end())
    {
      it = constraints_.erase(it);
    } else {
      ++it;
    }
  }

  // Rebuild the problem from the stored graph; construction is cheap
  // relative to the solve
  slam_toolbox_arael::Graph graph;
  std::unordered_map<int, uint32_t> index;
  index.reserve(nodes_.size());
  graph.poses().reserve(nodes_.size());
  for (auto & n : nodes_) {
    uint32_t i = graph.poses().size();
    auto pose = graph.poses().push();
    pose.set_pos({n.second(0), n.second(1)});
    pose.set_rot_angle(n.second(2));
    if (n.first == first_node_id_) {
      // gauge anchor, same as Ceres' SetParameterBlockConstant
      pose.set_pos_optimize(false);
      pose.set_rot_angle_optimize(false);
    }
    index.insert(std::make_pair(n.first, i));
  }

  graph.edges().reserve(constraints_.size());
  for (auto & c : constraints_) {
    const Constraint & con = c.second;
    auto e = graph.edges().push();
    e.set_a(graph.poses().ref_at(index[con.a]));
    e.set_b(graph.poses().ref_at(index[con.b]));
    e.set_dx(con.dx);
    e.set_dy(con.dy);
    e.set_dth(con.dth);
    e.set_s00(con.sqrt_info[0]);
    e.set_s01(con.sqrt_info[1]);
    e.set_s02(con.sqrt_info[2]);
    e.set_s11(con.sqrt_info[3]);
    e.set_s12(con.sqrt_info[4]);
    e.set_s22(con.sqrt_info[5]);
  }

  // well_conditioned: near-Gauss-Newton damping with quick stops.
  // Most solves are warm re-solves, which this makes about twice as
  // fast as conservative damping; Compute() cannot tell them from a
  // cold start
  slam_toolbox_arael::LmConfig cfg =
    slam_toolbox_arael::LmConfig::well_conditioned();
  slam_toolbox_arael::SolveResult result = graph.solve_sparse(cfg);
  if (result.is_err()) {
    RCLCPP_WARN(
      logger_, "AraelSolver: solve failed: %s", result.error().message);
    return;
  }
  last_iterations_ = static_cast<int>(result->iterations);
  last_cost_ = 0.5 * result->end_cost;
  if (debug_logging_) {
    RCLCPP_INFO(
      logger_,
      "AraelSolver: %zu nodes, %zu constraints, %u iterations,"
      " cost %f -> %f (sum of squares; Ceres reports half of this)",
      nodes_.size(), constraints_.size(),
      result->iterations, result->start_cost, result->end_cost);
  }

  // store corrected poses; nodes_ keeps them as the next warm start
  corrections_.clear();
  corrections_.reserve(nodes_.size());
  karto::Pose2 pose;
  std::unordered_map<int, Eigen::Vector3d>::iterator iter = nodes_.begin();
  for (; iter != nodes_.end(); ++iter) {
    auto solved = graph.poses()[index[iter->first]];
    iter->second(0) = solved.pos().x;
    iter->second(1) = solved.pos().y;
    iter->second(2) = karto::math::NormalizeAngle(solved.rot_angle());
    pose.SetX(iter->second(0));
    pose.SetY(iter->second(1));
    pose.SetHeading(iter->second(2));
    corrections_.push_back(std::make_pair(iter->first, pose));
  }
}

/*****************************************************************************/
const karto::ScanSolver::IdPoseVector & AraelSolver::GetCorrections() const
/*****************************************************************************/
{
  return corrections_;
}

/*****************************************************************************/
void AraelSolver::Clear()
/*****************************************************************************/
{
  corrections_.clear();
}

/*****************************************************************************/
void AraelSolver::Reset()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  corrections_.clear();
  nodes_.clear();
  constraints_.clear();
  first_node_id_ = -1;
}

/*****************************************************************************/
void AraelSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex)
/*****************************************************************************/
{
  if (!pVertex) {
    return;
  }

  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  Eigen::Vector3d pose2d(pose.GetX(), pose.GetY(), pose.GetHeading());

  const int id = pVertex->GetObject()->GetUniqueId();

  boost::mutex::scoped_lock lock(nodes_mutex_);
  nodes_.insert(std::pair<int, Eigen::Vector3d>(id, pose2d));

  if (nodes_.size() == 1) {
    first_node_id_ = id;
  }
}

/*****************************************************************************/
void AraelSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);

  if (!pEdge) {
    return;
  }

  const int node1 = pEdge->GetSource()->GetObject()->GetUniqueId();
  const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();

  if (nodes_.find(node1) == nodes_.end() ||
    nodes_.find(node2) == nodes_.end() || node1 == node2)
  {
    RCLCPP_WARN(
      logger_,
      "AraelSolver: Failed to add constraint, could not find nodes.");
    return;
  }

  // extract transformation
  karto::LinkInfo * pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();

  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix3d information;
  information(0, 0) = precisionMatrix(0, 0);
  information(0, 1) = information(1, 0) = precisionMatrix(0, 1);
  information(0, 2) = information(2, 0) = precisionMatrix(0, 2);
  information(1, 1) = precisionMatrix(1, 1);
  information(1, 2) = information(2, 1) = precisionMatrix(1, 2);
  information(2, 2) = precisionMatrix(2, 2);
  Eigen::Matrix3d sqrt_information = information.llt().matrixU();

  Constraint constraint;
  constraint.a = node1;
  constraint.b = node2;
  constraint.dx = diff.GetX();
  constraint.dy = diff.GetY();
  constraint.dth = diff.GetHeading();
  constraint.sqrt_info[0] = sqrt_information(0, 0);
  constraint.sqrt_info[1] = sqrt_information(0, 1);
  constraint.sqrt_info[2] = sqrt_information(0, 2);
  constraint.sqrt_info[3] = sqrt_information(1, 1);
  constraint.sqrt_info[4] = sqrt_information(1, 2);
  constraint.sqrt_info[5] = sqrt_information(2, 2);

  const bool inserted = constraints_.insert(
    std::pair<size_t, Constraint>(ConstraintHash(node1, node2), constraint)).second;
  if (!inserted) {
    RCLCPP_WARN(
      logger_,
      "AraelSolver: Duplicate constraint %i -> %i dropped.", node1, node2);
  }
}

/*****************************************************************************/
void AraelSolver::RemoveNode(kt_int32s id)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  std::unordered_map<int, Eigen::Vector3d>::iterator nodeit = nodes_.find(id);
  if (nodeit != nodes_.end()) {
    nodes_.erase(nodeit);
  } else {
    RCLCPP_ERROR(
      logger_, "RemoveNode: Failed to find node matching id %i",
      (int)id);
  }
}

/*****************************************************************************/
void AraelSolver::RemoveConstraint(kt_int32s sourceId, kt_int32s targetId)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  if (constraints_.erase(ConstraintHash(sourceId, targetId)) == 0 &&
    constraints_.erase(ConstraintHash(targetId, sourceId)) == 0)
  {
    RCLCPP_ERROR(
      logger_,
      "RemoveConstraint: Failed to find constraint for %i %i",
      (int)sourceId, (int)targetId);
  }
}

/*****************************************************************************/
void AraelSolver::ModifyNode(const int & unique_id, Eigen::Vector3d pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  std::unordered_map<int, Eigen::Vector3d>::iterator it = nodes_.find(unique_id);
  if (it != nodes_.end()) {
    double yaw_init = it->second(2);
    it->second = pose;
    it->second(2) += yaw_init;
  }
}

/*****************************************************************************/
void AraelSolver::GetNodeOrientation(const int & unique_id, double & pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  std::unordered_map<int, Eigen::Vector3d>::iterator it = nodes_.find(unique_id);
  if (it != nodes_.end()) {
    pose = it->second(2);
  }
}

/*****************************************************************************/
std::unordered_map<int, Eigen::Vector3d> * AraelSolver::getGraph()
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(nodes_mutex_);
  return &nodes_;
}

}  // namespace solver_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(solver_plugins::AraelSolver, karto::ScanSolver)

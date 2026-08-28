#ifndef SOLVERS__ARAEL_SOLVER_HPP_
#define SOLVERS__ARAEL_SOLVER_HPP_

#include <math.h>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <memory>
#include "karto_sdk/Mapper.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "slam_toolbox/toolbox_types.hpp"

namespace solver_plugins
{

using namespace ::toolbox_types;  // NOLINT

class AraelSolver : public karto::ScanSolver
{
public:
  AraelSolver();
  virtual ~AraelSolver();

public:
  // Get corrected poses after optimization
  virtual const karto::ScanSolver::IdPoseVector & GetCorrections() const;

  virtual void Compute();  // Solve
  virtual void Clear();  // Resets the corrections
  virtual void Reset();  // Resets the solver plugin clean

  virtual void Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  // Adds a node to the solver
  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> * pVertex);
  // Adds a constraint to the solver
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * pEdge);
  // Get graph stored
  virtual std::unordered_map<int, Eigen::Vector3d> * getGraph();
  // Removes a node from the solver correction table
  virtual void RemoveNode(kt_int32s id);
  // Removes constraints from the optimization problem
  virtual void RemoveConstraint(kt_int32s sourceId, kt_int32s targetId);

  // change a node's pose
  virtual void ModifyNode(const int & unique_id, Eigen::Vector3d pose);
  // get a node's current pose yaw
  virtual void GetNodeOrientation(const int & unique_id, double & pose);
  // solver iterations of the last Compute()
  int last_iterations() const {return last_iterations_;}
  // final cost of the last Compute() (0.5 * sum of squared residuals,
  // the Ceres convention, for comparability)
  double last_cost() const {return last_cost_;}

private:
  // One relative SE2 measurement: measured pose of b in a's frame plus
  // the upper-triangular square root S of the 3x3 information matrix
  // (row major: s00 s01 s02 s11 s12 s22)
  struct Constraint
  {
    int a, b;
    double dx, dy, dth;
    double sqrt_info[6];
  };

  // karto
  karto::ScanSolver::IdPoseVector corrections_;

  // graph
  std::unordered_map<int, Eigen::Vector3d> nodes_;
  std::unordered_map<size_t, Constraint> constraints_;
  int first_node_id_;
  boost::mutex nodes_mutex_;

  bool debug_logging_;
  int last_iterations_;
  double last_cost_;

  // ros
  rclcpp::Logger logger_{rclcpp::get_logger("AraelSolver")};
};

}  // namespace solver_plugins

#endif  // SOLVERS__ARAEL_SOLVER_HPP_

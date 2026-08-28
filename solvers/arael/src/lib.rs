//! 2D pose-graph model for the slam_toolbox arael solver plugin.
//!
//! The constraint reproduces slam_toolbox's Ceres formulation exactly
//! (solvers/ceres_utils.h PoseGraph2dErrorTerm): translation residual in
//! pose a's frame, wrapped heading difference, both scaled by the
//! upper-triangular square root S of the 3x3 information matrix. Total
//! cost is then comparable number for number. The gauge is fixed from
//! the C++ side by freezing the first pose's parameters, matching
//! Ceres' SetParameterBlockConstant.

use arael::angle::AngleParam;
use arael::model::{CrossBlock, Param, SelfBlock};
use arael::refs::{self, Ref};
use arael::vect::vect2d;

/// A 2D pose; the heading is optimized directly as an angle.
#[arael::model]
#[derive(Default)]
pub struct Pose2 {
    pub pos: Param<vect2d>,
    pub rot: AngleParam<f64>,
    pub hb: SelfBlock<Pose2>,
}

/// One relative SE2 measurement of pose b in pose a's frame.
/// (dx, dy, dth) is the measurement; s00..s22 are the entries of the
/// upper-triangular square root of the information matrix, row major.
#[arael::model]
#[arael(constraint(hb, {
    let local = a.rot.rotation_matrix.transpose() * (b.pos - a.pos);
    let r0 = local.x - edge.dx;
    let r1 = local.y - edge.dy;
    let r2 = rad_diff(b.rot.angle, a.rot.angle + edge.dth);
    [edge.s00 * r0 + edge.s01 * r1 + edge.s02 * r2,
     edge.s11 * r1 + edge.s12 * r2,
     edge.s22 * r2]
}))]
#[derive(Default)]
pub struct Edge {
    #[arael(ref = root.poses)]
    pub a: Ref<Pose2>,
    #[arael(ref = root.poses)]
    pub b: Ref<Pose2>,
    pub dx: f64,
    pub dy: f64,
    pub dth: f64,
    pub s00: f64,
    pub s01: f64,
    pub s02: f64,
    pub s11: f64,
    pub s12: f64,
    pub s22: f64,
    pub hb: CrossBlock<Pose2, Pose2>,
}

#[arael::model]
#[arael(root)]
#[derive(Default)]
pub struct Graph {
    pub poses: refs::Vec<Pose2>,
    pub edges: std::vec::Vec<Edge>,
}

# arael solver model

The 2D pose-graph model behind `solvers/arael_solver.cpp`, written
against the [arael](https://crates.io/crates/arael) nonlinear
least-squares library. `src/lib.rs` defines the model (poses, one
relative SE2 constraint reproducing the Ceres plugin's exact residual
and sqrt-information weighting); `capi/` and `cxx/` are the C interface
and C++ headers generated from it by
[cargo-arael](https://crates.io/crates/cargo-arael).

Built only when slam_toolbox is configured with
`-DBUILD_ARAEL_SOLVER=ON`. The build needs a Rust toolchain; cargo
fetches the pinned dependencies (Cargo.lock) from crates.io on the
first build.

After changing `src/lib.rs`, regenerate the interface:

```
cargo install cargo-arael
cargo arael export
```

`test/arael_ceres_equivalence_test.cpp` verifies the arael and Ceres
plugins reach the same optimum; `test/solver_benchmark.cpp` benchmarks
any ScanSolver plugin on 2D pose-graph datasets in g2o format.

## Benchmark

`test/solver_benchmark` on an AMD Ryzen 9 7950X3D, in a ros:lyrical
container (Ubuntu 26.04). Both plugins receive the identical graph through
the ScanSolver interface and minimize the identical objective; the cost
is 0.5 * sum of squared sqrt-information-weighted residuals, as the
plugins compute it. Both run as slam_toolbox ships them: the Ceres plugin
on its configured 50 threads, arael on one, unpinned. Each stops by its
own rule (Ceres at a relative cost decrease of 1e-3 or 50 iterations,
arael's well-conditioned preset), so the final cost stands next to the
time: a plugin that stops early ends higher. Each run is a forked process;
3 rounds, median. Datasets: M3500 and the 2D files of the
[SE-Sync](https://github.com/david-m-rosen/SE-Sync) collection; files
without vertex poses start from the odometry chain, as karto builds it.

```
solver_benchmark data/intel.g2o --reps 5 --warmup 1 --every 100
```

### Batch

The whole graph solved from its initial guess, 5 runs a round, each after
an untimed warm-up solve of the same graph. Peak memory is from a run
without the warm-up. "(cap)" marks a solve that stopped at its plugin's
iteration cap.

| dataset | nodes / edges | backend | time ms | iterations | final cost | peak memory MB |
|---|---|---|---|---|---|---|
| M3500 | 3,500 / 5,453 | arael | 14.5 | 5 | 68.9565 | 45 |
| M3500 | 3,500 / 5,453 | Ceres | 103.7 | 10 | 68.9845 | 57 |
| ais2klinik | 15,115 / 16,727 | arael | 65.2 | 11 | 86.4837 | 76 |
| ais2klinik | 15,115 / 16,727 | Ceres | 523.0 | 22 | 87.3851 | 113 |
| city10000 | 10,000 / 20,687 | arael | 81.0 | 7 | 255.9926 | 76 |
| city10000 | 10,000 / 20,687 | Ceres | 291.5 | 6 | 256.2761 | 112 |
| CSAIL [1] | 1,045 / 1,172 | arael | 2.60 | 5 | 30.3837 | 36 |
| CSAIL [1] | 1,045 / 1,172 | Ceres | 13.8 | 8 | 30.4806 | 41 |
| intel | 1,728 / 2,512 | arael | 4.20 | 3 | 22.2089 | 39 |
| intel | 1,728 / 2,512 | Ceres | 18.0 | 4 | 22.2151 | 46 |
| kitti_00 | 4,541 / 4,679 | arael | 9.40 | 4 | 49.1237 | 46 |
| kitti_00 | 4,541 / 4,679 | Ceres | 85.0 | 13 | 49.1520 | 58 |
| kitti_02 | 4,661 / 4,704 | arael | 10.3 | 5 | 39.4042 | 45 |
| kitti_02 | 4,661 / 4,704 | Ceres | 134.0 | 22 | 39.6120 | 59 |
| kitti_05 | 2,761 / 2,827 | arael | 4.70 | 3 | 78.5419 | 41 |
| kitti_05 | 2,761 / 2,827 | Ceres | 41.8 | 10 | 78.5470 | 49 |
| manhattan | 3,500 / 5,453 | arael | 18.8 | 7 | 1766.7 | 46 |
| manhattan | 3,500 / 5,453 | Ceres | 268.7 | 28 | 1783.3 | 56 |
| MIT | 808 / 827 | arael | 4.50 | 15 | 384.8697 | 36 |
| MIT | 808 / 827 | Ceres | 49.2 | 50 (cap) | 2437.2 | 40 |

### Incremental

The graph grows in file order and is re-solved after every 100
constraints, the way karto re-solves as it maps; the time is the total
over all solves, the cost the one after the last.

| dataset | solves | backend | total time ms | iterations | final cost | peak memory MB |
|---|---|---|---|---|---|---|
| M3500 | 55 | arael | 241.6 | 152 | 68.9565 | 46 |
| M3500 | 55 | Ceres | 1075 | 177 | 68.9731 | 59 |
| ais2klinik | 168 | arael | 2946 | 1106 | 86.4302 | 76 |
| ais2klinik | 168 | Ceres | 5723 | 856 | 149.5471 | 118 |
| city10000 | 207 | arael | 3223 | 544 | 255.9926 | 77 |
| city10000 | 207 | Ceres | 18175 | 698 | 256.0195 | 125 |
| CSAIL [1] | 12 | arael | 9.60 | 30 | 30.3837 | 36 |
| CSAIL [1] | 12 | Ceres | 62.2 | 41 | 30.4480 | 42 |
| intel | 26 | arael | 43.0 | 69 | 22.2089 | 39 |
| intel | 26 | Ceres | 265.1 | 160 | 22.2264 | 46 |
| kitti_00 | 47 | arael | 80.8 | 54 | 49.1237 | 46 |
| kitti_00 | 47 | Ceres | 553.3 | 161 | 49.3843 | 59 |
| kitti_02 | 48 | arael | 83.2 | 71 | 39.4042 | 46 |
| kitti_02 | 48 | Ceres | 363.1 | 52 | 39.6144 | 58 |
| kitti_05 | 29 | arael | 32.7 | 38 | 78.5419 | 41 |
| kitti_05 | 29 | Ceres | 176.4 | 56 | 78.5440 | 50 |
| manhattan | 55 | arael | 210.9 | 126 | 1766.7 | 46 |
| manhattan | 55 | Ceres | 1274 | 351 | 3294.0 | 60 |
| MIT | 9 | arael | 7.30 | 23 | 384.8697 | 36 |
| MIT | 9 | Ceres | 66.5 | 58 | 2425.3 | 40 |

[1] CSAIL's information matrices are near singular. The ScanSolver
interface passes covariances, and the round trip back to information
perturbs them; the costs shown are the ones both plugins minimize, which
differ from a cost computed from the file's own matrices.

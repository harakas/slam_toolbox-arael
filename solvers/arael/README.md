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

`solver_benchmark <file.g2o> --reps 3`, batch solve, median of 3 runs
on one machine. Both plugins receive the identical graph; cost is
0.5 * sum of squared sqrt-information-weighted residuals. The Ceres
plugin runs on 8 threads, arael on one. Each run is a forked child;
maxrss is its peak memory. Datasets from the
[SE-Sync](https://github.com/david-m-rosen/SE-Sync) collection plus
M3500.

| dataset | nodes / edges | Ceres ms / iters | arael ms / iters | final cost Ceres / arael | maxrss MB Ceres / arael |
|---|---|---|---|---|---|
| M3500 | 3500 / 5453 | 85.3 / 10 | 15.3 / 5 | 68.98 / 68.96 | 74.7 / 50.5 |
| ais2klinik | 15115 / 16727 | 362.6 / 22 | 60.4 / 11 | 87.39 / 86.48 | 123.3 / 81.3 |
| city10000 | 10000 / 20687 | 318.0 / 6 | 84.4 / 7 | 256.28 / 255.99 | 122.2 / 80.0 |
| CSAIL [1] | 1045 / 1172 | 11.8 / 8 | 2.7 / 5 | 30.48 / 30.38 | 60.5 / 42.3 |
| intel | 1728 / 2512 | 13.4 / 4 | 4.0 / 3 | 22.215 / 22.209 | 63.6 / 43.6 |
| kitti_00 | 4541 / 4679 | 60.0 / 13 | 7.9 / 4 | 49.152 / 49.152 | 76.2 / 50.6 |
| kitti_02 | 4661 / 4704 | 92.6 / 22 | 9.0 / 5 | 39.61 / 39.40 | 76.2 / 50.7 |
| kitti_05 | 2761 / 2827 | 31.4 / 10 | 4.4 / 3 | 78.547 / 78.542 | 68.5 / 45.9 |
| manhattan | 3500 / 5453 | 230.7 / 28 | 19.4 / 7 | 1783.3 / 1766.7 | 74.8 / 50.4 |
| MIT | 808 / 827 | 42.6 / 50 [2] | 3.4 / 15 | 2437.2 / 384.9 | 59.4 / 40.5 |

[1] Solver-internal costs: CSAIL's near-singular information matrices
make the raw-file reference cost unstable through the interface's
covariance round trip (affects both plugins equally).
[2] Unconverged: hit Ceres' default 50-iteration cap.

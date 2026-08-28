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
plugins reach the same optimum.

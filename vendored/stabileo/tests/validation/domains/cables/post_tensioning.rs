//! Post-tensioning loss checks — engine-coupled.
//! These call the engine's prestress-loss functions instead of recomputing
//! formulas inline (the old version never touched the engine).

use crate::common::assert_close;
use dedaliano_engine::solver::prestress::{
    aci_lump_sum_losses, elastic_shortening_loss, time_dependent_losses,
};

#[test]
fn validation_pt_elastic_shortening_engine() {
    // ES loss = n * f_cgp with Eps = 200 GPa, Eci = 28.6 GPa, f_cgp = 8.11 MPa
    // (pretensioned member, PCI Design Handbook style parameters)
    let loss = elastic_shortening_loss(200_000.0, 28_600.0, 8.11);
    assert_close(loss, 200_000.0 / 28_600.0 * 8.11, 1e-12, "ES = n*f_cgp");
    assert_close(loss, 56.7, 0.01, "ES loss magnitude ~56.7 MPa");
}

#[test]
fn validation_pt_time_dependent_losses_monotonic_engine() {
    // Long-term losses must grow with time and stay physically bounded.
    let loss = |t: f64| {
        time_dependent_losses(
            t, 70.0, 40.0, 32.0, 1400.0, 1670.0, 195_000.0, 30_000.0, true, 200.0,
        )
    };
    let l_30 = loss(30.0);
    let l_365 = loss(365.0);
    let l_70y = loss(25_550.0);
    assert!(l_30 > 0.0, "losses positive at 30 d (got {l_30})");
    assert!(
        l_30 < l_365 && l_365 < l_70y,
        "losses monotonic in time: {l_30} {l_365} {l_70y}"
    );
    assert!(
        l_70y < 0.5 * 1400.0,
        "long-term losses below 50% of f_pi (got {l_70y})"
    );
}

#[test]
fn validation_pt_aci_lump_sum_engine() {
    // ACI 318 lump-sum long-term losses: 240 MPa low-relaxation, 310 MPa stress-relieved.
    assert_close(aci_lump_sum_losses(1400.0, true), 240.0, 1e-12, "low-relax lump sum");
    assert_close(aci_lump_sum_losses(1400.0, false), 310.0, 1e-12, "stress-relieved lump sum");
}

//! EC2 Annex B creep/shrinkage checks — engine-coupled.
//! These call the engine's EC2 implementation instead of reimplementing
//! Annex B inline (the old version had zero `use` statements).

use dedaliano_engine::solver::creep_shrinkage::{
    ec2_creep_coefficient, ec2_shrinkage_strain, ConcreteCreepParams,
};

fn params_c30() -> ConcreteCreepParams {
    // C30/37 indoor member: fcm = 38 MPa, RH 70%, h0 = 200 mm, loaded at 28 d, class N
    ConcreteCreepParams {
        fc: 38.0,
        rh: 70.0,
        h0: 200.0,
        t0: 28.0,
        cement_class: "N".to_string(),
    }
}

#[test]
fn validation_ec2_creep_coefficient_range_and_monotonic() {
    let p = params_c30();
    let phi_1y = ec2_creep_coefficient(&p, 365.0);
    let phi_70y = ec2_creep_coefficient(&p, 25_550.0);
    // EC2 Figure 3.1 places phi(70y, 28d, RH70, h0=200, C30/37) in the ~1.5-2.5 band.
    assert!(phi_70y > 1.4 && phi_70y < 2.6, "phi(70y) out of EC2 band: {phi_70y}");
    assert!(phi_1y < phi_70y, "creep must grow with time: {phi_1y} vs {phi_70y}");
    assert!(ec2_creep_coefficient(&p, 28.0) < phi_1y, "phi grows from t0");
}

#[test]
fn validation_ec2_creep_humidity_effect() {
    // Drier environment => more creep.
    let dry = ConcreteCreepParams { rh: 50.0, ..params_c30() };
    let humid = ConcreteCreepParams { rh: 90.0, ..params_c30() };
    let phi_dry = ec2_creep_coefficient(&dry, 25_550.0);
    let phi_humid = ec2_creep_coefficient(&humid, 25_550.0);
    assert!(phi_dry > phi_humid, "RH50 must creep more than RH90: {phi_dry} vs {phi_humid}");
}

#[test]
fn validation_ec2_shrinkage_range_and_monotonic() {
    let p = params_c30();
    let eps_1y = ec2_shrinkage_strain(&p, 365.0);
    let eps_70y = ec2_shrinkage_strain(&p, 25_550.0);
    // Total shrinkage for C30/37 at RH70 is typically 200-600 microstrain.
    assert!(
        eps_70y.abs() > 100e-6 && eps_70y.abs() < 800e-6,
        "70y shrinkage out of physical band: {eps_70y}"
    );
    assert!(eps_1y.abs() < eps_70y.abs(), "shrinkage must grow with time");
}

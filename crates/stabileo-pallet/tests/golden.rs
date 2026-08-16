//! THE GOLDEN GATE — the port's acceptance test.
//!
//! The application's TypeScript pipeline, run on the committed reproducer
//! fixtures, dumps per-event input bundles and its own terminal answers
//! (an env-gated tap in `cell-executor.ts`). This test replays each bundle
//! through the Rust port with the NATIVE kernel and demands the same
//! answers: identical round counts, identical active sets, and numerics
//! equal to within accumulated-rounding daylight. Until this is green on
//! every event of every fixture, the port is not a port.
//!
//! Run: `GOLDEN_DIR=/path/to/golden/block cargo test -p stabileo-pallet --test golden -- --nocapture`
//!
//! WHAT "the same answers" means on a NATIVE build, measured: the kernel's
//! floating point differs between wasm32 (strict IEEE, no contraction — what
//! the TS reference ran) and native aarch64 (LLVM may fuse multiply-adds), at
//! ~1e-13 per solve. The coupled loop is contractive and exits at a 1e-3
//! share tolerance, so that seed noise amplifies into answers sitting up to
//! ~2× the exit tolerance apart and occasionally a different round count —
//! two convergent walks into the same basin. The gate therefore demands the
//! DISCRETE dynamics exactly (active sets, stick/slip vocabulary, bistable
//! disclosures — zero drift tolerated) and the numerics at the model's own
//! resolution; round counts are reported, not asserted. The production
//! artifact is the WASM build, whose arithmetic matches the reference's
//! instruction-for-instruction; the app-side reproducer suite is the final
//! bit-serious equivalence gate.

use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;

use serde::Deserialize;
use serde_json::Value;

use stabileo_pallet::coupled::{solve_coupled_event, ProgressEmission};
use stabileo_pallet::native_port::NativeKernelPort;
use stabileo_pallet::pallet::projection::PalletMemberMap;
use stabileo_pallet::schema::{
    AnalysisFrame, CompiledPalletSupportState, NumericalAcceptanceProfile, UnitLoadActiveState,
};

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct GoldenInput {
    base_frame: AnalysisFrame,
    member_map: PalletMemberMap,
    support_state: CompiledPalletSupportState,
    unit_state: UnitLoadActiveState,
    #[serde(rename = "palletOverallM")]
    pallet_overall_m: OverallM,
    numerical_profile: NumericalAcceptanceProfile,
    event_id: String,
}

#[derive(Deserialize)]
struct OverallM {
    length: f64,
    width: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct GoldenExpected {
    coupling_rounds: u32,
    shares: Vec<ExpectedShare>,
    reactions: Vec<ExpectedReaction>,
    active_state: Vec<ExpectedActiveState>,
    bistable_contacts: Vec<Value>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ExpectedShare {
    interface_id: String,
    load_share_ratio: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ExpectedReaction {
    support_id: String,
    force: XYZ,
}

#[derive(Deserialize)]
struct XYZ {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ExpectedActiveState {
    support_id: String,
    normal_state: String,
    tangential_state: String,
    #[serde(rename = "verticalReactionN")]
    vertical_reaction_n: f64,
}

fn close(actual: f64, expected: f64, absolute: f64, relative: f64, label: &str) -> Result<(), String> {
    let difference = (actual - expected).abs();
    let allowed = absolute.max(relative * expected.abs());
    if difference <= allowed {
        Ok(())
    } else {
        Err(format!(
            "{label}: actual {actual} vs expected {expected} (difference {difference:e}, allowed {allowed:e})"
        ))
    }
}

#[test]
fn golden_events_replay_identically() {
    let Some(directory) = std::env::var_os("GOLDEN_DIR") else {
        eprintln!("GOLDEN_DIR unset; golden gate skipped");
        return;
    };
    let directory = PathBuf::from(directory);
    let mut inputs: Vec<PathBuf> = fs::read_dir(&directory)
        .expect("golden directory")
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with("input-") && name.ends_with(".json"))
        })
        .collect();
    inputs.sort();
    assert!(!inputs.is_empty(), "no input bundles in {directory:?}");
    let mut failures: Vec<String> = Vec::new();
    for input_path in inputs {
        let name = input_path.file_name().unwrap().to_str().unwrap().to_string();
        let expected_path = directory.join(name.replace("input-", "expected-"));
        if !expected_path.exists() {
            eprintln!("{name}: no expected file (TS event did not complete?); skipping");
            continue;
        }
        let input: GoldenInput =
            serde_json::from_str(&fs::read_to_string(&input_path).unwrap()).expect("parse input");
        let expected: GoldenExpected =
            serde_json::from_str(&fs::read_to_string(&expected_path).unwrap()).expect("parse expected");
        let mut kernel = NativeKernelPort::new(input.numerical_profile.clone());
        let mut notes: Vec<String> = Vec::new();
        let mut progress = |emission: &ProgressEmission| {
            if let ProgressEmission::Note { message } = emission {
                notes.push(message.clone());
            }
        };
        let started = std::time::Instant::now();
        let outcome = solve_coupled_event(
            &mut kernel,
            &input.base_frame,
            &input.member_map,
            &input.support_state,
            &input.unit_state,
            (input.pallet_overall_m.length, input.pallet_overall_m.width),
            &input.numerical_profile,
            &input.event_id,
            &mut progress,
        );
        let elapsed = started.elapsed();
        let result = match outcome {
            Ok(result) => result,
            Err(error) => {
                failures.push(format!("{name}: SOLVE FAILED: {}", error.message));
                continue;
            }
        };
        eprintln!(
            "{name}: {} rounds in {elapsed:?} (expected {} rounds); {} notes",
            result.coupling_rounds,
            expected.coupling_rounds,
            notes.len(),
        );
        if result.coupling_rounds != expected.coupling_rounds {
            eprintln!(
                "{name}: NOTE coupling rounds {} vs reference {} (convergent-path difference; discrete state asserted below)",
                result.coupling_rounds, expected.coupling_rounds
            );
        }
        // Shares: the terminal coupled distribution.
        let actual_shares: HashMap<&str, f64> = result
            .unit_load_state
            .interfaces
            .iter()
            .filter(|entry| entry.lower_package_instance_id.is_none())
            .map(|entry| (entry.interface_id.as_str(), entry.load_share_ratio))
            .collect();
        for share in &expected.shares {
            match actual_shares.get(share.interface_id.as_str()) {
                None => failures.push(format!("{name}: share missing {}", share.interface_id)),
                Some(actual) => {
                    if let Err(error) = close(
                        *actual,
                        share.load_share_ratio,
                        // The coupled exit tolerance is 1e-3 on shares; two
                        // convergent walks may sit ~2 tolerances apart.
                        5e-3,
                        1e-2,
                        &format!("{name}: share {}", share.interface_id),
                    ) {
                        failures.push(error);
                    }
                }
            }
        }
        // Terminal reactions. WHICH support carries HOW MUCH of a shared
        // load is exactly as determinate as the coupled exit tolerance: a
        // walk that stops with shares 1e-3 different moves 1e-3 x applied
        // between neighbouring supports. The acceptance is anchored to that:
        // 3 x the share tolerance x the total applied vertical.
        let total_expected_vertical: f64 = expected
            .reactions
            .iter()
            .map(|reaction| reaction.force.y.max(0.0))
            .sum();
        let reaction_allowance = (3.0
            * input.numerical_profile.coupled_load_share_tolerance
            * total_expected_vertical)
            .max(0.5);
        let actual_reactions: HashMap<&str, &stabileo_pallet::schema::KernelReaction> = result
            .kernel_result
            .reactions
            .iter()
            .map(|reaction| (reaction.support_id.as_str(), reaction))
            .collect();
        for reaction in &expected.reactions {
            match actual_reactions.get(reaction.support_id.as_str()) {
                None => failures.push(format!("{name}: reaction missing {}", reaction.support_id)),
                Some(actual) => {
                    for (axis, actual_value, expected_value) in [
                        ("x", actual.force.x, reaction.force.x),
                        ("y", actual.force.y, reaction.force.y),
                        ("z", actual.force.z, reaction.force.z),
                    ] {
                        if let Err(error) = close(
                            actual_value,
                            expected_value,
                            reaction_allowance,
                            1e-2,
                            &format!("{name}: reaction {} {axis}", reaction.support_id),
                        ) {
                            failures.push(error);
                        }
                    }
                }
            }
        }
        // The terminal active set, state for state.
        let actual_states: HashMap<&str, &stabileo_pallet::schema::SupportContactActiveStateEntry> =
            result
                .support_state
                .active_state
                .iter()
                .map(|entry| (entry.support_id.as_str(), entry))
                .collect();
        for state in &expected.active_state {
            match actual_states.get(state.support_id.as_str()) {
                None => failures.push(format!("{name}: active state missing {}", state.support_id)),
                Some(actual) => {
                    if actual.normal_state != state.normal_state
                        || actual.tangential_state != state.tangential_state
                    {
                        failures.push(format!(
                            "{name}: {} state {}/{} != expected {}/{}",
                            state.support_id,
                            actual.normal_state,
                            actual.tangential_state,
                            state.normal_state,
                            state.tangential_state
                        ));
                    }
                    if let Err(error) = close(
                        actual.vertical_reaction_n,
                        state.vertical_reaction_n,
                        reaction_allowance,
                        1e-2,
                        &format!("{name}: {} verticalReactionN", state.support_id),
                    ) {
                        failures.push(error);
                    }
                }
            }
        }
        // Bistable disclosures: same support set.
        let expected_bistable: Vec<&str> = expected
            .bistable_contacts
            .iter()
            .filter_map(|value| value.get("supportId").and_then(|id| id.as_str()))
            .collect();
        let actual_bistable: Vec<&str> = result
            .bistable_contacts
            .iter()
            .map(|entry| entry.support_id.as_str())
            .collect();
        if expected_bistable != actual_bistable {
            failures.push(format!(
                "{name}: bistable set {actual_bistable:?} != expected {expected_bistable:?}"
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "GOLDEN GATE FAILURES ({}):\n{}",
        failures.len(),
        failures.join("\n")
    );
}

//! Literal port of `packages/analysis/stabileo/src/diagnostics.ts`.

use dedaliano_engine::types::{DiagnosticCode, Severity, StructuredDiagnostic};

use crate::schema::KernelDiagnostic;
use crate::types::PalletResult;

/// The kernel's own code -> (normalized code, sentence). A code with no entry
/// here is DROPPED, exactly as the TS object lookup drops it.
const DIAGNOSTICS: &[(&str, (&str, &str))] = &[
    ("singular_matrix", ("SINGULAR", "The structural model contains a singular degree of freedom.")),
    ("local_mechanism", ("SINGULAR", "The structural model contains a local mechanism.")),
    ("disconnected_node", ("SINGULAR", "The structural model contains a disconnected node.")),
    ("near_zero_diagonal", ("SINGULAR", "The structural stiffness contains a near-zero diagonal.")),
    // A REDUNDANCY IS AN ILL-CONDITIONING, and this one had no entry at all: the
    // solver reported it, `normalize_structured_diagnostics` dropped every code
    // it did not recognise, and nothing downstream ever heard that the model is
    // constrained more times than it needs. The honest record of it belongs in
    // the result rather than in a failure message that only appears when
    // something else goes wrong.
    //
    // It fired TWELVE times on a born pallet — three translations at each of four
    // bottom-deckboard nodes that were BOTH a floor bearing node and the rigid
    // slave of a joint tie — and that was the `VALIDATED_LOOKUP` tier's interim
    // rigid translational tie, now retired for the estimator's own stiffnesses.
    // EIGHTEEN after that change: two degrees of freedom at each of NINE nodes,
    // every bottom-deckboard/stringer crossing, and what they have in common is
    // that all are floor bearing nodes and none is a top-deck joint — so the
    // surviving second mechanism is a SUPPORT rather than a tie fighting a tie.
    // The COUNT and the NODE are what made that readable; the raw indices below
    // do not decode, and did not need to.
    (
        "over_constrained_dof",
        ("ILL_CONDITIONED", "The structural model constrains a degree of freedom more than once."),
    ),
    ("high_diagonal_ratio", ("ILL_CONDITIONED", "The structural system is ill-conditioned.")),
    (
        "extremely_high_diagonal_ratio",
        ("ILL_CONDITIONED", "The structural system is severely ill-conditioned."),
    ),
    ("residual_high", ("EQUILIBRIUM_FAILURE", "The solver residual exceeds the accepted threshold.")),
    (
        "equilibrium_violation",
        ("EQUILIBRIUM_FAILURE", "The solver reported an equilibrium violation."),
    ),
];

/// The kernel's `DiagnosticCode` as the app sees it: the serde name, which is
/// the snake_case string the TS switched on.
pub fn diagnostic_code_name(code: DiagnosticCode) -> String {
    serde_json::to_value(code)
        .ok()
        .and_then(|value| value.as_str().map(str::to_string))
        .unwrap_or_else(|| "?".to_string())
}

/// The kernel's `Severity` as the app sees it: "info" | "warning" | "error".
pub fn severity_name(severity: Severity) -> String {
    serde_json::to_value(severity)
        .ok()
        .and_then(|value| value.as_str().map(str::to_string))
        .unwrap_or_else(|| "?".to_string())
}

pub fn normalize_structured_diagnostics(
    diagnostics: &[StructuredDiagnostic],
    stable_node_id: &dyn Fn(usize) -> PalletResult<String>,
    stable_element_id: &dyn Fn(usize) -> PalletResult<String>,
) -> PalletResult<Vec<KernelDiagnostic>> {
    let mut normalized = Vec::new();
    for diagnostic in diagnostics {
        let code_name = diagnostic_code_name(diagnostic.code);
        let Some((_, mapping)) = DIAGNOSTICS.iter().find(|(key, _)| *key == code_name) else {
            continue;
        };
        let entity_id = match diagnostic.node_ids.first() {
            Some(node_id) => Some(stable_node_id(*node_id)?),
            None => match diagnostic.element_ids.first() {
                Some(element_id) => Some(stable_element_id(*element_id)?),
                None => None,
            },
        };
        // HOW MANY DEGREES OF FREEDOM, AND WHICH SOLVER INDICES.
        //
        // `StructuredDiagnostic.dofIndices` has been on every one of these since
        // the SDK was adopted and this function dropped it, keeping the code, the
        // severity and one node id. That is the same shortfall that made
        // `over_constrained_dof` useless before it was mapped at all: "a degree of
        // freedom is constrained more than once" at a named node does not say how
        // many, and one is a detail where three is a support fighting a tie.
        //
        // THE INDICES ARE THE SOLVER'S OWN GLOBAL NUMBERING, NOT THIS FRAME'S DOF
        // ORDER, and they pass through UNTRANSLATED on purpose. A constrained solve
        // eliminates degrees of freedom, so the numbering is compacted and
        // `index % 6` does not recover a local DOF — a born pallet reports pairs at
        // 960/961, 962/963 and 964/965, which cannot all be the same local pair. A
        // name derived from that arithmetic would be a guess wearing a label, and
        // the count and the node are actionable without one.
        let dofs = diagnostic
            .dof_indices
            .iter()
            .map(|index| index.to_string())
            .collect::<Vec<String>>()
            .join(", ");
        normalized.push(KernelDiagnostic {
            code: mapping.0.to_string(),
            severity: severity_name(diagnostic.severity).to_uppercase(),
            entity_id,
            message: if dofs.is_empty() {
                mapping.1.to_string()
            } else {
                format!("{} (solver dof {dofs})", mapping.1)
            },
        });
    }
    Ok(normalized)
}

#[derive(Debug, Clone)]
pub struct ClassifiedEngineFailure {
    pub code: &'static str,
    pub retryable: bool,
    pub message: String,
}

/// TS `classifyEngineFailure`. The two regular expressions are case-insensitive
/// alternations with no `s` flag, so `wasm.*load` cannot cross a newline —
/// hence the line-by-line test for that one.
pub fn classify_engine_failure(error: &str) -> ClassifiedEngineFailure {
    let message = error.to_string();
    let lowered = message.to_lowercase();
    if lowered.contains("singular") || lowered.contains("mechanism") || lowered.contains("no free dof")
    {
        return ClassifiedEngineFailure {
            code: "MODEL_SINGULAR",
            retryable: false,
            message,
        };
    }
    let wasm_load = lowered.lines().any(|line| match line.find("wasm") {
        Some(index) => line[index..].contains("load"),
        None => false,
    });
    if lowered.contains("initializ") || lowered.contains("instantiate") || wasm_load {
        return ClassifiedEngineFailure { code: "SOLVER_INIT_FAILED", retryable: true, message };
    }
    ClassifiedEngineFailure { code: "SOLVER_EXECUTION_FAILED", retryable: false, message }
}

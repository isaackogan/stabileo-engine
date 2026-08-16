//! Request/response types for the coupled pallet solve. Placeholder while the
//! orchestration port lands module by module.

use serde::{Deserialize, Serialize};

/// Version discriminator for the coupled-solve request envelope.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoupledSolveVersionProbe {
    #[serde(rename = "schemaVersion")]
    pub schema_version: String,
}

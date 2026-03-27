use thiserror::Error;

/// Unified error type for the antmap engine.
#[derive(Debug, Error)]
pub enum AntmapError {
    // --- OSM / Network ---
    #[error("HTTP request failed: {0}")]
    Http(#[from] reqwest::Error),

    #[error("JSON parsing failed: {0}")]
    Json(#[from] serde_json::Error),

    #[error("OSM data is malformed or missing required fields: {0}")]
    OsmMalformed(String),

    // --- Graph ---
    #[error("Node with OSM id {0} not found in graph index")]
    NodeNotFound(i64),

    #[error("No path found between node {from} and node {to}")]
    NoPathFound { from: i64, to: i64 },

    // --- ACO ---
    #[error("ACO configuration is invalid: {0}")]
    InvalidConfig(String),

    // --- I/O ---
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

/// Convenient Result alias used throughout the crate.
pub type Result<T> = std::result::Result<T, AntmapError>;

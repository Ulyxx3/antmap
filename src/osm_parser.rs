//! # OSM Parser
//!
//! Converts raw OpenStreetMap data into the engine's typed structures
//! ([`OsmNode`] and [`OsmWay`]).
//!
//! ## Data sources
//!
//! Two entry points are provided:
//!
//! - [`fetch_from_overpass`] — sends an Overpass QL query over HTTP and parses
//!   the JSON response on the fly.
//! - [`load_from_file`] — reads a previously saved Overpass JSON file from
//!   disk. Useful for reproducible runs and offline development.
//!
//! Both return the same [`OsmData`] type consumed by `graph_builder`.

use std::collections::HashMap;
use std::path::Path;

use log::{debug, info, warn};
use serde::Deserialize;

use crate::error::{AntmapError, Result};
use crate::types::{BoundingBox, HighwayType, OsmData, OsmNode, OsmWay};

// ---------------------------------------------------------------------------
// Overpass API
// ---------------------------------------------------------------------------

/// Default public Overpass API endpoint.
pub const DEFAULT_OVERPASS_URL: &str = "https://overpass-api.de/api/interpreter";

/// Build the Overpass QL query for all routable ways inside `bbox`.
///
/// The `>;` operator expands every way into its constituent nodes so we
/// receive full coordinates in a single request.
fn build_overpass_query(bbox: &BoundingBox) -> String {
    let filter = bbox.to_overpass_filter();
    format!(
        r#"[out:json][timeout:120];
(
  way["highway"]({filter});
  >;
);
out body;"#,
        filter = filter
    )
}

/// Query the Overpass API and return parsed [`OsmData`].
///
/// # Errors
///
/// - [`AntmapError::Http`]  — network or HTTP error.
/// - [`AntmapError::Json`]  — malformed response from the server.
/// - [`AntmapError::OsmMalformed`] — data passes JSON parsing but is
///   semantically invalid (e.g. a way references a node not in the response).
pub fn fetch_from_overpass(bbox: BoundingBox, overpass_url: &str) -> Result<OsmData> {
    let query = build_overpass_query(&bbox);
    info!(
        "Querying Overpass at {} for bbox {}",
        overpass_url,
        bbox.to_overpass_filter()
    );

    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(180))
        .user_agent("antmap/0.1 (https://github.com/Ulyxx3/antmap)")
        .build()?;

    let response = client
        .post(overpass_url)
        .body(query)
        .send()?
        .error_for_status()?;

    let text = response.text()?;
    info!("Received {} bytes from Overpass", text.len());

    parse_overpass_json(&text)
}

/// Load an Overpass JSON file from disk and return parsed [`OsmData`].
///
/// Useful for offline development or reproducible benchmarks — download once,
/// run many times.
///
/// # Errors
///
/// - [`AntmapError::Io`]   — file not found or unreadable.
/// - [`AntmapError::Json`] — invalid JSON.
/// - [`AntmapError::OsmMalformed`] — semantically invalid OSM data.
pub fn load_from_file(path: &Path) -> Result<OsmData> {
    info!("Loading OSM data from {}", path.display());
    let text = std::fs::read_to_string(path)?;
    parse_overpass_json(&text)
}

// ---------------------------------------------------------------------------
// JSON deserialization
// ---------------------------------------------------------------------------

/// Top-level Overpass API response envelope.
#[derive(Deserialize)]
struct OverpassResponse {
    elements: Vec<RawElement>,
}

/// A single JSON element. Both nodes and ways live in the same `elements`
/// array; we discriminate on the `type` field.
#[derive(Deserialize)]
struct RawElement {
    #[serde(rename = "type")]
    element_type: ElementKind,
    id: i64,
    // Node-specific fields
    lat: Option<f64>,
    lon: Option<f64>,
    // Way-specific fields
    nodes: Option<Vec<i64>>,
    tags: Option<HashMap<String, String>>,
}

#[derive(Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
enum ElementKind {
    Node,
    Way,
    Relation,
}

// ---------------------------------------------------------------------------
// Core parsing logic
// ---------------------------------------------------------------------------

/// Parse a raw Overpass JSON string into [`OsmData`].
pub fn parse_overpass_json(json: &str) -> Result<OsmData> {
    let response: OverpassResponse = serde_json::from_str(json)?;

    let mut raw_nodes: Vec<OsmNode> = Vec::new();
    let mut raw_ways: Vec<OsmWay> = Vec::new();

    for element in response.elements {
        match element.element_type {
            ElementKind::Node => {
                let lat = element.lat.ok_or_else(|| {
                    AntmapError::OsmMalformed(format!("Node {} missing lat", element.id))
                })?;
                let lon = element.lon.ok_or_else(|| {
                    AntmapError::OsmMalformed(format!("Node {} missing lon", element.id))
                })?;
                raw_nodes.push(OsmNode {
                    id: element.id,
                    lat,
                    lon,
                });
            }

            ElementKind::Way => {
                let Some(node_refs) = element.nodes else {
                    warn!("Way {} has no node list, skipping", element.id);
                    continue;
                };
                if node_refs.len() < 2 {
                    warn!("Way {} has fewer than 2 nodes, skipping", element.id);
                    continue;
                }

                let tags = element.tags.unwrap_or_default();

                // highway tag is required for routing; skip non-highway ways
                let Some(highway_str) = tags.get("highway") else {
                    debug!("Way {} has no highway tag, skipping", element.id);
                    continue;
                };

                let highway = parse_highway_type(highway_str);

                // Skip highway types not suitable for motor vehicles.
                if !is_routable(&highway) {
                    debug!(
                        "Way {} has non-routable highway type {:?}, skipping",
                        element.id, highway
                    );
                    continue;
                }

                let max_speed_kmh = tags
                    .get("maxspeed")
                    .map(|s| parse_maxspeed(s, highway))
                    .unwrap_or_else(|| highway.default_speed_kmh());

                let is_oneway = parse_oneway(tags.get("oneway").map(String::as_str));

                raw_ways.push(OsmWay {
                    id: element.id,
                    node_refs,
                    highway,
                    max_speed_kmh,
                    is_oneway,
                });
            }

            // Relations are not used for routing at this stage.
            ElementKind::Relation => {}
        }
    }

    info!(
        "Parsed {} nodes and {} ways",
        raw_nodes.len(),
        raw_ways.len()
    );

    Ok(OsmData {
        nodes: raw_nodes,
        ways: raw_ways,
    })
}

// ---------------------------------------------------------------------------
// Tag parsers
// ---------------------------------------------------------------------------

/// Convert an OSM `highway` tag value to our typed enum.
fn parse_highway_type(value: &str) -> HighwayType {
    match value {
        "motorway" => HighwayType::Motorway,
        "motorway_link" => HighwayType::MotorwayLink,
        "trunk" => HighwayType::Trunk,
        "trunk_link" => HighwayType::TrunkLink,
        "primary" => HighwayType::Primary,
        "primary_link" => HighwayType::PrimaryLink,
        "secondary" => HighwayType::Secondary,
        "secondary_link" => HighwayType::SecondaryLink,
        "tertiary" => HighwayType::Tertiary,
        "tertiary_link" => HighwayType::TertiaryLink,
        "residential" => HighwayType::Residential,
        "living_street" => HighwayType::LivingStreet,
        "service" => HighwayType::Service,
        "track" => HighwayType::Track,
        "path" => HighwayType::Path,
        "cycleway" => HighwayType::Cycleway,
        "footway" => HighwayType::Footway,
        "unclassified" => HighwayType::Unclassified,
        _ => HighwayType::Other,
    }
}

/// Returns `true` for highway types usable by motor vehicles.
/// Uses a strict whitelist to ensure pedestrian squares, steps, and construction aren't included.
fn is_routable(h: &HighwayType) -> bool {
    matches!(
        h,
        HighwayType::Motorway
            | HighwayType::MotorwayLink
            | HighwayType::Trunk
            | HighwayType::TrunkLink
            | HighwayType::Primary
            | HighwayType::PrimaryLink
            | HighwayType::Secondary
            | HighwayType::SecondaryLink
            | HighwayType::Tertiary
            | HighwayType::TertiaryLink
            | HighwayType::Residential
            | HighwayType::LivingStreet
            | HighwayType::Unclassified
    )
}

/// Parse an OSM `maxspeed` tag value into km/h.
///
/// Handles all common formats:
///
/// | Tag value       | Interpretation          |
/// |-----------------|-------------------------|
/// | `"50"`          | 50 km/h (plain integer) |
/// | `"50 mph"`      | 50 × 1.60934 ≈ 80 km/h |
/// | `"FR:urban"`    | 50 km/h                 |
/// | `"FR:rural"`    | 80 km/h                 |
/// | `"FR:motorway"` | 130 km/h                |
/// | `"walk"/"foot"` | 7 km/h                  |
/// | `"none"`        | highway class default   |
///
/// Falls back to `highway.default_speed_kmh()` on unrecognised values.
fn parse_maxspeed(value: &str, highway: HighwayType) -> f64 {
    let value = value.trim();

    // 1. Plain integer in km/h — most common case
    if let Ok(speed) = value.parse::<f64>() {
        return speed.max(1.0); // guard against 0 km/h divisions
    }

    // 2. Value with "mph" suffix  (e.g. "30 mph", "30mph")
    let without_mph = value
        .strip_suffix("mph")
        .or_else(|| value.strip_suffix(" mph"))
        .map(str::trim);

    if let Some(mph_str) = without_mph {
        if let Ok(mph) = mph_str.parse::<f64>() {
            return (mph * 1.609_344).max(1.0);
        }
    }

    // 3. Country-qualified tags (e.g. "FR:urban", "DE:rural")
    //    We ignore the country prefix and look at the road class suffix only.
    let class_part = value.find(':').map(|i| &value[i + 1..]).unwrap_or(value);

    match class_part.to_lowercase().as_str() {
        "urban" | "city" => 50.0,
        "rural" | "country" => 80.0,
        "motorway" => 130.0,
        "trunk" => 110.0,
        "living_street" => 10.0,
        "walk" | "pedestrian" | "foot" => 7.0,
        "none" | "unlimited" => highway.default_speed_kmh(),
        _ => {
            warn!(
                "Unrecognised maxspeed value {:?}, falling back to highway default ({} km/h)",
                value,
                highway.default_speed_kmh()
            );
            highway.default_speed_kmh()
        }
    }
}

/// Parse an OSM `oneway` tag.
///
/// Returns `true` for `yes`, `1`, `true`; returns `false` otherwise
/// (including `-1`, which means reversed — we treat it as bidirectional
/// for now since graph_builder handles direction from node order).
fn parse_oneway(value: Option<&str>) -> bool {
    matches!(value, Some("yes") | Some("1") | Some("true"))
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- maxspeed ---

    #[test]
    fn parse_plain_kmh() {
        assert_eq!(parse_maxspeed("50", HighwayType::Residential), 50.0);
        assert_eq!(parse_maxspeed("130", HighwayType::Motorway), 130.0);
    }

    #[test]
    fn parse_mph() {
        let result = parse_maxspeed("30 mph", HighwayType::Residential);
        // 30 × 1.609344 ≈ 48.28
        assert!((result - 48.28).abs() < 0.1, "got {}", result);
    }

    #[test]
    fn parse_country_urban() {
        assert_eq!(parse_maxspeed("FR:urban", HighwayType::Primary), 50.0);
        assert_eq!(parse_maxspeed("DE:urban", HighwayType::Primary), 50.0);
    }

    #[test]
    fn parse_country_rural() {
        assert_eq!(parse_maxspeed("FR:rural", HighwayType::Primary), 80.0);
    }

    #[test]
    fn parse_none_falls_back_to_highway_default() {
        let default = HighwayType::Motorway.default_speed_kmh();
        assert_eq!(parse_maxspeed("none", HighwayType::Motorway), default);
    }

    // --- oneway ---

    #[test]
    fn oneway_yes() {
        assert!(parse_oneway(Some("yes")));
        assert!(parse_oneway(Some("1")));
        assert!(parse_oneway(Some("true")));
    }

    #[test]
    fn oneway_no() {
        assert!(!parse_oneway(Some("no")));
        assert!(!parse_oneway(Some("-1")));
        assert!(!parse_oneway(None));
    }

    // --- highway ---

    #[test]
    fn highway_roundtrip() {
        assert_eq!(parse_highway_type("motorway"), HighwayType::Motorway);
        assert_eq!(parse_highway_type("residential"), HighwayType::Residential);
        assert_eq!(parse_highway_type("unknown_tag"), HighwayType::Other);
    }

    // --- JSON parsing ---

    #[test]
    fn parse_minimal_json() {
        let json = r#"
        {
            "elements": [
                {"type":"node","id":1,"lat":48.856,"lon":2.352},
                {"type":"node","id":2,"lat":48.857,"lon":2.353},
                {
                    "type":"way","id":10,
                    "nodes":[1,2],
                    "tags":{"highway":"primary","maxspeed":"50","oneway":"yes"}
                }
            ]
        }"#;

        let data = parse_overpass_json(json).expect("should parse");
        assert_eq!(data.nodes.len(), 2);
        assert_eq!(data.ways.len(), 1);
        assert_eq!(data.ways[0].max_speed_kmh, 50.0);
        assert!(data.ways[0].is_oneway);
        assert_eq!(data.ways[0].highway, HighwayType::Primary);
    }

    #[test]
    fn footway_is_excluded() {
        let json = r#"
        {
            "elements": [
                {"type":"node","id":1,"lat":48.0,"lon":2.0},
                {"type":"node","id":2,"lat":48.1,"lon":2.1},
                {
                    "type":"way","id":99,
                    "nodes":[1,2],
                    "tags":{"highway":"footway"}
                }
            ]
        }"#;

        let data = parse_overpass_json(json).expect("should parse");
        // Footways must be excluded from the routing graph
        assert_eq!(data.ways.len(), 0);
    }
}

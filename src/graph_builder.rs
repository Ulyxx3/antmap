//! # Graph Builder
//!
//! Converts [`OsmData`] (the output of `osm_parser`) into a
//! [`petgraph::DiGraph`] where:
//!
//! - **Nodes** carry [`NodeData`] (OSM id, lat, lon).
//! - **Edges** carry [`EdgeData`] (travel time, distance, highway type, speed).
//!
//! Edge weights represent the ACO heuristic η = 1 / travel_time_s,
//! computed here as:
//!
//! ```text
//! travel_time_s = distance_m / (max_speed_kmh / 3.6) + highway_penalty_s
//! ```
//!
//! Bidirectional ways produce two directed edges (A→B and B→A).
//! `oneway` ways produce only the forward edge.

use std::collections::{HashMap, HashSet};

use log::{debug, info, warn};
use petgraph::graph::{DiGraph, NodeIndex};

use crate::error::{AntmapError, Result};
use crate::types::{EdgeData, NodeData, OsmData};

// ---------------------------------------------------------------------------
// Public output type
// ---------------------------------------------------------------------------

/// The road network represented as a directed weighted graph.
///
/// Use [`build_graph`] to construct this from [`OsmData`].
#[derive(Debug)]
pub struct RoadGraph {
    /// Directed graph where node/edge weights are our typed structs.
    pub graph: DiGraph<NodeData, EdgeData>,

    /// Maps every OSM node id that appears in the graph to its
    /// [`NodeIndex`], enabling O(1) lookups by OSM id.
    pub node_index: HashMap<i64, NodeIndex>,
}

impl RoadGraph {
    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    /// Look up a [`NodeIndex`] by OSM node id.
    ///
    /// # Errors
    /// Returns [`AntmapError::NodeNotFound`] if the id is not in the graph.
    pub fn get_node(&self, osm_id: i64) -> Result<NodeIndex> {
        self.node_index
            .get(&osm_id)
            .copied()
            .ok_or(AntmapError::NodeNotFound(osm_id))
    }

    /// Find the graph node closest (by Haversine distance) to the given
    /// GPS coordinates.
    ///
    /// Returns `None` only if the graph contains no nodes.
    pub fn nearest_node(&self, lat: f64, lon: f64) -> Option<NodeIndex> {
        self.graph.node_indices().min_by(|&a, &b| {
            let nd_a = &self.graph[a];
            let nd_b = &self.graph[b];
            let da = haversine_m(lat, lon, nd_a.lat, nd_a.lon);
            let db = haversine_m(lat, lon, nd_b.lat, nd_b.lon);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Total number of nodes.
    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }

    /// Total number of directed edges.
    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Build a [`RoadGraph`] from parsed OSM data.
///
/// # Algorithm
///
/// 1. Index all OSM nodes by id for O(1) lookups.
/// 2. Collect the set of node ids that are actually referenced by routable
///    ways (avoids inserting unused nodes).
/// 3. Insert those nodes as graph vertices.
/// 4. For every consecutive node pair `(A, B)` in each way:
///    - Compute Haversine distance and travel time.
///    - Insert a directed edge A → B.
///    - If the way is **not** oneway, also insert B → A.
///
/// # Errors
///
/// Returns an error only if the internal graph state is irrecoverable.
/// Missing node references (way references a node absent from the node list)
/// produce a `warn!` log and are silently skipped.
pub fn build_graph(osm: &OsmData) -> Result<RoadGraph> {
    // ----------------------------------------------------------------
    // Step 1 — index raw nodes by OSM id
    // ----------------------------------------------------------------
    let raw_node_map: HashMap<i64, (f64, f64)> =
        osm.nodes.iter().map(|n| (n.id, (n.lat, n.lon))).collect();

    // ----------------------------------------------------------------
    // Step 2 — collect only nodes referenced by ways
    // ----------------------------------------------------------------
    let referenced_ids: HashSet<i64> = osm
        .ways
        .iter()
        .flat_map(|w| w.node_refs.iter().copied())
        .collect();

    info!(
        "OSM data: {} total nodes, {} referenced by ways, {} ways",
        raw_node_map.len(),
        referenced_ids.len(),
        osm.ways.len()
    );

    // ----------------------------------------------------------------
    // Step 3 — create graph vertices for referenced nodes
    // ----------------------------------------------------------------
    let mut graph: DiGraph<NodeData, EdgeData> =
        DiGraph::with_capacity(referenced_ids.len(), osm.ways.len() * 2);
    let mut node_index: HashMap<i64, NodeIndex> = HashMap::with_capacity(referenced_ids.len());

    for osm_id in &referenced_ids {
        match raw_node_map.get(osm_id) {
            Some(&(lat, lon)) => {
                let idx = graph.add_node(NodeData {
                    osm_id: *osm_id,
                    lat,
                    lon,
                });
                node_index.insert(*osm_id, idx);
            }
            None => {
                warn!(
                    "Node {} is referenced by a way but missing from the \
                     node list (incomplete Overpass response?)",
                    osm_id
                );
            }
        }
    }

    // ----------------------------------------------------------------
    // Step 4 — add directed edges for every way segment
    // ----------------------------------------------------------------
    let mut edges_added = 0usize;
    let mut edges_skipped = 0usize;

    for way in &osm.ways {
        for segment in way.node_refs.windows(2) {
            let (from_id, to_id) = (segment[0], segment[1]);

            let (Some(&from_idx), Some(&to_idx)) =
                (node_index.get(&from_id), node_index.get(&to_id))
            else {
                debug!(
                    "Skipping segment {}->{} in way {}: node(s) absent from graph",
                    from_id, to_id, way.id
                );
                edges_skipped += 1;
                continue;
            };

            // -- Geometry --
            let from_data = &graph[from_idx];
            let to_data = &graph[to_idx];
            let distance_m = haversine_m(from_data.lat, from_data.lon, to_data.lat, to_data.lon);

            // -- Travel time (core ACO heuristic) --
            // speed_ms : max speed in m/s
            let speed_ms = (way.max_speed_kmh / 3.6).max(f64::EPSILON);
            let travel_time_s = distance_m / speed_ms + way.highway.time_penalty_seconds();

            let edge = EdgeData {
                osm_way_id: way.id,
                distance_m,
                travel_time_s,
                highway: way.highway,
                max_speed_kmh: way.max_speed_kmh,
            };

            // Forward edge (A → B) — always present
            graph.add_edge(from_idx, to_idx, edge.clone());
            edges_added += 1;

            // Reverse edge (B → A) — only for bidirectional ways
            if !way.is_oneway {
                graph.add_edge(to_idx, from_idx, edge);
                edges_added += 1;
            }
        }
    }

    info!(
        "Graph built: {} nodes, {} directed edges ({} segments skipped)",
        graph.node_count(),
        edges_added,
        edges_skipped,
    );

    Ok(RoadGraph { graph, node_index })
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Haversine great-circle distance between two GPS coordinates, in metres.
///
/// Accurate to ≈ 0.5 % for the distances encountered in urban road graphs.
pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R: f64 = 6_371_000.0; // mean Earth radius in metres

    let dlat = (lat2 - lat1).to_radians();
    let dlon = (lon2 - lon1).to_radians();
    let lat1_r = lat1.to_radians();
    let lat2_r = lat2.to_radians();

    let a = (dlat / 2.0).sin().powi(2) + lat1_r.cos() * lat2_r.cos() * (dlon / 2.0).sin().powi(2);

    let c = 2.0 * a.sqrt().asin(); // numerically stable form
    R * c
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{HighwayType, OsmNode, OsmWay};

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    fn make_osm_data(ways: Vec<OsmWay>, nodes: Vec<OsmNode>) -> OsmData {
        OsmData { nodes, ways }
    }

    fn simple_node(id: i64, lat: f64, lon: f64) -> OsmNode {
        OsmNode { id, lat, lon }
    }

    fn simple_way(id: i64, node_refs: Vec<i64>, oneway: bool) -> OsmWay {
        OsmWay {
            id,
            node_refs,
            highway: HighwayType::Primary,
            max_speed_kmh: 50.0,
            is_oneway: oneway,
        }
    }

    // ------------------------------------------------------------------
    // Haversine
    // ------------------------------------------------------------------

    #[test]
    fn haversine_same_point_is_zero() {
        assert_eq!(haversine_m(48.856, 2.352, 48.856, 2.352), 0.0);
    }

    #[test]
    fn haversine_paris_to_london_approx() {
        // Paris (48.8566, 2.3522) → London (51.5074, -0.1278) ≈ 340 km
        let d = haversine_m(48.8566, 2.3522, 51.5074, -0.1278);
        assert!(
            (d - 340_000.0).abs() < 5_000.0,
            "Expected ~340 km, got {:.0} m",
            d
        );
    }

    #[test]
    fn haversine_short_segment_reasonable() {
        // Two points ~111 m apart (≈ 0.001° lat)
        let d = haversine_m(48.000, 2.000, 48.001, 2.000);
        assert!((d - 111.0).abs() < 5.0, "Expected ~111 m, got {:.1} m", d);
    }

    // ------------------------------------------------------------------
    // Graph construction
    // ------------------------------------------------------------------

    #[test]
    fn bidirectional_way_adds_two_edges() {
        let osm = make_osm_data(
            vec![simple_way(1, vec![10, 20], false)],
            vec![simple_node(10, 48.0, 2.0), simple_node(20, 48.001, 2.0)],
        );
        let g = build_graph(&osm).expect("build_graph failed");
        assert_eq!(g.node_count(), 2);
        assert_eq!(
            g.edge_count(),
            2,
            "Expected 2 edges for a bidirectional way"
        );
    }

    #[test]
    fn oneway_adds_single_edge() {
        let osm = make_osm_data(
            vec![simple_way(1, vec![10, 20], true)],
            vec![simple_node(10, 48.0, 2.0), simple_node(20, 48.001, 2.0)],
        );
        let g = build_graph(&osm).expect("build_graph failed");
        assert_eq!(g.edge_count(), 1, "Expected 1 edge for a oneway way");
    }

    #[test]
    fn three_node_way_produces_correct_edge_count() {
        // A — B — C bidirectional: 4 directed edges (A↔B, B↔C)
        let osm = make_osm_data(
            vec![simple_way(1, vec![1, 2, 3], false)],
            vec![
                simple_node(1, 48.000, 2.000),
                simple_node(2, 48.001, 2.000),
                simple_node(3, 48.002, 2.000),
            ],
        );
        let g = build_graph(&osm).expect("build_graph failed");
        assert_eq!(g.node_count(), 3);
        assert_eq!(g.edge_count(), 4);
    }

    #[test]
    fn unused_nodes_are_excluded_from_graph() {
        // Node 99 exists in OSM data but is not referenced by any way
        let osm = make_osm_data(
            vec![simple_way(1, vec![10, 20], false)],
            vec![
                simple_node(10, 48.0, 2.0),
                simple_node(20, 48.001, 2.0),
                simple_node(99, 50.0, 3.0), // unreferenced
            ],
        );
        let g = build_graph(&osm).expect("build_graph failed");
        assert_eq!(
            g.node_count(),
            2,
            "Unreferenced node should not be in graph"
        );
    }

    #[test]
    fn travel_time_is_positive_and_finite() {
        let osm = make_osm_data(
            vec![simple_way(1, vec![10, 20], false)],
            vec![simple_node(10, 48.0, 2.0), simple_node(20, 48.001, 2.0)],
        );
        let g = build_graph(&osm).expect("build_graph failed");
        for edge in g.graph.edge_weights() {
            assert!(
                edge.travel_time_s.is_finite() && edge.travel_time_s > 0.0,
                "travel_time_s must be positive finite, got {}",
                edge.travel_time_s
            );
        }
    }

    // ------------------------------------------------------------------
    // Nearest node
    // ------------------------------------------------------------------

    #[test]
    fn nearest_node_returns_closest() {
        let osm = make_osm_data(
            vec![simple_way(1, vec![1, 2, 3], false)],
            vec![
                simple_node(1, 48.000, 2.000), // far
                simple_node(2, 48.500, 2.500), // medium
                simple_node(3, 49.000, 3.000), // target area
            ],
        );
        let g = build_graph(&osm).expect("build_graph failed");

        // Query close to node 3
        let nearest = g.nearest_node(49.001, 3.001).expect("should find a node");
        let nd = &g.graph[nearest];
        assert_eq!(nd.osm_id, 3, "Expected node 3 to be nearest");
    }

    #[test]
    fn nearest_node_on_empty_graph_returns_none() {
        let g = RoadGraph {
            graph: DiGraph::new(),
            node_index: HashMap::new(),
        };
        assert!(g.nearest_node(48.0, 2.0).is_none());
    }

    // ------------------------------------------------------------------
    // get_node
    // ------------------------------------------------------------------

    #[test]
    fn get_node_missing_returns_error() {
        let g = RoadGraph {
            graph: DiGraph::new(),
            node_index: HashMap::new(),
        };
        assert!(g.get_node(404).is_err());
    }
}

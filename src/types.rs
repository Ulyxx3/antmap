//! All shared data types for the antmap engine.
//! This module contains only plain data structures — no business logic.

// ---------------------------------------------------------------------------
// OSM raw types (output of osm_parser.rs)
// ---------------------------------------------------------------------------

/// A raw OSM node as returned by the Overpass API.
#[derive(Debug, Clone)]
pub struct OsmNode {
    /// OSM node identifier (positive integer).
    pub id: i64,
    /// Latitude in decimal degrees (WGS 84).
    pub lat: f64,
    /// Longitude in decimal degrees (WGS 84).
    pub lon: f64,
}

/// A raw OSM way (road segment) as returned by the Overpass API.
#[derive(Debug, Clone)]
pub struct OsmWay {
    /// OSM way identifier.
    pub id: i64,
    /// Ordered list of OSM node IDs forming this way.
    pub node_refs: Vec<i64>,
    /// Parsed highway classification (e.g. Motorway, Residential…).
    pub highway: HighwayType,
    /// Maximum allowed speed in km/h. Defaults to the highway class default
    /// when the `maxspeed` tag is absent.
    pub max_speed_kmh: f64,
    /// Whether this way is one-directional (OSM `oneway=yes`).
    pub is_oneway: bool,
}

/// OSM `highway` tag values relevant to routing, mapped to an enum for
/// exhaustive matching and penalty lookup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HighwayType {
    Motorway,
    MotorwayLink,
    Trunk,
    TrunkLink,
    Primary,
    PrimaryLink,
    Secondary,
    SecondaryLink,
    Tertiary,
    TertiaryLink,
    Residential,
    LivingStreet,
    Service,
    Track,
    Path,
    Cycleway,
    Footway,
    Unclassified,
    /// Any other value not explicitly handled.
    Other,
}

impl HighwayType {
    /// Default speed in km/h when `maxspeed` tag is absent, based on OSM
    /// community guidelines and European urban defaults.
    pub fn default_speed_kmh(self) -> f64 {
        match self {
            Self::Motorway | Self::MotorwayLink => 130.0,
            Self::Trunk | Self::TrunkLink => 110.0,
            Self::Primary | Self::PrimaryLink => 80.0,
            Self::Secondary | Self::SecondaryLink => 60.0,
            Self::Tertiary | Self::TertiaryLink => 50.0,
            Self::Residential => 30.0,
            Self::LivingStreet => 10.0,
            Self::Service => 20.0,
            Self::Track | Self::Path => 15.0,
            Self::Cycleway | Self::Footway => 5.0,
            Self::Unclassified | Self::Other => 40.0,
        }
    }

    /// Time penalty in **seconds** added on top of the pure distance/speed
    /// estimate. Higher penalties discourage ants from using slow or
    /// uncomfortable road types.
    pub fn time_penalty_seconds(self) -> f64 {
        match self {
            Self::Motorway | Self::Trunk => 0.0,
            Self::Primary | Self::Secondary => 5.0,
            Self::Tertiary => 10.0,
            Self::Residential => 20.0,
            Self::LivingStreet => 40.0,
            Self::Service => 30.0,
            Self::Track | Self::Path => 60.0,
            Self::Cycleway | Self::Footway => 120.0,
            _ => 15.0,
        }
    }
}

impl std::fmt::Display for HighwayType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

// ---------------------------------------------------------------------------
// Graph node / edge types (used by petgraph)
// ---------------------------------------------------------------------------

/// Data stored at each **node** of the routing graph.
#[derive(Debug, Clone)]
pub struct NodeData {
    /// Original OSM node id.
    pub osm_id: i64,
    /// Latitude in decimal degrees.
    pub lat: f64,
    /// Longitude in decimal degrees.
    pub lon: f64,
}

/// Data stored on each **directed edge** of the routing graph.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct EdgeData {
    /// OSM way id this edge was derived from.
    pub osm_way_id: i64,
    /// Euclidean / haversine distance in metres.
    pub distance_m: f64,
    /// Estimated travel time in **seconds** (distance / speed + penalty).
    /// This is the heuristic η used by the ants.
    pub travel_time_s: f64,
    /// Road classification kept for display / debugging.
    pub highway: HighwayType,
    /// Speed limit in km/h.
    pub max_speed_kmh: f64,
}

// ---------------------------------------------------------------------------
// ACO colony configuration
// ---------------------------------------------------------------------------

/// Hyper-parameters of the Ant Colony Optimization algorithm.
/// Each field has a documented default that works reasonably well out of the
/// box; users can override via CLI flags.
#[derive(Debug, Clone)]
pub struct ColonyConfig {
    /// Number of ants per iteration.
    /// Larger values improve solution quality at the cost of wall-clock time.
    /// **Default:** 50
    pub ant_count: usize,

    /// Total number of ACO iterations.
    /// **Default:** 100
    pub iterations: usize,

    /// α — pheromone importance exponent.
    /// Controls how strongly ants follow existing pheromone trails.
    /// **Default:** 1.0
    pub alpha: f64,

    /// β — heuristic importance exponent.
    /// Controls how strongly ants prefer edges with short travel times.
    /// **Default:** 3.0  (higher than α → ants prefer greedy choices early)
    pub beta: f64,

    /// ρ — pheromone evaporation rate ∈ (0, 1).
    /// After each iteration, pheromone on edge (i,j) becomes τ·(1−ρ).
    /// **Default:** 0.1
    pub rho: f64,

    /// Q — pheromone deposit constant.
    /// An ant that completes a path of total time T deposits Q/T on each edge.
    /// **Default:** 100.0
    pub q: f64,

    /// Initial pheromone concentration on all edges at iteration 0.
    /// **Default:** 1.0
    pub tau_initial: f64,

    /// Minimum pheromone level (anti-stagnation floor).
    /// **Default:** 0.01
    pub tau_min: f64,
}

impl Default for ColonyConfig {
    fn default() -> Self {
        Self {
            ant_count: 50,
            iterations: 100,
            alpha: 1.0,
            beta: 3.0,
            rho: 0.1,
            q: 100.0,
            tau_initial: 1.0,
            tau_min: 0.01,
        }
    }
}

// ---------------------------------------------------------------------------
// Result type from the solver
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Overpass query bounding box
// ---------------------------------------------------------------------------

/// Geographic bounding box used to query the Overpass API.
/// All values are in decimal degrees (WGS 84).
#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    pub south: f64,
    pub west: f64,
    pub north: f64,
    pub east: f64,
}

impl BoundingBox {
    /// Build a box centred on `(lat, lon)` with a given radius in kilometres.
    #[allow(dead_code)]
    pub fn from_center(lat: f64, lon: f64, radius_km: f64) -> Self {
        // 1° lat ≈ 111 km; 1° lon ≈ 111 km · cos(lat)
        let delta_lat = radius_km / 111.0;
        let delta_lon = radius_km / (111.0 * lat.to_radians().cos());
        Self {
            south: lat - delta_lat,
            west: lon - delta_lon,
            north: lat + delta_lat,
            east: lon + delta_lon,
        }
    }

    /// Returns `(south, west, north, east)` as a comma-separated string,
    /// ready for inclusion in an Overpass QL query.
    pub fn to_overpass_filter(self) -> String {
        format!(
            "{:.6},{:.6},{:.6},{:.6}",
            self.south, self.west, self.north, self.east
        )
    }
}

// ---------------------------------------------------------------------------
// Parsed OSM dataset (output of osm_parser, input of graph_builder)
// ---------------------------------------------------------------------------

/// The combined output of the OSM parser: a set of nodes and ways.
#[derive(Debug, Clone, Default)]
pub struct OsmData {
    pub nodes: Vec<OsmNode>,
    pub ways: Vec<OsmWay>,
}

// ---------------------------------------------------------------------------

/// The best route found by the ACO solver.
#[derive(Debug, Clone)]
pub struct Route {
    /// Sequence of OSM node IDs from source to destination.
    pub node_ids: Vec<i64>,
    /// Total estimated travel time in seconds.
    pub total_time_s: f64,
    /// Total distance in metres.
    pub total_distance_m: f64,
}

impl Route {
    /// Returns the travel time formatted as `Xh Ym Zs`.
    pub fn formatted_time(&self) -> String {
        let total = self.total_time_s as u64;
        let h = total / 3600;
        let m = (total % 3600) / 60;
        let s = total % 60;
        if h > 0 {
            format!("{}h {}m {}s", h, m, s)
        } else {
            format!("{}m {}s", m, s)
        }
    }
}

use std::path::PathBuf;

use crate::aco_solver;
use crate::graph_builder;
use crate::osm_parser;
use crate::types::{BoundingBox, ColonyConfig, Route};

pub struct EngineConfig {
    pub from: (f64, f64),
    pub to: (f64, f64),
    pub input: Option<PathBuf>,
    pub ants: usize,
    pub iterations: usize,
    pub alpha: f64,
    pub beta: f64,
    pub rho: f64,
}

pub struct EngineResult {
    pub route: Route,
    pub geojson: String,
}

pub fn run_routing(config: EngineConfig) -> anyhow::Result<EngineResult> {
    let colony_config = ColonyConfig {
        ant_count: config.ants,
        iterations: config.iterations,
        alpha: config.alpha,
        beta: config.beta,
        rho: config.rho,
        ..Default::default()
    };

    let osm_data = match &config.input {
        Some(path) => osm_parser::load_from_file(path)?,
        None => {
            // Compute a bounding box wrapping both coords + 0.008° margin (~800m)
            // The public Overpass API restricts large queries during peak hours (504 Timeout)
            let margin = 0.008;
            let bbox = BoundingBox {
                south: config.from.0.min(config.to.0) - margin,
                north: config.from.0.max(config.to.0) + margin,
                west: config.from.1.min(config.to.1) - margin,
                east: config.from.1.max(config.to.1) + margin,
            };
            osm_parser::fetch_from_overpass(bbox, osm_parser::DEFAULT_OVERPASS_URL)?
        }
    };

    let road_graph = graph_builder::build_graph(&osm_data)?;

    let start_node = road_graph
        .nearest_node(config.from.0, config.from.1)
        .ok_or_else(|| anyhow::anyhow!("No road nodes found near start coordinate"))?;

    let target_node = road_graph
        .nearest_node(config.to.0, config.to.1)
        .ok_or_else(|| anyhow::anyhow!("No road nodes found near target coordinate"))?;

    let start_osm_id = road_graph.graph[start_node].osm_id;
    let target_osm_id = road_graph.graph[target_node].osm_id;

    log::info!(
        "Snapped start to OSM node {} (offset: {:.1}m)",
        start_osm_id,
        graph_builder::haversine_m(
            config.from.0,
            config.from.1,
            road_graph.graph[start_node].lat,
            road_graph.graph[start_node].lon
        )
    );
    log::info!(
        "Snapped target to OSM node {} (offset: {:.1}m)",
        target_osm_id,
        graph_builder::haversine_m(
            config.to.0,
            config.to.1,
            road_graph.graph[target_node].lat,
            road_graph.graph[target_node].lon
        )
    );

    let route = aco_solver::solve(&road_graph, start_osm_id, target_osm_id, &colony_config)?;

    // 8. Export GeoJSON for visual rendering
    let mut coord_strings = Vec::with_capacity(route.node_ids.len());
    for &osm_id in &route.node_ids {
        if let Ok(idx) = road_graph.get_node(osm_id) {
            let nd = &road_graph.graph[idx];
            // GeoJSON expects [longitude, latitude]
            coord_strings.push(format!("[{:.6}, {:.6}]", nd.lon, nd.lat));
        }
    }

    let geojson = if !coord_strings.is_empty() {
        format!(
            r##"{{
  "type": "FeatureCollection",
  "features": [
    {{
      "type": "Feature",
      "geometry": {{
        "type": "LineString",
        "coordinates": [
          {}
        ]
      }},
      "properties": {{
        "name": "Ant Route",
        "time": "{}",
        "distance_km": {:.2},
        "stroke": "#ff2600",
        "stroke-width": 4
      }}
    }}
  ]
}}"##,
            coord_strings.join(",\n          "),
            route.formatted_time(),
            route.total_distance_m / 1000.0
        )
    } else {
        String::new()
    };

    Ok(EngineResult { route, geojson })
}

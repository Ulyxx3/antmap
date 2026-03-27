use std::path::PathBuf;

use clap::Parser;
use log::info;

mod aco_solver;
mod error;
mod graph_builder;
mod osm_parser;
mod types;

use crate::types::{BoundingBox, ColonyConfig};

/// antmap — ACO routing engine
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Start coordinates: "lat,lon"
    #[arg(long, value_parser = parse_coords)]
    from: (f64, f64),

    /// Target coordinates: "lat,lon"
    #[arg(long, value_parser = parse_coords)]
    to: (f64, f64),

    /// Read local OpenStreetMap JSON file instead of querying Overpass API
    #[arg(long, short)]
    input: Option<PathBuf>,

    /// Number of ants per iteration
    #[arg(long, default_value_t = 50)]
    ants: usize,

    /// Number of ACO iterations
    #[arg(long, default_value_t = 100)]
    iterations: usize,

    /// Pheromone importance (alpha)
    #[arg(long, default_value_t = 1.0)]
    alpha: f64,

    /// Heuristic importance (beta)
    #[arg(long, default_value_t = 3.0)]
    beta: f64,

    /// Pheromone evaporation rate (rho)
    #[arg(long, default_value_t = 0.1)]
    rho: f64,
}

/// Parse "lat,lon" string from CLI into a tuple of (f64, f64)
fn parse_coords(s: &str) -> std::result::Result<(f64, f64), String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err("Coordinates must be exactly formatted as 'lat,lon'".to_string());
    }
    let lat = parts[0]
        .parse::<f64>()
        .map_err(|e| format!("Invalid latitude: {}", e))?;
    let lon = parts[1]
        .parse::<f64>()
        .map_err(|e| format!("Invalid longitude: {}", e))?;
    Ok((lat, lon))
}

fn main() -> anyhow::Result<()> {
    // 1. Initialise logging
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // 2. Parse CLI arguments
    let args = Args::parse();

    let config = ColonyConfig {
        ant_count: args.ants,
        iterations: args.iterations,
        alpha: args.alpha,
        beta: args.beta,
        rho: args.rho,
        ..Default::default()
    };

    // 3. Load or fetch OSM Data
    let osm_data = match args.input {
        Some(path) => osm_parser::load_from_file(&path)?,
        None => {
            // Compute a bounding box wrapping both coords + 0.008° margin (~800m)
            // The public Overpass API restricts large queries during peak hours (504 Timeout)
            let margin = 0.008;
            let bbox = BoundingBox {
                south: args.from.0.min(args.to.0) - margin,
                north: args.from.0.max(args.to.0) + margin,
                west: args.from.1.min(args.to.1) - margin,
                east: args.from.1.max(args.to.1) + margin,
            };
            osm_parser::fetch_from_overpass(bbox, osm_parser::DEFAULT_OVERPASS_URL)?
        }
    };

    // 4. Build routing graph
    let road_graph = graph_builder::build_graph(&osm_data)?;

    // 5. Snap coordinates to nearest road nodes
    let start_node = road_graph
        .nearest_node(args.from.0, args.from.1)
        .ok_or_else(|| anyhow::anyhow!("No road nodes found near start coordinate"))?;

    let target_node = road_graph
        .nearest_node(args.to.0, args.to.1)
        .ok_or_else(|| anyhow::anyhow!("No road nodes found near target coordinate"))?;

    let start_osm_id = road_graph.graph[start_node].osm_id;
    let target_osm_id = road_graph.graph[target_node].osm_id;

    let start_snap_dist = graph_builder::haversine_m(
        args.from.0,
        args.from.1,
        road_graph.graph[start_node].lat,
        road_graph.graph[start_node].lon,
    );
    let target_snap_dist = graph_builder::haversine_m(
        args.to.0,
        args.to.1,
        road_graph.graph[target_node].lat,
        road_graph.graph[target_node].lon,
    );

    info!(
        "Snapped start to OSM node {} (offset: {:.1}m)",
        start_osm_id, start_snap_dist
    );
    info!(
        "Snapped target to OSM node {} (offset: {:.1}m)",
        target_osm_id, target_snap_dist
    );

    // 6. Run ACO solver
    let route = aco_solver::solve(&road_graph, start_osm_id, target_osm_id, &config)?;

    // 7. Output result
    println!("\n===========================================");
    println!("🐜 Route Found!");
    println!("Time: {}", route.formatted_time());
    println!("Distance: {:.1} km", route.total_distance_m / 1000.0);
    println!("Waypoints: {} nodes traversed", route.node_ids.len());
    println!("===========================================\n");

    // 8. Export GeoJSON for visual rendering
    let mut coord_strings = Vec::with_capacity(route.node_ids.len());
    for &osm_id in &route.node_ids {
        if let Ok(idx) = road_graph.get_node(osm_id) {
            let nd = &road_graph.graph[idx];
            // GeoJSON expects [longitude, latitude]
            coord_strings.push(format!("[{:.6}, {:.6}]", nd.lon, nd.lat));
        }
    }

    if !coord_strings.is_empty() {
        let geojson = format!(
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
        );

        std::fs::write("route.geojson", geojson)?;
        println!("🗺️  Visual map saved to 'route.geojson'.");
        println!("👉 Drag & drop this file onto https://geojson.io to view your route!");
        println!();
    }

    Ok(())
}

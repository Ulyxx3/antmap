use std::path::PathBuf;

use clap::Parser;

mod aco_solver;
mod engine;
mod error;
mod graph_builder;
mod gui;
mod osm_parser;
mod types;

/// antmap — ACO routing engine
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Start coordinates: "lat,lon"
    #[arg(long, value_parser = parse_coords)]
    from: Option<(f64, f64)>,

    /// Target coordinates: "lat,lon"
    #[arg(long, value_parser = parse_coords)]
    to: Option<(f64, f64)>,

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

    // 2. Hybrid Launch: if no arguments are provided, launch GUI!
    let cli_args: Vec<String> = std::env::args().collect();
    let is_gui = cli_args.len() <= 1;

    if is_gui {
        let native_options = eframe::NativeOptions {
            viewport: eframe::egui::ViewportBuilder::default()
                .with_inner_size([400.0, 500.0])
                .with_min_inner_size([300.0, 400.0]),
            ..Default::default()
        };
        return match eframe::run_native(
            "🐜 Antmap - ACO Routing",
            native_options,
            Box::new(|cc| Box::new(gui::AntmapApp::new(cc))),
        ) {
            Ok(_) => Ok(()),
            Err(e) => Err(anyhow::anyhow!("Eframe error: {}", e)),
        };
    }

    // 3. Otherwise, parse CLI arguments normally
    let args = Args::parse();
    
    // Validate required arguments for CLI
    let from = args.from.ok_or_else(|| anyhow::anyhow!("--from is required in CLI mode"))?;
    let to = args.to.ok_or_else(|| anyhow::anyhow!("--to is required in CLI mode"))?;

    let config = engine::EngineConfig {
        from,
        to,
        input: args.input,
        ants: args.ants,
        iterations: args.iterations,
        alpha: args.alpha,
        beta: args.beta,
        rho: args.rho,
    };

    // 4. Run the decoupled engine
    let result = engine::run_routing(config)?;

    // 5. Output result
    println!("\n===========================================");
    println!("🐜 Route Found!");
    println!("Time: {}", result.route.formatted_time());
    println!("Distance: {:.1} km", result.route.total_distance_m / 1000.0);
    println!("Waypoints: {} nodes traversed", result.route.node_ids.len());
    println!("===========================================\n");

    if !result.geojson.is_empty() {
        std::fs::write("route.geojson", &result.geojson)?;
        println!("🗺️  Visual map saved to 'route.geojson'.");
        println!("👉 Drag & drop this file onto https://geojson.io to view your route!");
        println!();
    }

    Ok(())
}

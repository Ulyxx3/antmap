//! # ACO Solver
//!
//! Applies the Ant Colony Optimization algorithm to find the lowest-time
//! path on the road graph.

use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rand::distributions::WeightedIndex;
use rand::prelude::*;
use rayon::prelude::*;

use crate::error::{AntmapError, Result};
use crate::graph_builder::RoadGraph;
use crate::types::{ColonyConfig, Route};

/// Solves the routing problem from `from_osm` to `to_osm` using ACO.
pub fn solve(
    road_graph: &RoadGraph,
    from_osm: i64,
    to_osm: i64,
    config: &ColonyConfig,
) -> Result<Route> {
    let start_node = road_graph.get_node(from_osm)?;
    let target_node = road_graph.get_node(to_osm)?;

    if start_node == target_node {
        return Ok(Route {
            node_ids: vec![from_osm],
            total_time_s: 0.0,
            total_distance_m: 0.0,
        });
    }

    let edge_count = road_graph.edge_count();
    if edge_count == 0 {
        return Err(AntmapError::NoPathFound {
            from: from_osm,
            to: to_osm,
        });
    }

    // Pheromone matrix: indexed by EdgeIndex.index()
    let mut pheromones = vec![config.tau_initial; edge_count];

    // --- Graph Connectivity Check ---
    let astar_path = petgraph::algo::astar(
        &road_graph.graph,
        start_node,
        |finish| finish == target_node,
        |e| e.weight().travel_time_s,
        |_| 0.0, // We could use a heuristic here, but Dijkstra is fine for a quick check.
    );
    if let Some((cost, _)) = astar_path {
        log::info!("A* verification: A valid path DOES exist (optimal time: {:.1}s). ACO will now search for it.", cost);
    } else {
        log::error!("A* verification: NO PATH EXISTS between the selected nodes. The graph is disconnected here (e.g. one-way traps, pedestrian zones).");
        return Err(AntmapError::NoPathFound {
            from: from_osm,
            to: to_osm,
        });
    }
    // --------------------------------

    let mut global_best: Option<Route> = None;
    let mut global_best_time = f64::MAX;

    log::info!(
        "Starting ACO: {} ants, {} iterations, start={}, target={}",
        config.ant_count,
        config.iterations,
        from_osm,
        to_osm
    );

    for iteration in 0..config.iterations {
        // 1. Run ants in parallel
        let iter_results: Vec<Option<AntResult>> = (0..config.ant_count)
            .into_par_iter()
            .map(|_| run_ant(road_graph, start_node, target_node, config, &pheromones))
            .collect();

        // Filter out ants that got stuck in dead-ends
        let valid_paths: Vec<AntResult> = iter_results.into_iter().flatten().collect();
        let success_count = valid_paths.len();

        let mut iter_best: Option<AntResult> = None;
        let mut iter_best_time = f64::MAX;

        for result in &valid_paths {
            if result.total_time_s < iter_best_time {
                iter_best_time = result.total_time_s;
                iter_best = Some(result.clone());
            }
        }

        // 2. Pheromone Evaporation: τ = max(τ * (1 - ρ), τ_min)
        for tau in pheromones.iter_mut() {
            *tau = (*tau * (1.0 - config.rho)).max(config.tau_min);
        }

        // 3. Pheromone Deposit
        // Ants that found a path leave pheromones.
        // The shorter the time, the more pheromones they deposit.
        for result in &valid_paths {
            let deposit = config.q / result.total_time_s;
            for &edge_idx in &result.edges {
                pheromones[edge_idx.index()] += deposit;
            }
        }

        // Update global best if this iteration found a new record
        if let Some(best) = iter_best {
            if best.total_time_s < global_best_time {
                global_best_time = best.total_time_s;

                // Convert petgraph NodeIndex sequence to OSM node IDs
                let node_ids = best
                    .nodes
                    .iter()
                    .map(|&n| road_graph.graph[n].osm_id)
                    .collect();

                global_best = Some(Route {
                    node_ids,
                    total_time_s: best.total_time_s,
                    total_distance_m: best.total_distance_m,
                });

                log::info!(
                    "Iteration {}: New best route found! Time: {}, distance: {:.0}m",
                    iteration,
                    global_best.as_ref().unwrap().formatted_time(),
                    best.total_distance_m
                );
            }
        }

        log::debug!(
            "Iteration {} finished. {}/{} ants reached target.",
            iteration,
            success_count,
            config.ant_count
        );

        // Anti-stagnation safeguard (optional, but good practice):
        // If 0 ants found the path after the first iteration, it might be impossible.
        if success_count == 0 && global_best.is_none() {
            log::warn!(
                "Early abort: No ants could reach the target. Check if the graph is connected."
            );
            break;
        }
    }

    global_best.ok_or(AntmapError::NoPathFound {
        from: from_osm,
        to: to_osm,
    })
}

#[derive(Clone)]
struct AntResult {
    nodes: Vec<NodeIndex>,
    edges: Vec<EdgeIndex>,
    total_time_s: f64,
    total_distance_m: f64,
}

/// Simulates a single ant walking the graph probabilistically using a Loop-Erased Random Walk.
fn run_ant(
    road_graph: &RoadGraph,
    start: NodeIndex,
    target: NodeIndex,
    config: &ColonyConfig,
    pheromones: &[f64],
) -> Option<AntResult> {
    let mut rng = rand::thread_rng();

    let mut current_node = start;
    let mut path_nodes = vec![start];
    let mut path_edges = Vec::new();

    // Nodes that are strictly dead-ends (no way forward) are added here when backtracking.
    let mut dead_ends = std::collections::HashSet::new();

    // Safety break to prevent very long walks in massive graphs.
    // LERW requires more steps since ants can explore culs-de-sac and backtrack.
    let max_steps = road_graph.node_count() * 2;
    let mut steps = 0;

    let target_data = &road_graph.graph[target];

    while current_node != target && steps < max_steps {
        steps += 1;

        // The node we just came from, to prevent trivial "A -> B -> A" ping-ponging on straight roads,
        // EXCEPT when we are in a dead end and HAVE to go back.
        let predecessor = if path_nodes.len() >= 2 {
            Some(path_nodes[path_nodes.len() - 2])
        } else {
            None
        };

        let mut available_edges = Vec::new();
        let mut fallback_edges = Vec::new();

        for edge_ref in road_graph.graph.edges(current_node) {
            let next_node = edge_ref.target();

            // Never enter a known dead-end branch
            if dead_ends.contains(&next_node) {
                continue;
            }

            if Some(next_node) != predecessor {
                available_edges.push(edge_ref);
            } else {
                fallback_edges.push(edge_ref);
            }
        }

        // If we are in a cul-de-sac, our only way out is the fallback edge.
        let candidates = if !available_edges.is_empty() {
            available_edges
        } else if !fallback_edges.is_empty() {
            // We are forced to backtrack. This means current_node is a dead end!
            dead_ends.insert(current_node);
            fallback_edges
        } else {
            // Completely trapped (e.g., node with 0 outgoing edges)
            return None;
        };

        let mut weights = Vec::with_capacity(candidates.len());
        for edge_ref in &candidates {
            let next_node = edge_ref.target();
            let tau = pheromones[edge_ref.id().index()];
            let next_data = &road_graph.graph[next_node];

            let dist_to_target = crate::graph_builder::haversine_m(
                next_data.lat,
                next_data.lon,
                target_data.lat,
                target_data.lon,
            );

            // Basic eta = 1 / travel_time
            let mut eta = 1.0 / edge_ref.weight().travel_time_s;

            // Geographical guidance: boost paths moving closer to the target
            let current_data = &road_graph.graph[current_node];
            let current_dist_to_target = crate::graph_builder::haversine_m(
                current_data.lat,
                current_data.lon,
                target_data.lat,
                target_data.lon,
            );

            if dist_to_target < current_dist_to_target {
                eta *= 2.0; // Strong pull towards target
            } else {
                eta *= 0.5; // Penalty for moving away
            }

            let prob_weight = tau.powf(config.alpha) * eta.powf(config.beta);
            weights.push(prob_weight);
        }

        let dist = match WeightedIndex::new(&weights) {
            Ok(d) => d,
            Err(_) => return None,
        };

        let chosen_idx = dist.sample(&mut rng);
        let chosen_edge = candidates[chosen_idx];
        let next_node = chosen_edge.target();

        // --- Loop Erasure ---
        // If we revisit a node already in our path, we immediately slice off the entire loop.
        if let Some(loop_start_idx) = path_nodes.iter().position(|&n| n == next_node) {
            // Cut the path back to the state it was right after we visited `next_node` for the first time
            path_nodes.truncate(loop_start_idx + 1);
            path_edges.truncate(loop_start_idx);
        } else {
            path_nodes.push(next_node);
            path_edges.push(chosen_edge.id());
        }

        current_node = next_node;
    }

    if current_node == target {
        // Recompute the sum of the final, loop-free path
        let mut total_time_s = 0.0;
        let mut total_distance_m = 0.0;

        for &e_idx in &path_edges {
            let weight = road_graph.graph.edge_weight(e_idx).unwrap();
            total_time_s += weight.travel_time_s;
            total_distance_m += weight.distance_m;
        }

        Some(AntResult {
            nodes: path_nodes,
            edges: path_edges,
            total_time_s,
            total_distance_m,
        })
    } else {
        None // Took too many steps without reaching the target
    }
}

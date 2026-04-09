<div align="center">

# 🐜 antmap

**An Ant Colony Optimization routing engine over real OpenStreetMap data — written in Rust.**

[![CI](https://github.com/Ulyxx3/antmap/actions/workflows/ci.yml/badge.svg)](https://github.com/Ulyxx3/antmap/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Rust](https://img.shields.io/badge/rust-stable-orange.svg)

</div>

---

## What is antmap?

`antmap` finds the **fastest driving route** between two GPS coordinates using the **Ant Colony Optimization (ACO)** metaheuristic on a real road graph built from [OpenStreetMap](https://www.openstreetmap.org/) data.

Unlike Dijkstra or A\*, ACO is a probabilistic, bio-inspired approach that:
- explores many candidate paths **in parallel** (parallelised with `rayon`)
- reinforces fast paths via **pheromone deposits**
- converges toward an optimum through **evaporation + positive feedback**

---

## Features

- 🗺️ **Real map data** — queries the [Overpass API](https://overpass-api.de/) or reads a local JSON dump
- ⏱️ **Time-based heuristic** — optimises travel time (distance ÷ speed) + road-type penalties, not raw distance
- 🐜 **Parallel ants** — every ant in an iteration runs in its own thread via `rayon`
- 🛣️ **Full OSM tag support** — `highway`, `maxspeed` (including mph & country codes), `oneway`
- 🦀 **Idiomatic Rust** — strict `Result` error handling, zero `unwrap()` in library code

---

## Architecture

```
src/
├── main.rs          # CLI entry point (clap)
├── error.rs         # AntmapError + Result<T> alias
├── types.rs         # Shared structs: NodeData, EdgeData, ColonyConfig, Route …
├── osm_parser.rs    # Overpass JSON → OsmNode / OsmWay
├── graph_builder.rs # OsmData → petgraph DiGraph weighted by travel time
└── aco_solver.rs    # ACO algorithm: pheromones, evaporation, parallel ants
```

---

## Installation

```bash
git clone https://github.com/Ulyxx3/antmap.git
cd antmap
cargo build --release
```

> **Requirements:** Rust stable ≥ 1.75, internet access for Overpass queries.

---

## Usage

```bash
# Route between two GPS coordinates (fetches data from Overpass)
./target/release/antmap --from 48.8566,2.3522 --to 48.8742,2.3470

# Use a local OSM JSON file instead
./target/release/antmap --from 48.8566,2.3522 --to 48.8742,2.3470 --input map.json

# Tune the ACO parameters
./target/release/antmap \
  --from 48.8566,2.3522 \
  --to 48.8742,2.3470 \
  --ants 100 \
  --iterations 200 \
  --alpha 1.0 \
  --beta 4.0 \
  --rho 0.05
```

---

## How ACO works here

| Step | Description |
|---|---|
| **Graph construction** | OSM ways are split into directed edges; each edge weight = `distance / maxspeed + road_penalty` |
| **Initialisation** | All edges start with uniform pheromone τ₀ |
| **Iteration** | Each ant builds a path with a probabilistic rule: `P(i→j) ∝ τ(i,j)^α · η(i,j)^β` where η = 1/travel_time |
| **Pheromone update** | Each ant deposits `Q / total_time` on its edges; all edges evaporate by `(1−ρ)` |
| **Result** | The shortest-time path found across all iterations is returned |

---

## License

MIT © Ulyxx3

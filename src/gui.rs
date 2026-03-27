use eframe::egui;
use std::sync::mpsc::Receiver;

use crate::engine::{run_routing, EngineConfig, EngineResult};

pub struct AntmapApp {
    from_lat: String,
    from_lon: String,
    to_lat: String,
    to_lon: String,

    ants: usize,
    iterations: usize,
    alpha: f64,
    beta: f64,
    rho: f64,

    computation_receiver: Option<Receiver<anyhow::Result<EngineResult>>>,
    last_result: Option<anyhow::Result<EngineResult>>,
}

impl Default for AntmapApp {
    fn default() -> Self {
        Self {
            from_lat: "48.8715".to_string(),
            from_lon: "2.2936".to_string(),
            to_lat: "48.8643".to_string(),
            to_lon: "2.2965".to_string(),

            ants: 50,
            iterations: 100,
            alpha: 1.0,
            beta: 3.0,
            rho: 0.1,

            computation_receiver: None,
            last_result: None,
        }
    }
}

impl AntmapApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Default::default()
    }
}

impl eframe::App for AntmapApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // If a computation is actively running, check if it's done
        if let Some(rx) = &self.computation_receiver {
            if let Ok(res) = rx.try_recv() {
                self.last_result = Some(res);
                self.computation_receiver = None;
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("🐜 Antmap - Ant Colony Routing Engine");
            ui.add_space(10.0);

            let is_running = self.computation_receiver.is_some();

            ui.add_enabled_ui(!is_running, |ui| {
                ui.horizontal(|ui| {
                    ui.group(|ui| {
                        ui.label("Start Coordinates (Lat/Lon)");
                        ui.horizontal(|ui| {
                            ui.add(egui::TextEdit::singleline(&mut self.from_lat).desired_width(80.0));
                            ui.add(egui::TextEdit::singleline(&mut self.from_lon).desired_width(80.0));
                        });
                    });

                    ui.group(|ui| {
                        ui.label("Target Coordinates (Lat/Lon)");
                        ui.horizontal(|ui| {
                            ui.add(egui::TextEdit::singleline(&mut self.to_lat).desired_width(80.0));
                            ui.add(egui::TextEdit::singleline(&mut self.to_lon).desired_width(80.0));
                        });
                    });
                });

                ui.add_space(10.0);

                ui.collapsing("Advanced Parameters ⚙️", |ui| {
                    ui.add(egui::Slider::new(&mut self.ants, 10..=500).text("Ants per iteration"));
                    ui.add(egui::Slider::new(&mut self.iterations, 10..=500).text("Iterations"));
                    ui.add(egui::Slider::new(&mut self.alpha, 0.0..=5.0).text("Alpha (Pheromones)"));
                    ui.add(egui::Slider::new(&mut self.beta, 0.0..=10.0).text("Beta (Heuristics)"));
                    ui.add(egui::Slider::new(&mut self.rho, 0.0..=0.99).text("Rho (Evaporation rate)"));
                });

                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    let run_btn = ui.add_sized([150.0, 30.0], egui::Button::new("Run Routing!"));
                    if run_btn.clicked() {
                        let from_lat = self.from_lat.parse::<f64>().unwrap_or_default();
                        let from_lon = self.from_lon.parse::<f64>().unwrap_or_default();
                        let to_lat = self.to_lat.parse::<f64>().unwrap_or_default();
                        let to_lon = self.to_lon.parse::<f64>().unwrap_or_default();

                        let config = EngineConfig {
                            from: (from_lat, from_lon),
                            to: (to_lat, to_lon),
                            input: None,
                            ants: self.ants,
                            iterations: self.iterations,
                            alpha: self.alpha,
                            beta: self.beta,
                            rho: self.rho,
                        };

                        let (tx, rx) = std::sync::mpsc::channel();
                        self.computation_receiver = Some(rx);
                        self.last_result = None;

                        let ctx_clone = ctx.clone();
                        std::thread::spawn(move || {
                            let res = run_routing(config);
                            let _ = tx.send(res);
                            ctx_clone.request_repaint(); // Wake up UI
                        });
                    }
                });
            });

            ui.separator();

            if is_running {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(egui::RichText::new("Running the ACO simulation... This may take a minute based on Overpass API.").italics());
                });
            } else if let Some(res) = &self.last_result {
                match res {
                    Ok(engine_result) => {
                        ui.label(egui::RichText::new("🐜 Route Found!").color(egui::Color32::GREEN).size(18.0));
                        ui.label(format!("Time: {}", engine_result.route.formatted_time()));
                        ui.label(format!("Distance: {:.1} km", engine_result.route.total_distance_m / 1000.0));

                        ui.add_space(10.0);
                        ui.horizontal(|ui| {
                            if ui.button("📋 Copy GeoJSON to Clipboard").clicked() {
                                ui.output_mut(|o| o.copied_text = engine_result.geojson.clone());
                            }
                            if ui.button("🗺 Open geojson.io").clicked() {
                                let _ = webbrowser::open("https://geojson.io/");
                            }
                        });
                    }
                    Err(e) => {
                        ui.label(egui::RichText::new("Failed to compute route:").color(egui::Color32::RED).size(14.0));
                        ui.label(egui::RichText::new(format!("{}", e)).color(egui::Color32::RED));
                    }
                }
            }
        });
    }
}

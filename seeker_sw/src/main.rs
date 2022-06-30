use clap::{Arg, Parser};
use eframe::egui;
use egui::{RichText, Window};
use peripherals::{Compass, GUIItem, Radio, GPS};

mod configuration;
mod peripherals;
mod widgets;

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[clap(short, long, value_parser)]
    gps_port: String,

    #[clap(short, long, value_parser)]
    calibrate_magnetometer: bool,
}

pub mod items {
    include!(concat!(env!("OUT_DIR"), "/eguiprototype.items.rs"));
}

fn main() {
    let native_options = eframe::NativeOptions::default();
    let args = Args::parse();

    eframe::run_native(
        "Rocket Seeker V0.1 indev",
        native_options,
        Box::new(|cc| Box::new(Seeker::new(cc, args))),
    );
}

struct Seeker {
    radio: Radio,
    gps: GPS,
    compass: Compass,
}

impl Seeker {
    fn new(cc: &eframe::CreationContext<'_>, args: Args) -> Self {
        // Customize egui here with cc.egui_ctx.set_fonts and cc.egui_ctx.set_visuals.
        // Restore app state using cc.storage (requires the "persistence" feature).
        // Use the cc.gl (a glow::Context) to create graphics shaders and buffers that you can use
        // for e.g. egui::PaintCallback.

        let mut s: Seeker = Seeker {
            radio: Radio::default(),
            gps: GPS::new(args.gps_port),
            compass: Compass::new(cc),
        };

        #[cfg(target_arch = "arm")]
        if (args.calibrate_magnetometer) {
            s.compass
                .run_calibration()
                .expect("Error during magnetometer configuration");
        }

        return s;
    }
}

impl eframe::App for Seeker {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.heading("Seeker");
            });
        });

        self.radio.render_window(ctx, frame);
        self.gps.render_window(ctx, frame);
        self.compass.render_window(ctx, frame);
        ctx.request_repaint();
    }
}

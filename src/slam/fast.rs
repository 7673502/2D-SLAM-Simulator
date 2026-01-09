use crate::slam::Slam;
use crate::simulation::Observation;
use crate::config::Config;

pub struct FastSlam {
    
}

impl Slam for FastSlam {
    fn predict(&mut self, linear_vel: f32, angular_vel: f32, dt: f32, cfg: &Config) {
        todo!()
    }

    fn update(&mut self, observations: &[Observation], cfg: &Config) {
        todo!()
    }

    fn get_state(&self) -> (f32, f32, f32) {
        todo!()
    }

    fn get_landmarks(&self) -> Vec<(usize, f32, f32)> {
        todo!()
    }

    fn color(&self) -> macroquad::prelude::Color {
        todo!()
    }
}

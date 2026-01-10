use crate::simulation::Observation;
use crate::config::Config;
use macroquad::color::Color;

pub trait Slam {
    fn predict(&mut self, linear_velocity: f32, angular_velocity: f32, delta_time: f32, cfg: &Config);
    fn update(&mut self, observations: &[Observation], cfg: &Config);
    fn get_state(&self) -> (f32, f32, f32);
    fn get_landmarks(&self) -> Vec<(usize, f32, f32)>;
    fn color(&self) -> Color;
}


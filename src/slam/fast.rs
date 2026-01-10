use std::collections::HashMap;
use nalgebra::{Matrix2, Vector2};

use crate::slam::Slam;
use crate::simulation::Observation;
use crate::config::Config;
use crate::utils::sample_normal;

#[derive(Clone)]
pub struct LandmarkEstimate {
    pub mu: Vector2<f32>,
    pub sigma: Matrix2<f32>,
}

#[derive(Clone)]
pub struct Particle {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub weight: f32,
    pub landmarks: HashMap<usize, LandmarkEstimate>,
}

pub struct FastSlam {
    pub particles: Vec<Particle>,
    pub num_particles: usize,
}

impl FastSlam {
    pub fn new(num_particles: usize) -> Self {
        let particles = vec![
            Particle {
                x: 0.0,
                y: 0.0,
                theta: 0.0,
                weight: 1.0,
                landmarks: HashMap::new(),
            };
            num_particles
        ];

        Self {
            particles,
            num_particles,
        }
    }
}

impl Slam for FastSlam {
    fn predict(&mut self, linear_velocity: f32, angular_velocity: f32, delta_time: f32, cfg: &Config) {
        for particle in &mut self.particles {
            let noisy_linear_velocity = linear_velocity + sample_normal(0.0, (cfg.est_stdev_linear * linear_velocity.abs()).max(0.01));
            let noisy_angular_velocity  = angular_velocity + sample_normal(0.0, (cfg.est_stdev_linear * linear_velocity.abs()).max(0.01));

            let theta_half = particle.theta + 0.5 * noisy_angular_velocity * delta_time;
        
            // update position estimate
            particle.x += noisy_linear_velocity * delta_time * theta_half.cos();
            particle.y += noisy_linear_velocity * delta_time * theta_half.sin();
            particle.theta += noisy_angular_velocity * delta_time;

            // normalize angle to (-PI, PI]
            particle.theta = f32::atan2(particle.theta.sin(), particle.theta.cos());
        }
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

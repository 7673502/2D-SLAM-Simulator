use macroquad::prelude::*;
use ::rand::rng;
use rand_distr::{Normal, Distribution};

/*
 * Constants
 */
const LINEAR_ACC: f32 = 250.0;
const ANGULAR_ACC: f32 = 25.0;
const DECAY_FACTOR: f32 = 0.02; // velocity falloff
const ROBOT_RADIUS: f32 = 24.0;

// speed caps
const MAX_LINEAR_SPEED: f32 = 100.0;
const MAX_ANGULAR_SPEED: f32 = 25.0;

// process noise coefficients
const ALPHA_LINEAR: f32 = 0.05;
const ALPHA_ANGULAR: f32 = 0.01;

// landmark size
const LANDMARK_RADIUS: f32 = 4.0;

// obstruction size
const OBSTRUCTION_WIDTH: f32 = 50.0;
const OBSTRUCTION_HEIGHT: f32 = 50.0;

fn window_conf() -> Conf {
    Conf {
        window_title: "2D EKF SLAM Simulator".to_owned(),
        window_width: 800,
        window_height: 600,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // position states
    let mut x: f32 = screen_width() / 4.0;
    let mut y: f32 = screen_height() / 2.0; 
    let mut linear_velocity: f32 = 0.0;

    // direction states
    let mut dir: f32 = 0.0;
    let mut angular_velocity: f32 = 0.0;
    
    // noise generation
    let mut rng = rng();
    let normal = Normal::new(0.0, 1.0).unwrap();
    let mut epsilon = |alpha: f32| alpha * normal.sample(&mut rng);
    
    let mut prev_gt_linear_velocity: f32 = 0.0;
    let mut prev_gt_angular_velocity: f32 = 0.0;
    
    // rectangles and landmarks
    let mut obstructions: Vec<Rect> = Vec::new();
    let mut landmarks: Vec<(f32, f32)> = Vec::new();


    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();

        // movement
        if is_key_down(KeyCode::Up) {
            linear_velocity += LINEAR_ACC * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            linear_velocity -= LINEAR_ACC * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            angular_velocity += ANGULAR_ACC * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            angular_velocity -= ANGULAR_ACC * delta_time;
        }
        
        // adding landmarks and obstructions
        let mouse_x: f32 = mouse_position().0;
        let mouse_y: f32 = mouse_position().1;
        if is_mouse_button_released(MouseButton::Left) {
            // delete the obstruction if mouse is touching it
            let mut removed = false;
            for i in 0..obstructions.len() {
                let obstruction = obstructions[i];
                if mouse_x < obstruction.x + obstruction.w &&
                   mouse_x > obstruction.x &&
                   mouse_y < obstruction.y + obstruction.h &&
                   mouse_y > obstruction.y {
                    obstructions.remove(i);
                    removed = true;                 
                    break;
                }
            }
            if !removed && mouse_x < (screen_width()) / 2.0 - OBSTRUCTION_WIDTH {
                obstructions.push(Rect::new(mouse_x, mouse_y, OBSTRUCTION_WIDTH, OBSTRUCTION_HEIGHT));
            }
        }
        if is_mouse_button_released(MouseButton::Right) {
            let mut removed = false;
            for i in 0..landmarks.len() {
                let landmark = landmarks[i];
                if mouse_x < landmark.0 + LANDMARK_RADIUS &&
                   mouse_x > landmark.0 - LANDMARK_RADIUS &&
                   mouse_y < landmark.1 + LANDMARK_RADIUS &&
                   mouse_y > landmark.1 - LANDMARK_RADIUS {
                    landmarks.remove(i);
                    removed = true;                 
                    break;
                }
            }
            if !removed && mouse_x < screen_width() / 2.0 - LANDMARK_RADIUS {
                landmarks.push(mouse_position());
            }
        }
        
        // bound velocity
        linear_velocity = linear_velocity.clamp(-MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        angular_velocity = angular_velocity.clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        // apply decay
        linear_velocity *= DECAY_FACTOR.powf(delta_time);
        angular_velocity *= DECAY_FACTOR.powf(delta_time);
        
        let gt_linear_velocity: f32 = linear_velocity + epsilon(ALPHA_LINEAR * linear_velocity.abs());
        let gt_angular_velocity: f32 = angular_velocity + epsilon(ALPHA_ANGULAR * angular_velocity.abs());

        // update direction
        dir += 0.5 * (gt_angular_velocity + prev_gt_angular_velocity) * delta_time;
        dir = (dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        x += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * dir.cos();
        y += (0.5 * (gt_linear_velocity + prev_gt_linear_velocity) * delta_time) * dir.sin();
        x = x.clamp(0.0 + ROBOT_RADIUS, screen_width() / 2.0 - ROBOT_RADIUS);
        y = y.clamp(0.0 + ROBOT_RADIUS, screen_height() - ROBOT_RADIUS);

        // draw obstructions and landmarks
        for obstruction in obstructions.iter() {
            draw_rectangle(obstruction.x, obstruction.y, obstruction.w, obstruction.h, GRAY);
        }
        for landmark in landmarks.iter() {
            draw_circle(landmark.0, landmark.1, LANDMARK_RADIUS, RED);
        }

        // draw "robot"; segment shows direction
        draw_circle(x, y, ROBOT_RADIUS, BLUE);
        draw_line(x, y, x + ROBOT_RADIUS * dir.cos(), y + ROBOT_RADIUS * dir.sin(), 4.0, WHITE);

        // ground truth text
        draw_text(&format!("pos: ({:.0}, {:.0})", x, y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", dir), 25.0, 100.0, 36.0, WHITE);
        
        // dividing line between ground truth world and robot's perceived world
        draw_line(screen_width() / 2.0, 0.0, screen_width() / 2.0, screen_height(), 4.0, WHITE);


        // needed for calculating x, y, and dir on next frame
        prev_gt_linear_velocity = gt_linear_velocity;
        prev_gt_angular_velocity = gt_angular_velocity;

        next_frame().await
    }
}

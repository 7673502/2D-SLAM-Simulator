use macroquad::prelude::*;

#[macroquad::main("2D EKF SLAM Simulator")]
async fn main() {
    // position states
    let mut x: f32 = screen_width() / 2.0;
    let mut y: f32 = screen_height() / 2.0; 
    let mut linear_velocity: f32 = 0.0;

    // direction states
    let mut dir: f32 = 0.0;
    let mut angular_velocity: f32 = 0.0;

    // constants
    let linear_acc: f32 = 2000.0;
    let angular_acc: f32 = 25.0;
    let max_linear_speed: f32 = 5000.0;
    let max_angular_speed: f32 = 25.0;
    let decay_factor: f32 = 0.05;

    loop {
        clear_background(BLACK);
        
        let delta_time: f32 = get_frame_time();

        let initial_linear_velocity: f32 = linear_velocity;
        let initial_angular_velocity: f32 = angular_velocity;

        // keyboard controls
        if is_key_down(KeyCode::Up) {
            linear_velocity += linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            linear_velocity -= linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            angular_velocity += angular_acc * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            angular_velocity -= angular_acc * delta_time;
        }
        
        // bound velocity
        linear_velocity = linear_velocity.clamp(-max_linear_speed, max_linear_speed);
        angular_velocity = angular_velocity.clamp(-max_angular_speed, max_angular_speed);
        
        // update direction
        dir += 0.5 * (angular_velocity + initial_angular_velocity) * delta_time;
        dir = (dir + 2.0 * std::f32::consts::PI) % (2.0 * std::f32::consts::PI);
        
        // update position
        x += (0.5 * (linear_velocity + initial_linear_velocity) * delta_time) * dir.cos();
        y += (0.5 * (linear_velocity + initial_linear_velocity) * delta_time) * dir.sin();
        x = x.clamp(0.0, screen_width());
        y = y.clamp(0.0, screen_height());

        // draw "robot"; segment shows direction
        draw_circle(x, y, 48.0, BLUE);
        draw_line(x, y, x + 48.0 * dir.cos(), y + 48.0 * dir.sin(), 4.0, WHITE);
        draw_text(&format!("pos: ({:.0}, {:.0})", x, y), 25.0, 50.0, 36.0, WHITE);
        draw_text(&format!("angle: {:.2} rad", dir), 25.0, 100.0, 36.0, WHITE);
        
        // apply decay
        linear_velocity *= decay_factor.powf(delta_time);
        angular_velocity *= decay_factor.powf(delta_time);

        next_frame().await
    }
}

use macroquad::prelude::*;

fn window_conf() -> Conf {
    Conf {
        window_title: "2D EKF SLAM Simulator".to_owned(),
        //sample_count: 10,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut x: f32 = screen_width() / 2.0;
    let mut y: f32 = screen_height() / 2.0; 
    let mut linear_velocity: f32 = 0.0;
    let linear_acc: f32 = 2000.0;
    let decay_factor: f32 = 0.01;

    let mut dir: f32 = 0.0;
    
    let max_linear_speed = 5000.0;

    loop {
        clear_background(RED);
        
        let delta_time = get_frame_time();
        let initial_velocity = linear_velocity;
        if is_key_down(KeyCode::Up) {
            linear_velocity += linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Down) {
            linear_velocity -= linear_acc * delta_time;
        }
        if is_key_down(KeyCode::Right) {
            dir += 1.0 * delta_time;
        }
        if is_key_down(KeyCode::Left) {
            dir -= 1.0 * delta_time;
        }
        
        linear_velocity = linear_velocity.clamp(-max_linear_speed, max_linear_speed);
        
        x += (0.5 * (linear_velocity + initial_velocity) * delta_time) * dir.cos();
        y += (0.5 * (linear_velocity + initial_velocity) * delta_time) * dir.sin();
        //y += (linear_velocity * delta_time - 0.5 * linear_acc * delta_time * delta_time) * dir.sin();
        x = x.clamp(0.0, screen_width());
        y = y.clamp(0.0, screen_height());
        //y = clamp(y - linear_velocity * delta_time, 0.0, screen_height());

        draw_circle(x, y, 48.0, BLUE);
        draw_line(x, y, x + 48.0 * dir.cos(), y + 48.0 * dir.sin(), 2.0, WHITE);
        
        linear_velocity *= decay_factor.powf(delta_time);

        //println!("x: {}, y: {}", x, y);
        next_frame().await
    }
}

//
// Cloth simulation using Verlet integration.
// Using the method described here: http://web.archive.org/web/20070610223835/http://www.teknikus.dk/tj/gdc2001.htm

use macroquad::prelude::*;
use macroquad::telemetry::frame;
use core::num;
use std::ops;
use std::time::{Duration, Instant};
use std::thread::sleep;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl ops::Add<Vec3> for Vec3 {
    type Output = Vec3;

    fn add(self, rhs: Vec3) -> Self::Output {
        let mut result: Vec3 = self;

        result.x += rhs.x;
        result.y += rhs.y;
        result.z += rhs.z;

        result
    }
}

impl ops::Sub<Vec3> for Vec3 {
    type Output = Vec3;

    fn sub(self, rhs: Vec3) -> Self::Output {
        let mut result: Vec3 = self;

        result.x -= rhs.x;
        result.y -= rhs.y;
        result.z -= rhs.z;

        result
    }
}

impl ops::Mul<f32> for Vec3 {
    type Output = Vec3;

    fn mul(self, rhs: f32) -> Self::Output {
        let mut result: Vec3 = self;

        result.x *= rhs;
        result.y *= rhs;
        result.z *= rhs;

        result
    }
}

impl ops::AddAssign<Vec3> for Vec3 {
    fn add_assign(&mut self, rhs: Vec3) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl ops::SubAssign<Vec3> for Vec3 {
    fn sub_assign(&mut self, rhs: Vec3) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;

    }
}

impl Vec3 {
    fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

fn distance(a: Vec3, b: Vec3) -> f32 {
    (a - b).length()
}

fn vclamp(value: Vec3, min: Vec3, max: Vec3) -> Vec3 {
    Vec3 {
        x: clamp(value.x, min.x, max.x),
        y: clamp(value.y, min.y, max.y),
        z: clamp(value.z, min.z, max.z),
    }
}

fn random_i32(min: i32, max: i32) -> i32 {
    let span = (max - min).abs() as u32;

    (rand::rand() % span) as i32 - (span as i32) / 2
}

fn random_f32(min: f32, max: f32) -> f32 {
    union f32_u32 {
        f: f32,
        u: u32,
    }
    
    let mut a = f32_u32 {u: (rand::rand() >> 9) | 0x3f800000};
    
    unsafe {
        a.f -= 1.0;
        
        let span = (max - min).abs();
        a.f *= span;
        a.f += min;

        a.f
    }
}

#[derive(Clone, Copy)]
struct Constraint {
    idx_1: usize,
    idx_2: usize,
    rest_length: f32,
}

#[derive(Clone, Copy)]
struct PointConstraint {
    idx: usize,
    point: Vec3,
}

#[macroquad::main("BasicShapes")]
async fn main() {
    const NUM_COLS:        usize = 10;
    const NUM_ROWS:        usize = 10;
    const NUM_PARTICLES:   usize = NUM_ROWS * NUM_COLS;
    const NUM_CONSTRAINTS: usize = (NUM_ROWS - 1) * NUM_COLS + (NUM_COLS - 1) * NUM_ROWS;
    const NUM_ITERATIONS:  usize = 1;
    const START_DISTANCE:  f32   = 20.0;
    const PARTICLE_RADIUS: f32   = 3.0;
    const INTERSECT_THRESHOLD: f32 = PARTICLE_RADIUS + 3.0;
    
    let mut pos     = [Vec3{x: 0.0, y: 0.0, z: 0.0}; NUM_PARTICLES];
    let mut old_pos = [Vec3{x: 0.0, y: 0.0, z: 0.0}; NUM_PARTICLES];
    let mut forces  = [Vec3{x: 0.0, y: 0.0, z: 0.0}; NUM_PARTICLES];
    let mut constraints = [Constraint{idx_1: 0, idx_2: 0, rest_length: 0.0}; NUM_CONSTRAINTS];
    let mut point_constraints: Vec<PointConstraint> = Vec::new();

    let mut holding_particle = false;
    // The particle that the mouse is "holding"
    let mut held_constraint = PointConstraint {
        idx: 0,
        point: Vec3 {
            x: 0.0, y: 0.0, z: 0.0
        }
    };
    
    let gravity = Vec3{x: 0.0, y: 10.0 * 9.82, z: 0.0};
    let time_step = 0.01666667;

    // Randomized initial conditions
    /*for p in 0..NUM_PARTICLES {
        pos[p].x = screen_width() / 2.0  + random_f32(-90.0, 90.0);
        pos[p].y = screen_height() / 2.0 + random_f32(-90.0, 90.0);

        old_pos[p].x = pos[p].x + random_f32(-1.0, 1.0);
        old_pos[p].y = pos[p].y + random_f32(-1.0, 1.0);
    }*/

    for p in 0..NUM_PARTICLES {
        pos[p].x = screen_width() / 2.0;
        pos[p].y = screen_height() / 2.0;
        
        let cols = 10;
        let row = (p / cols) as f32;
        let col = (p % cols) as f32;

        pos[p].x += col * START_DISTANCE + random_f32(-1.0, 1.0);
        pos[p].y += row * START_DISTANCE + random_f32(-1.0, 1.0);

        old_pos[p] = pos[p];
    }

    let mut c_idx = 0;
    // Horizontal
    for p_x in 0..NUM_ROWS {
        for p_y in 0..NUM_COLS-1 {
            let p_idx = p_x * NUM_COLS + p_y;

            constraints[c_idx].idx_1 = p_idx;
            constraints[c_idx].idx_2 = p_idx + 1;
            constraints[c_idx].rest_length = 20.0;

            c_idx += 1;
        }
    }
    
    // Vertical
    for p_y in 0..NUM_COLS {
        for p_x in 0..NUM_ROWS-1 {
            let p_idx = p_x * NUM_COLS + p_y;

            constraints[c_idx].idx_1 = p_idx;
            constraints[c_idx].idx_2 = p_idx + NUM_COLS;
            constraints[c_idx].rest_length = 20.0;

            c_idx += 1
        }
    }

    point_constraints.push(PointConstraint { idx: 0, point: pos[0]});
    point_constraints.push(PointConstraint { idx: NUM_COLS / 2, point: pos[NUM_COLS / 2]});
    point_constraints.push(PointConstraint { idx: NUM_COLS - 1, point: pos[NUM_COLS - 1]});

    let mut last_frame = Instant::now();
    loop {
        /**** Handle input ****/
        if is_mouse_button_down(MouseButton::Left) {
            let mouse = mouse_position();
            let mouse_vec = Vec3{x: mouse.0, y: mouse.1, z: 0.0};

            if !holding_particle {
                for p in 0..NUM_PARTICLES {
                    
                    if distance(mouse_vec, pos[p]) < INTERSECT_THRESHOLD {
                        holding_particle = true;
                        held_constraint.idx = p;
                        held_constraint.point = mouse_vec;
                        break;
                    }
                }
            }
            else {
                held_constraint.point = mouse_vec;
            }
        }
        else {
            holding_particle = false;
        }


        /**** Update ****/
        // Verlet integration step
        for p in 0..NUM_PARTICLES {
            let tmp = pos[p];
            pos[p] += pos[p] - old_pos[p] + forces[p] * time_step * time_step;
            old_pos[p] = tmp;
        }

        // Accumulate forces
        for p in 0..NUM_PARTICLES {
            forces[p] = gravity;
        }

        // Satisfy constraints
        for p in 0..NUM_PARTICLES {
            pos[p] = vclamp(pos[p], Vec3{x: 0.0, y: 0.0, z: 0.0}, Vec3{x: screen_width(), y: screen_height(), z: 0.0});
        }

        for _i in 0..NUM_ITERATIONS {   
            for c in 0..NUM_CONSTRAINTS {
                let p1 = pos[constraints[c].idx_1];
                let p2 = pos[constraints[c].idx_2];
                
                // NOTE: We can approximate this to avoid the sqrt. Unsure how relevant that is on modern systems.
                let delta = p2 - p1;
                let delta_len = (delta.x * delta.x + delta.y * delta.y + delta.z + delta.z).sqrt();
                let diff_len = (delta_len - constraints[c].rest_length) / delta_len;

                pos[constraints[c].idx_1] += delta * 0.5 * diff_len;
                pos[constraints[c].idx_2] -= delta * 0.5 * diff_len;
            }

            for constraint in &point_constraints {
                pos[constraint.idx] = constraint.point;
            }

            if holding_particle {
                pos[held_constraint.idx] = held_constraint.point;
            }
        }

        /**** Draw ****/
        clear_background(BLACK);
        
        for c in 0..NUM_CONSTRAINTS {
            let p1 = pos[constraints[c].idx_1];
            let p2 = pos[constraints[c].idx_2];
            draw_line(p1.x, p1.y, p2.x, p2.y, 5.0, GRAY);
        }

        for p in 0..NUM_PARTICLES {
            draw_circle(pos[p].x, pos[p].y, PARTICLE_RADIUS, WHITE)
        }

        draw_text(last_frame.elapsed().as_secs_f32().to_string().as_str(), 20.0, 20.0, 20.0, DARKGRAY);

        // finish frame
        last_frame = Instant::now();

        next_frame().await
    }
}
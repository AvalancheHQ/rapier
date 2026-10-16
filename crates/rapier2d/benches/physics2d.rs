//! CodSpeed/divan benchmarks for the 2D physics pipeline.
//!
//! Run locally with `cargo bench -p rapier2d --bench physics2d`, or through CodSpeed with
//! `cargo codspeed build -p rapier2d && cargo codspeed run -p rapier2d`.
//!
//! Each benchmark builds its scene outside of the measured section and only times the
//! physics steps (or queries), so the results track the cost of the engine itself.

use rapier2d::prelude::*;

fn main() {
    divan::main();
}

/// Number of physics steps simulated per measured iteration.
const STEPS: usize = 10;

fn add_ground(world: &mut PhysicsWorld, half_width: Real) {
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(ColliderBuilder::cuboid(half_width, 0.1), Some(ground));
}

/// A pyramid of boxes resting on the ground: contact-heavy, stacked scene.
fn pyramid_scene(base: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    add_ground(&mut world, 200.0);
    let rad = 0.5;
    let shift = rad * 2.0;
    for level in 0..base {
        let n = base - level;
        let offset = -(n as Real) * shift * 0.5;
        for i in 0..n {
            let pos = Vector::new(
                offset + i as Real * shift,
                0.1 + rad + level as Real * shift,
            );
            world.insert(
                RigidBodyBuilder::dynamic().translation(pos),
                ColliderBuilder::cuboid(rad, rad),
            );
        }
    }
    // Let the pyramid settle a bit so the contact graph is populated.
    for _ in 0..5 {
        world.step();
    }
    world
}

/// A grid of balls falling onto the ground: broad-phase and narrow-phase heavy.
fn falling_balls_scene(n: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    add_ground(&mut world, 200.0);
    let rad = 0.5;
    let spacing = rad * 2.5;
    let offset = -(n as Real) * spacing * 0.5;
    for i in 0..n {
        for j in 0..n {
            let pos = Vector::new(offset + i as Real * spacing, 1.0 + j as Real * spacing);
            world.insert(
                RigidBodyBuilder::dynamic().translation(pos),
                ColliderBuilder::ball(rad),
            );
        }
    }
    world
}

/// A grid of bodies linked by revolute joints (as in the `joint grid` example).
fn joint_grid_scene(n: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let shift = 1.0;
    let mut handles = vec![RigidBodyHandle::invalid(); n * n];
    for k in 0..n {
        for i in 0..n {
            let fixed = i == 0 && (k % 4 == 0 || k == n - 1);
            let body = if fixed {
                RigidBodyBuilder::fixed()
            } else {
                RigidBodyBuilder::dynamic()
            };
            let (handle, _) = world.insert(
                body.translation(Vector::new(k as Real * shift, -(i as Real) * shift)),
                ColliderBuilder::ball(0.2),
            );
            let index = k * n + i;
            if i > 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vector::new(0.0, -shift))
                    .local_anchor2(Vector::ZERO)
                    .contacts_enabled(false);
                world.insert_impulse_joint(handles[index - 1], handle, joint);
            }
            if k > 0 {
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vector::new(shift, 0.0))
                    .local_anchor2(Vector::ZERO)
                    .contacts_enabled(false);
                world.insert_impulse_joint(handles[index - n], handle, joint);
            }
            handles[index] = handle;
        }
    }
    world
}

fn run_steps(world: &mut PhysicsWorld) {
    for _ in 0..STEPS {
        world.step();
    }
}

#[divan::bench(args = [10, 30])]
fn step_pyramid(bencher: divan::Bencher, base: usize) {
    bencher
        .with_inputs(|| pyramid_scene(base))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

#[divan::bench(args = [10, 25])]
fn step_falling_balls(bencher: divan::Bencher, n: usize) {
    bencher
        .with_inputs(|| falling_balls_scene(n))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

#[divan::bench]
fn step_joint_grid(bencher: divan::Bencher) {
    bencher
        .with_inputs(|| joint_grid_scene(12))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

/// Ray casts against a settled pyramid scene through the query pipeline.
#[divan::bench]
fn cast_rays(bencher: divan::Bencher) {
    let world = pyramid_scene(20);
    let rays: Vec<Ray> = (0..256)
        .map(|i| {
            let x = i as Real * 0.08 - 10.0;
            Ray::new(Vector::new(x, 50.0), Vector::new(0.0, -1.0))
        })
        .collect();
    bencher.bench_local(|| {
        let mut hits = 0;
        for ray in &rays {
            if world
                .cast_ray(divan::black_box(ray), 100.0, true, QueryFilter::default())
                .is_some()
            {
                hits += 1;
            }
        }
        hits
    });
}

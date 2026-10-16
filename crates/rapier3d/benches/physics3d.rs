//! CodSpeed/divan benchmarks for the 3D physics pipeline.
//!
//! Run locally with `cargo bench -p rapier3d --bench physics3d`, or through CodSpeed with
//! `cargo codspeed build -p rapier3d && cargo codspeed run -p rapier3d`.
//!
//! Each benchmark builds its scene outside of the measured section and only times the
//! physics steps (or queries), so the results track the cost of the engine itself.

use rapier3d::prelude::*;

fn main() {
    divan::main();
}

/// Number of physics steps simulated per measured iteration.
const STEPS: usize = 10;

fn add_ground(world: &mut PhysicsWorld, half_extent: Real) {
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(
        ColliderBuilder::cuboid(half_extent, 0.1, half_extent),
        Some(ground),
    );
}

/// A pyramid of boxes resting on the ground: contact-heavy, stacked scene.
fn pyramid_scene(base: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    add_ground(&mut world, 100.0);
    let rad = 0.5;
    let shift = rad * 2.0;
    for level in 0..base {
        let n = base - level;
        let offset = -(n as Real) * shift * 0.5;
        for i in 0..n {
            for j in 0..n {
                let pos = Vector::new(
                    offset + i as Real * shift,
                    0.1 + rad + level as Real * shift,
                    offset + j as Real * shift,
                );
                world.insert(
                    RigidBodyBuilder::dynamic().translation(pos),
                    ColliderBuilder::cuboid(rad, rad, rad),
                );
            }
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
    add_ground(&mut world, 100.0);
    let rad = 0.5;
    let spacing = rad * 2.5;
    let offset = -(n as Real) * spacing * 0.5;
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let pos = Vector::new(
                    offset + i as Real * spacing,
                    1.0 + j as Real * spacing,
                    offset + k as Real * spacing,
                );
                world.insert(
                    RigidBodyBuilder::dynamic().translation(pos),
                    ColliderBuilder::ball(rad),
                );
            }
        }
    }
    world
}

/// Hanging chains of bodies linked by spherical impulse joints: joint-solver heavy.
fn joint_chains_scene(num_chains: usize, chain_len: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let rad = 0.2;
    let shift = 1.0;
    for c in 0..num_chains {
        let x = c as Real * 2.0;
        let mut parent =
            world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(x, 0.0, 0.0)));
        for i in 0..chain_len {
            let (child, _) = world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(
                    x + (i + 1) as Real * shift,
                    0.0,
                    0.0,
                )),
                ColliderBuilder::ball(rad),
            );
            let joint = SphericalJointBuilder::new()
                .local_anchor1(Vector::new(if i == 0 { 0.0 } else { shift }, 0.0, 0.0))
                .local_anchor2(Vector::ZERO)
                .contacts_enabled(false);
            world.insert_impulse_joint(parent, child, joint);
            parent = child;
        }
    }
    world
}

/// Same chains as [`joint_chains_scene`] but using reduced-coordinates multibody joints.
fn multibody_chains_scene(num_chains: usize, chain_len: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let rad = 0.2;
    let shift = 1.0;
    for c in 0..num_chains {
        let x = c as Real * 2.0;
        let mut parent =
            world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(x, 0.0, 0.0)));
        for i in 0..chain_len {
            let (child, _) = world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(
                    x + (i + 1) as Real * shift,
                    0.0,
                    0.0,
                )),
                ColliderBuilder::ball(rad),
            );
            let joint = SphericalJointBuilder::new()
                .local_anchor1(Vector::new(if i == 0 { 0.0 } else { shift }, 0.0, 0.0))
                .local_anchor2(Vector::ZERO)
                .contacts_enabled(false);
            world.insert_multibody_joint(parent, child, joint);
            parent = child;
        }
    }
    world
}

/// Boxes and balls dropped on a triangle-mesh terrain: exercises mesh contacts.
fn trimesh_scene(n: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let nsub = 20;
    let size = 40.0;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for i in 0..=nsub {
        for j in 0..=nsub {
            let x = -size * 0.5 + size * i as Real / nsub as Real;
            let z = -size * 0.5 + size * j as Real / nsub as Real;
            let y = (x * 0.3).sin() * (z * 0.3).cos();
            vertices.push(Vector::new(x, y, z));
        }
    }
    for i in 0..nsub {
        for j in 0..nsub {
            let a = (i * (nsub + 1) + j) as u32;
            let b = a + 1;
            let c = a + (nsub + 1) as u32;
            let d = c + 1;
            indices.push([a, c, b]);
            indices.push([b, c, d]);
        }
    }
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(
        ColliderBuilder::trimesh(vertices, indices).expect("valid trimesh"),
        Some(ground),
    );
    let offset = -(n as Real) * 0.75;
    for i in 0..n {
        for j in 0..n {
            let pos = Vector::new(offset + i as Real * 1.5, 1.5, offset + j as Real * 1.5);
            let collider = if (i + j) % 2 == 0 {
                ColliderBuilder::cuboid(0.4, 0.4, 0.4)
            } else {
                ColliderBuilder::ball(0.4)
            };
            world.insert(RigidBodyBuilder::dynamic().translation(pos), collider);
        }
    }
    // Let the bodies land on the terrain so mesh contacts are active.
    for _ in 0..30 {
        world.step();
    }
    world
}

fn run_steps(world: &mut PhysicsWorld) {
    for _ in 0..STEPS {
        world.step();
    }
}

#[divan::bench(args = [4, 8])]
fn step_pyramid(bencher: divan::Bencher, base: usize) {
    bencher
        .with_inputs(|| pyramid_scene(base))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

#[divan::bench(args = [5, 8])]
fn step_falling_balls(bencher: divan::Bencher, n: usize) {
    bencher
        .with_inputs(|| falling_balls_scene(n))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

#[divan::bench]
fn step_impulse_joint_chains(bencher: divan::Bencher) {
    bencher
        .with_inputs(|| joint_chains_scene(5, 20))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

#[divan::bench]
fn step_multibody_chains(bencher: divan::Bencher) {
    bencher
        .with_inputs(|| multibody_chains_scene(2, 10))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

#[divan::bench]
fn step_trimesh_terrain(bencher: divan::Bencher) {
    bencher
        .with_inputs(|| trimesh_scene(8))
        .bench_local_values(|mut world| {
            run_steps(&mut world);
            world
        });
}

/// Cost of building a scene from scratch (body/collider insertion) and its first step.
#[divan::bench]
fn build_and_first_step(bencher: divan::Bencher) {
    bencher.bench_local(|| {
        let mut world = falling_balls_scene(6);
        world.step();
        world
    });
}

/// Ray casts against a settled pyramid scene through the query pipeline.
#[divan::bench]
fn cast_rays(bencher: divan::Bencher) {
    let world = pyramid_scene(6);
    let rays: Vec<Ray> = (0..256)
        .map(|i| {
            let x = (i % 16) as Real * 0.5 - 4.0;
            let z = (i / 16) as Real * 0.5 - 4.0;
            Ray::new(Vector::new(x, 20.0, z), Vector::new(0.0, -1.0, 0.0))
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

//! HORUS Communication Systems
//!
//! Native systems that read/write HORUS topics to control robots and publish sensor data.
//! Uses existing sim3d components (CmdVel, DifferentialDrive, RigidBodyComponent).

use bevy::prelude::*;
use horus_core::core::NodeInfo;

use crate::horus_native::HorusComm;
use crate::physics::diff_drive::CmdVel;
use crate::physics::rigid_body::RigidBodyComponent;
use crate::physics::PhysicsWorld;

/// Component linking a Bevy entity to a HORUS robot name
#[derive(Component, Clone)]
pub struct HorusRobot {
    pub name: String,
}

/// System to receive cmd_vel from HORUS and update CmdVel component
pub fn horus_cmd_vel_system(
    mut horus_comm: Option<ResMut<HorusComm>>,
    mut robots: Query<(&HorusRobot, &mut CmdVel)>,
) {
    let Some(ref mut comm) = horus_comm else {
        return;
    };

    for (horus_robot, mut cmd_vel) in robots.iter_mut() {
        if let Some(hubs) = comm.robot_hubs.get_mut(&horus_robot.name) {
            if let Some(ref mut hub) = hubs.cmd_vel_sub {
                // Non-blocking receive
                if let Some(twist) = hub.recv(&mut None::<&mut NodeInfo>) {
                    // Twist.linear is [f64; 3], use [0] for forward velocity
                    // Twist.angular is [f64; 3], use [2] for yaw rate
                    cmd_vel.linear = twist.linear[0] as f32;
                    cmd_vel.angular = twist.angular[2] as f32;
                }
            }
        }
    }
}

/// System to publish odometry from robot physics state
pub fn horus_odom_publish_system(
    mut horus_comm: Option<ResMut<HorusComm>>,
    physics_world: Res<PhysicsWorld>,
    robots: Query<(&HorusRobot, &RigidBodyComponent)>,
) {
    let Some(ref mut comm) = horus_comm else {
        return;
    };

    for (horus_robot, rb_comp) in robots.iter() {
        if let Some(hubs) = comm.robot_hubs.get_mut(&horus_robot.name) {
            if let Some(ref mut hub) = hubs.odom_pub {
                if let Some(rb) = physics_world.rigid_body_set.get(rb_comp.handle) {
                    let pos = rb.translation();
                    let rot = rb.rotation();
                    let linvel = rb.linvel();
                    let angvel = rb.angvel();

                    // Extract yaw from quaternion
                    let yaw = rot.euler_angles().2 as f64;

                    use horus_library::messages::geometry::{Pose2D, Twist};
                    use horus_library::messages::sensor::Odometry;

                    let timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as u64;

                    let mut odom = Odometry {
                        pose: Pose2D {
                            x: pos.x as f64,
                            y: pos.z as f64, // Bevy Y-up to standard
                            theta: yaw,
                            timestamp,
                        },
                        twist: Twist {
                            linear: [linvel.x as f64, linvel.z as f64, linvel.y as f64],
                            angular: [angvel.x as f64, angvel.z as f64, angvel.y as f64],
                            timestamp,
                        },
                        pose_covariance: [0.0; 36],
                        twist_covariance: [0.0; 36],
                        frame_id: [0; 32],
                        child_frame_id: [0; 32],
                        timestamp,
                    };

                    // Set frame IDs
                    let frame = format!("{}/odom", horus_robot.name);
                    let child = format!("{}/base_link", horus_robot.name);
                    odom.set_frames(&frame, &child);

                    let _ = hub.send(odom, &mut None::<&mut NodeInfo>);
                }
            }
        }
    }
}

/// System to publish IMU data from robot physics state
pub fn horus_imu_publish_system(
    mut horus_comm: Option<ResMut<HorusComm>>,
    physics_world: Res<PhysicsWorld>,
    robots: Query<(&HorusRobot, &RigidBodyComponent)>,
    mut prev_linvel: Local<std::collections::HashMap<String, [f32; 3]>>,
    time: Res<Time>,
) {
    let Some(ref mut comm) = horus_comm else {
        return;
    };

    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for (horus_robot, rb_comp) in robots.iter() {
        if let Some(hubs) = comm.robot_hubs.get_mut(&horus_robot.name) {
            if let Some(ref mut hub) = hubs.imu_pub {
                if let Some(rb) = physics_world.rigid_body_set.get(rb_comp.handle) {
                    let rot = rb.rotation();
                    let linvel = rb.linvel();
                    let angvel = rb.angvel();

                    // Compute acceleration from velocity change
                    let prev = prev_linvel
                        .entry(horus_robot.name.clone())
                        .or_insert([0.0, 0.0, 0.0]);
                    let accel = [
                        ((linvel.x - prev[0]) / dt) as f64,
                        ((linvel.y - prev[1]) / dt + 9.81) as f64, // Add gravity
                        ((linvel.z - prev[2]) / dt) as f64,
                    ];
                    *prev = [linvel.x, linvel.y, linvel.z];

                    use horus_library::messages::sensor::Imu;

                    let imu = Imu {
                        orientation: [rot.i as f64, rot.j as f64, rot.k as f64, rot.w as f64],
                        orientation_covariance: [0.0; 9],
                        angular_velocity: [angvel.x as f64, angvel.y as f64, angvel.z as f64],
                        angular_velocity_covariance: [0.0; 9],
                        linear_acceleration: accel,
                        linear_acceleration_covariance: [0.0; 9],
                        timestamp: std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap_or_default()
                            .as_nanos() as u64,
                    };

                    let _ = hub.send(imu, &mut None::<&mut NodeInfo>);
                }
            }
        }
    }
}

/// Plugin to register HORUS communication systems
pub struct HorusCommPlugin;

impl Plugin for HorusCommPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                horus_cmd_vel_system,
                horus_odom_publish_system,
                horus_imu_publish_system,
            ),
        );
    }
}

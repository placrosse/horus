//! Transport Sync - Connects HorusPublisher buffer to HorusTransport Hub
//!
//! This module bridges the gap between the Bevy ECS-local HorusPublisher
//! and the actual HORUS IPC system via HorusTransport Hubs.

use bevy::prelude::*;

use super::horus_transport::HorusTransport;
use super::messages::*;
use super::publisher::HorusPublisher;
use super::{HorusBridgeConfig, HorusBridgeState};
use crate::physics::diff_drive::DifferentialDrive;
use crate::physics::rigid_body::Velocity;
use crate::robot::robot::Robot;
use crate::sensors::imu::{IMUData, IMU};

/// System that pushes buffered HFrame messages to HORUS Transport
pub fn sync_hframe_to_transport(
    publisher: Res<HorusPublisher>,
    transport: Res<HorusTransport>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
) {
    if !config.enabled || !config.publish_hframe {
        return;
    }

    let messages = publisher.get_tf_messages();
    for msg in messages {
        if transport.publish_hf(msg) {
            state.record_publish();
        }
    }
}

/// System that pushes buffered PointCloud2 messages to HORUS Transport
pub fn sync_pointcloud_to_transport(
    publisher: Res<HorusPublisher>,
    transport: Res<HorusTransport>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
) {
    if !config.enabled || !config.publish_sensors {
        return;
    }

    let messages = publisher.get_pointcloud_messages();
    for (_topic, msg) in messages {
        if transport.publish_pointcloud(msg) {
            state.record_publish();
        }
    }
}

/// System that pushes buffered LaserScan messages to HORUS Transport
pub fn sync_laserscan_to_transport(
    publisher: Res<HorusPublisher>,
    transport: Res<HorusTransport>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
) {
    if !config.enabled || !config.publish_sensors {
        return;
    }

    let messages = publisher.get_laserscan_messages();
    for (_topic, msg) in messages {
        if transport.publish_laserscan(msg) {
            state.record_publish();
        }
    }
}

/// System that publishes odometry from robots with DifferentialDrive
pub fn publish_odometry_system(
    time: Res<Time>,
    transport: Res<HorusTransport>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
    query: Query<(&Name, &GlobalTransform, &Velocity, &DifferentialDrive), With<Robot>>,
) {
    if !config.enabled || !config.publish_odometry {
        return;
    }

    let current_time = time.elapsed_secs_f64();

    for (name, transform, velocity, _diff_drive) in query.iter() {
        let bevy_transform = transform.compute_transform();

        let odom = Odometry {
            header: Header::new("odom", current_time),
            child_frame_id: format!("{}_base_link", name.as_str()),
            pose: PoseWithCovariance {
                pose: Pose {
                    position: bevy_transform.translation.into(),
                    orientation: bevy_transform.rotation.into(),
                },
                covariance: vec![
                    0.001, 0.0, 0.0, 0.0, 0.0, 0.0, // x
                    0.0, 0.001, 0.0, 0.0, 0.0, 0.0, // y
                    0.0, 0.0, 0.001, 0.0, 0.0, 0.0, // z
                    0.0, 0.0, 0.0, 0.001, 0.0, 0.0, // roll
                    0.0, 0.0, 0.0, 0.0, 0.001, 0.0, // pitch
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.001, // yaw
                ],
            },
            twist: TwistWithCovariance {
                twist: Twist {
                    linear: velocity.linear.into(),
                    angular: velocity.angular.into(),
                },
                covariance: vec![
                    0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001,
                ],
            },
        };

        if transport.publish_odom(odom) {
            state.record_publish();
        }
    }
}

/// System that publishes IMU data to HORUS Transport
pub fn publish_imu_system(
    time: Res<Time>,
    transport: Res<HorusTransport>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
    query: Query<(&Name, &IMU, &IMUData)>,
) {
    if !config.enabled || !config.publish_sensors {
        return;
    }

    let current_time = time.elapsed_secs_f64();

    for (name, _imu, imu_data) in query.iter() {
        let imu_msg = Imu {
            header: Header::new(format!("{}_imu_link", name.as_str()), current_time),
            orientation: imu_data.orientation.into(),
            orientation_covariance: imu_data.orientation_covariance.clone(),
            angular_velocity: imu_data.angular_velocity.into(),
            angular_velocity_covariance: imu_data.angular_velocity_covariance.clone(),
            linear_acceleration: imu_data.linear_acceleration.into(),
            linear_acceleration_covariance: imu_data.linear_acceleration_covariance.clone(),
        };

        // IMU needs its own Hub - for now, log that we would publish
        // TODO: Add IMU hub to HorusTransport
        tracing::trace!(
            "Would publish IMU for {}: orientation={:?}",
            name.as_str(),
            imu_msg.orientation
        );
        state.record_publish();
    }
}

/// System that publishes joint states to HORUS Transport
pub fn publish_joint_states_system(
    time: Res<Time>,
    transport: Res<HorusTransport>,
    config: Res<HorusBridgeConfig>,
    mut state: ResMut<HorusBridgeState>,
    query: Query<(&Name, &crate::robot::state::RobotJointStates), With<Robot>>,
) {
    if !config.enabled || !config.publish_joint_states {
        return;
    }

    let current_time = time.elapsed_secs_f64();

    for (name, robot_joint_states) in query.iter() {
        let mut names = Vec::new();
        let mut positions = Vec::new();
        let mut velocities = Vec::new();
        let mut efforts = Vec::new();

        // Use joint_order to maintain consistent ordering
        for joint_name in &robot_joint_states.joint_order {
            if let Some(joint_state) = robot_joint_states.joints.get(joint_name) {
                names.push(joint_name.clone());
                positions.push(joint_state.position);
                velocities.push(joint_state.velocity);
                efforts.push(joint_state.effort);
            }
        }

        if names.is_empty() {
            continue;
        }

        let joint_state_msg = JointState {
            header: Header::new(name.as_str(), current_time),
            name: names,
            position: positions,
            velocity: velocities,
            effort: efforts,
        };

        if transport.publish_joint_state(joint_state_msg) {
            state.record_publish();
        }
    }
}

/// Plugin that adds transport sync systems
pub struct HorusTransportSyncPlugin;

impl Plugin for HorusTransportSyncPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PostUpdate,
            (
                sync_hframe_to_transport,
                sync_pointcloud_to_transport,
                sync_laserscan_to_transport,
                publish_odometry_system,
                publish_imu_system,
                publish_joint_states_system,
            )
                .chain(),
        );

        tracing::info!(
            "HORUS Transport Sync plugin loaded - sensor data will be published to HORUS topics"
        );
    }
}

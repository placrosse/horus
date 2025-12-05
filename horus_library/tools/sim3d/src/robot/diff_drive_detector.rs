//! Automatic differential drive detection from URDF joint structure
//!
//! This module detects differential drive robots by analyzing the URDF joint
//! configuration, even when no explicit Gazebo plugin is specified.

use crate::robot::gazebo::DifferentialDriveConfig;
use tracing::info;
use urdf_rs::Robot as URDFRobot;

/// Detects if a URDF represents a differential drive robot and extracts configuration
pub struct DiffDriveDetector;

impl DiffDriveDetector {
    /// Analyze a URDF robot and detect if it's a differential drive robot
    ///
    /// Detection criteria:
    /// 1. Has exactly 2 continuous wheel joints (or revolute with no limits)
    /// 2. Wheels are on opposite sides (symmetric Y offset)
    /// 3. Wheel joints share the same rotation axis
    pub fn detect(urdf: &URDFRobot) -> Option<DifferentialDriveConfig> {
        // Find potential wheel joints (continuous or unlimited revolute)
        let wheel_joints: Vec<_> = urdf
            .joints
            .iter()
            .filter(|j| Self::is_wheel_joint(j))
            .collect();

        if wheel_joints.len() < 2 {
            return None;
        }

        // Try to find a left-right wheel pair
        for i in 0..wheel_joints.len() {
            for j in (i + 1)..wheel_joints.len() {
                let left = wheel_joints[i];
                let right = wheel_joints[j];

                if let Some(config) = Self::analyze_wheel_pair(urdf, left, right) {
                    info!(
                        "Auto-detected differential drive: left='{}', right='{}', wheel_sep={:.3}m, wheel_radius={:.3}m",
                        config.left_joint, config.right_joint, config.wheel_separation, config.wheel_diameter / 2.0
                    );
                    return Some(config);
                }
            }
        }

        None
    }

    /// Check if a joint is likely a wheel joint
    fn is_wheel_joint(joint: &urdf_rs::Joint) -> bool {
        // Must be continuous or revolute
        let is_rotational = matches!(
            joint.joint_type,
            urdf_rs::JointType::Continuous | urdf_rs::JointType::Revolute
        );

        if !is_rotational {
            return false;
        }

        // Check for wheel-related naming patterns
        let name_lower = joint.name.to_lowercase();
        let child_lower = joint.child.link.to_lowercase();

        let is_wheel_name = name_lower.contains("wheel")
            || child_lower.contains("wheel")
            || name_lower.contains("drive")
            || child_lower.contains("drive");

        // Also accept joints with horizontal rotation axis (typical for wheels)
        let has_wheel_axis = {
            let axis = &joint.axis.xyz;
            // Wheel typically rotates around Y (left-right axis) in robot frame
            // or X axis in some configurations
            (axis[1].abs() > 0.9 || axis[0].abs() > 0.9) && axis[2].abs() < 0.1
        };

        is_wheel_name || has_wheel_axis
    }

    /// Analyze a potential wheel pair to extract differential drive parameters
    fn analyze_wheel_pair(
        urdf: &URDFRobot,
        joint1: &urdf_rs::Joint,
        joint2: &urdf_rs::Joint,
    ) -> Option<DifferentialDriveConfig> {
        // Get joint origins (Y position determines left/right)
        let y1 = joint1.origin.xyz[1];
        let y2 = joint2.origin.xyz[1];

        // Wheels should be on opposite sides
        if y1 * y2 >= 0.0 && (y1 - y2).abs() < 0.01 {
            // Both on same side or too close together
            return None;
        }

        // Determine left (positive Y) and right (negative Y)
        let (left_joint, right_joint, left_y, right_y) = if y1 > y2 {
            (joint1, joint2, y1, y2)
        } else {
            (joint2, joint1, y2, y1)
        };

        // Calculate wheel separation
        let wheel_separation = (left_y - right_y).abs() as f32;

        // Try to estimate wheel radius from link geometry
        let wheel_radius = Self::estimate_wheel_radius(urdf, &left_joint.child.link)
            .or_else(|| Self::estimate_wheel_radius(urdf, &right_joint.child.link))
            .unwrap_or(0.033); // Default TurtleBot3 wheel radius

        Some(DifferentialDriveConfig {
            left_joint: left_joint.name.clone(),
            right_joint: right_joint.name.clone(),
            wheel_separation,
            wheel_diameter: wheel_radius * 2.0,
            max_wheel_torque: 5.0,       // Default torque
            max_wheel_acceleration: 2.0, // Default acceleration
            command_topic: "cmd_vel".to_string(),
            odometry_topic: "odom".to_string(),
        })
    }

    /// Estimate wheel radius from link geometry
    fn estimate_wheel_radius(urdf: &URDFRobot, link_name: &str) -> Option<f32> {
        let link = urdf.links.iter().find(|l| l.name == link_name)?;

        // Check collision geometry first (more accurate for physics)
        for collision in &link.collision {
            if let Some(radius) = Self::extract_radius_from_geometry(&collision.geometry) {
                return Some(radius);
            }
        }

        // Fall back to visual geometry
        for visual in &link.visual {
            if let Some(radius) = Self::extract_radius_from_geometry(&visual.geometry) {
                return Some(radius);
            }
        }

        None
    }

    /// Extract radius from cylinder or sphere geometry
    fn extract_radius_from_geometry(geometry: &urdf_rs::Geometry) -> Option<f32> {
        match geometry {
            urdf_rs::Geometry::Cylinder { radius, .. } => Some(*radius as f32),
            urdf_rs::Geometry::Sphere { radius } => Some(*radius as f32),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_diff_drive_from_joint_names() {
        let urdf_str = r#"
        <robot name="test_robot">
            <link name="base_link"/>
            <link name="left_wheel"/>
            <link name="right_wheel"/>
            <joint name="left_wheel_joint" type="continuous">
                <parent link="base_link"/>
                <child link="left_wheel"/>
                <origin xyz="0 0.1 0"/>
                <axis xyz="0 1 0"/>
            </joint>
            <joint name="right_wheel_joint" type="continuous">
                <parent link="base_link"/>
                <child link="right_wheel"/>
                <origin xyz="0 -0.1 0"/>
                <axis xyz="0 1 0"/>
            </joint>
        </robot>
        "#;

        let urdf = urdf_rs::read_from_string(urdf_str).unwrap();
        let config = DiffDriveDetector::detect(&urdf);

        assert!(config.is_some(), "Should detect differential drive");
        let config = config.unwrap();
        assert_eq!(config.left_joint, "left_wheel_joint");
        assert_eq!(config.right_joint, "right_wheel_joint");
        assert!((config.wheel_separation - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_detect_diff_drive_with_cylinder_geometry() {
        let urdf_str = r#"
        <robot name="test_robot">
            <link name="base_link"/>
            <link name="left_wheel">
                <collision>
                    <geometry>
                        <cylinder radius="0.04" length="0.02"/>
                    </geometry>
                </collision>
            </link>
            <link name="right_wheel">
                <collision>
                    <geometry>
                        <cylinder radius="0.04" length="0.02"/>
                    </geometry>
                </collision>
            </link>
            <joint name="left_wheel_joint" type="continuous">
                <parent link="base_link"/>
                <child link="left_wheel"/>
                <origin xyz="0 0.085 0"/>
                <axis xyz="0 1 0"/>
            </joint>
            <joint name="right_wheel_joint" type="continuous">
                <parent link="base_link"/>
                <child link="right_wheel"/>
                <origin xyz="0 -0.085 0"/>
                <axis xyz="0 1 0"/>
            </joint>
        </robot>
        "#;

        let urdf = urdf_rs::read_from_string(urdf_str).unwrap();
        let config = DiffDriveDetector::detect(&urdf);

        assert!(config.is_some());
        let config = config.unwrap();
        assert!((config.wheel_diameter / 2.0 - 0.04).abs() < 0.001);
    }

    #[test]
    fn test_no_detection_for_non_diff_drive() {
        let urdf_str = r#"
        <robot name="arm_robot">
            <link name="base_link"/>
            <link name="link1"/>
            <link name="link2"/>
            <joint name="joint1" type="revolute">
                <parent link="base_link"/>
                <child link="link1"/>
                <origin xyz="0 0 0.1"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
            </joint>
            <joint name="joint2" type="revolute">
                <parent link="link1"/>
                <child link="link2"/>
                <origin xyz="0 0 0.2"/>
                <axis xyz="0 1 0"/>
                <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
            </joint>
        </robot>
        "#;

        let urdf = urdf_rs::read_from_string(urdf_str).unwrap();
        let config = DiffDriveDetector::detect(&urdf);

        assert!(
            config.is_none(),
            "Should not detect diff drive for arm robot"
        );
    }
}

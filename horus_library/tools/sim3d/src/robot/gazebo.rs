//! Gazebo URDF extension parser
//!
//! Parses <gazebo> tags in URDF files to extract Gazebo-specific
//! materials, physics properties, sensors, and plugins.

use anyhow::{Context, Result};
use roxmltree::{Document, Node};
use std::collections::HashMap;

/// Gazebo extensions parsed from URDF
#[derive(Debug, Clone, Default)]
pub struct GazeboExtensions {
    /// Material properties per link
    pub link_materials: HashMap<String, GazeboMaterial>,
    /// Physics properties per link
    pub link_physics: HashMap<String, GazeboPhysics>,
    /// Sensors attached to links
    pub sensors: Vec<GazeboSensor>,
    /// Gazebo plugins
    pub plugins: Vec<GazeboPlugin>,
    /// Differential drive configuration (if detected)
    pub diff_drive: Option<DifferentialDriveConfig>,
}

/// Differential drive configuration parsed from Gazebo plugin
#[derive(Debug, Clone)]
pub struct DifferentialDriveConfig {
    pub left_joint: String,
    pub right_joint: String,
    pub wheel_separation: f32,
    pub wheel_diameter: f32,
    pub max_wheel_torque: f32,
    pub max_wheel_acceleration: f32,
    pub command_topic: String,
    pub odometry_topic: String,
}

impl Default for DifferentialDriveConfig {
    fn default() -> Self {
        Self {
            left_joint: "left_wheel_joint".to_string(),
            right_joint: "right_wheel_joint".to_string(),
            wheel_separation: 0.17,
            wheel_diameter: 0.066,
            max_wheel_torque: 5.0,
            max_wheel_acceleration: 2.0,
            command_topic: "cmd_vel".to_string(),
            odometry_topic: "odom".to_string(),
        }
    }
}

/// Gazebo material definition
#[derive(Debug, Clone)]
pub struct GazeboMaterial {
    pub ambient: Option<[f32; 4]>,
    pub diffuse: Option<[f32; 4]>,
    pub specular: Option<[f32; 4]>,
    pub emissive: Option<[f32; 4]>,
}

/// Gazebo physics properties
#[derive(Debug, Clone)]
pub struct GazeboPhysics {
    pub gravity: Option<bool>,
    pub mu1: Option<f32>, // Friction coefficient 1
    pub mu2: Option<f32>, // Friction coefficient 2
    pub kp: Option<f32>,  // Contact stiffness
    pub kd: Option<f32>,  // Contact damping
    pub max_contacts: Option<u32>,
}

/// Gazebo sensor definition
#[derive(Debug, Clone)]
pub struct GazeboSensor {
    pub name: String,
    pub sensor_type: SensorType,
    pub link_name: String,
    pub update_rate: f32,
    pub always_on: bool,
    pub visualize: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum SensorType {
    Ray,         // Laser scanner
    Camera,      // RGB camera
    Imu,         // Inertial measurement unit
    Contact,     // Contact sensor
    Sonar,       // Sonar/ultrasonic
    Gps,         // GPS sensor
    ForceTorque, // Force/torque sensor
}

/// Gazebo plugin definition
#[derive(Debug, Clone)]
pub struct GazeboPlugin {
    pub name: String,
    pub filename: String,
    pub parameters: HashMap<String, String>,
}

/// Parser for Gazebo URDF extensions
pub struct GazeboExtensionParser;

impl GazeboExtensionParser {
    /// Parse Gazebo extensions from URDF XML string
    pub fn parse(urdf_xml: &str) -> Result<GazeboExtensions> {
        let doc =
            Document::parse(urdf_xml).context("Failed to parse URDF XML for Gazebo extensions")?;

        let mut extensions = GazeboExtensions::default();

        // Find all <gazebo> tags
        for node in doc.descendants() {
            if node.tag_name().name() == "gazebo" {
                Self::parse_gazebo_tag(&node, &mut extensions)?;
            }
        }

        Ok(extensions)
    }

    fn parse_gazebo_tag(node: &Node, extensions: &mut GazeboExtensions) -> Result<()> {
        let reference = node.attribute("reference");

        if let Some(link_name) = reference {
            // Link-specific extensions
            if let Some(material) = Self::parse_material(node) {
                extensions
                    .link_materials
                    .insert(link_name.to_string(), material);
            }

            if let Some(physics) = Self::parse_physics(node) {
                extensions
                    .link_physics
                    .insert(link_name.to_string(), physics);
            }

            if let Some(sensor) = Self::parse_sensor(node, link_name) {
                extensions.sensors.push(sensor);
            }
        } else {
            // Global extensions (plugins, etc.)
            if let Some(plugin) = Self::parse_plugin(node) {
                // Check if this is a differential drive plugin
                if Self::is_diff_drive_plugin(&plugin) {
                    extensions.diff_drive = Some(Self::parse_diff_drive_config(&plugin));
                }
                extensions.plugins.push(plugin);
            }
        }

        Ok(())
    }

    /// Check if a plugin is a differential drive controller
    fn is_diff_drive_plugin(plugin: &GazeboPlugin) -> bool {
        // Check filename patterns used by various diff drive plugins
        let diff_drive_filenames = [
            "libgazebo_ros_diff_drive",
            "diff_drive",
            "differential_drive",
            "libdiff_drive_controller",
        ];

        let filename_lower = plugin.filename.to_lowercase();
        let name_lower = plugin.name.to_lowercase();

        diff_drive_filenames
            .iter()
            .any(|pattern| filename_lower.contains(pattern) || name_lower.contains(pattern))
    }

    /// Parse differential drive configuration from plugin parameters
    fn parse_diff_drive_config(plugin: &GazeboPlugin) -> DifferentialDriveConfig {
        let params = &plugin.parameters;

        DifferentialDriveConfig {
            left_joint: params
                .get("left_joint")
                .or_else(|| params.get("leftJoint"))
                .cloned()
                .unwrap_or_else(|| "left_wheel_joint".to_string()),
            right_joint: params
                .get("right_joint")
                .or_else(|| params.get("rightJoint"))
                .cloned()
                .unwrap_or_else(|| "right_wheel_joint".to_string()),
            wheel_separation: params
                .get("wheel_separation")
                .or_else(|| params.get("wheelSeparation"))
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.17),
            wheel_diameter: params
                .get("wheel_diameter")
                .or_else(|| params.get("wheelDiameter"))
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.066),
            max_wheel_torque: params
                .get("max_wheel_torque")
                .or_else(|| params.get("torque"))
                .and_then(|s| s.parse().ok())
                .unwrap_or(5.0),
            max_wheel_acceleration: params
                .get("max_wheel_acceleration")
                .and_then(|s| s.parse().ok())
                .unwrap_or(2.0),
            command_topic: params
                .get("command_topic")
                .or_else(|| params.get("commandTopic"))
                .cloned()
                .unwrap_or_else(|| "cmd_vel".to_string()),
            odometry_topic: params
                .get("odometry_topic")
                .or_else(|| params.get("odometryTopic"))
                .cloned()
                .unwrap_or_else(|| "odom".to_string()),
        }
    }

    fn parse_material(node: &Node) -> Option<GazeboMaterial> {
        let material_node = node
            .children()
            .find(|n| n.tag_name().name() == "material")?;

        let mut material = GazeboMaterial {
            ambient: None,
            diffuse: None,
            specular: None,
            emissive: None,
        };

        for child in material_node.children() {
            let tag = child.tag_name().name();
            let text = child.text()?;

            match tag {
                "ambient" => material.ambient = Self::parse_rgba(text),
                "diffuse" => material.diffuse = Self::parse_rgba(text),
                "specular" => material.specular = Self::parse_rgba(text),
                "emissive" => material.emissive = Self::parse_rgba(text),
                _ => {}
            }
        }

        Some(material)
    }

    fn parse_rgba(text: &str) -> Option<[f32; 4]> {
        let parts: Vec<f32> = text
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();

        if parts.len() == 4 {
            Some([parts[0], parts[1], parts[2], parts[3]])
        } else {
            None
        }
    }

    fn parse_physics(node: &Node) -> Option<GazeboPhysics> {
        let mut physics = GazeboPhysics {
            gravity: None,
            mu1: None,
            mu2: None,
            kp: None,
            kd: None,
            max_contacts: None,
        };

        let mut found_any = false;

        for child in node.children() {
            let tag = child.tag_name().name();

            match tag {
                "gravity" => {
                    if let Some(text) = child.text() {
                        physics.gravity = text.parse::<bool>().ok().or_else(|| {
                            // Handle "true"/"false" or "1"/"0"
                            if text == "1" || text.to_lowercase() == "true" {
                                Some(true)
                            } else if text == "0" || text.to_lowercase() == "false" {
                                Some(false)
                            } else {
                                None
                            }
                        });
                        found_any = true;
                    }
                }
                "mu1" => {
                    physics.mu1 = child.text().and_then(|t| t.parse().ok());
                    found_any = true;
                }
                "mu2" => {
                    physics.mu2 = child.text().and_then(|t| t.parse().ok());
                    found_any = true;
                }
                "kp" => {
                    physics.kp = child.text().and_then(|t| t.parse().ok());
                    found_any = true;
                }
                "kd" => {
                    physics.kd = child.text().and_then(|t| t.parse().ok());
                    found_any = true;
                }
                "max_contacts" => {
                    physics.max_contacts = child.text().and_then(|t| t.parse().ok());
                    found_any = true;
                }
                _ => {}
            }
        }

        if found_any {
            Some(physics)
        } else {
            None
        }
    }

    fn parse_sensor(node: &Node, link_name: &str) -> Option<GazeboSensor> {
        let sensor_node = node.children().find(|n| n.tag_name().name() == "sensor")?;

        let sensor_type_str = sensor_node.attribute("type")?;
        let name = sensor_node.attribute("name")?;

        let sensor_type = match sensor_type_str {
            "ray" => SensorType::Ray,
            "camera" => SensorType::Camera,
            "imu" => SensorType::Imu,
            "contact" => SensorType::Contact,
            "sonar" => SensorType::Sonar,
            "gps" => SensorType::Gps,
            "force_torque" => SensorType::ForceTorque,
            _ => {
                tracing::warn!("Unknown Gazebo sensor type: {}", sensor_type_str);
                return None;
            }
        };

        // Parse update rate
        let update_rate = sensor_node
            .children()
            .find(|n| n.tag_name().name() == "update_rate")
            .and_then(|n| n.text())
            .and_then(|t| t.parse().ok())
            .unwrap_or(30.0);

        // Parse always_on
        let always_on = sensor_node
            .children()
            .find(|n| n.tag_name().name() == "always_on")
            .and_then(|n| n.text())
            .map(|t| t == "1" || t.to_lowercase() == "true")
            .unwrap_or(true);

        // Parse visualize
        let visualize = sensor_node
            .children()
            .find(|n| n.tag_name().name() == "visualize")
            .and_then(|n| n.text())
            .map(|t| t == "1" || t.to_lowercase() == "true")
            .unwrap_or(false);

        Some(GazeboSensor {
            name: name.to_string(),
            sensor_type,
            link_name: link_name.to_string(),
            update_rate,
            always_on,
            visualize,
        })
    }

    fn parse_plugin(node: &Node) -> Option<GazeboPlugin> {
        let plugin_node = node.children().find(|n| n.tag_name().name() == "plugin")?;

        let name = plugin_node.attribute("name")?;
        let filename = plugin_node.attribute("filename")?;

        let mut parameters = HashMap::new();

        // Parse all child elements as parameters
        for child in plugin_node.children().filter(|n| n.is_element()) {
            let key = child.tag_name().name();
            if let Some(value) = child.text() {
                parameters.insert(key.to_string(), value.to_string());
            }
        }

        Some(GazeboPlugin {
            name: name.to_string(),
            filename: filename.to_string(),
            parameters,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_gazebo_material() {
        let urdf_xml = r#"
        <robot name="test">
            <gazebo reference="link1">
                <material>
                    <ambient>1 0 0 1</ambient>
                    <diffuse>1 0 0 1</diffuse>
                </material>
            </gazebo>
        </robot>
        "#;

        let extensions = GazeboExtensionParser::parse(urdf_xml).unwrap();
        assert_eq!(extensions.link_materials.len(), 1);

        let material = extensions.link_materials.get("link1").unwrap();
        assert_eq!(material.ambient, Some([1.0, 0.0, 0.0, 1.0]));
        assert_eq!(material.diffuse, Some([1.0, 0.0, 0.0, 1.0]));
    }

    #[test]
    fn test_parse_gazebo_physics() {
        let urdf_xml = r#"
        <robot name="test">
            <gazebo reference="link1">
                <mu1>0.5</mu1>
                <mu2>0.5</mu2>
                <kp>1000000</kp>
                <kd>100</kd>
            </gazebo>
        </robot>
        "#;

        let extensions = GazeboExtensionParser::parse(urdf_xml).unwrap();
        assert_eq!(extensions.link_physics.len(), 1);

        let physics = extensions.link_physics.get("link1").unwrap();
        assert_eq!(physics.mu1, Some(0.5));
        assert_eq!(physics.mu2, Some(0.5));
        assert_eq!(physics.kp, Some(1000000.0));
        assert_eq!(physics.kd, Some(100.0));
    }

    #[test]
    fn test_parse_gazebo_sensor() {
        let urdf_xml = r#"
        <robot name="test">
            <gazebo reference="laser_link">
                <sensor type="ray" name="laser">
                    <update_rate>10</update_rate>
                    <always_on>1</always_on>
                    <visualize>true</visualize>
                </sensor>
            </gazebo>
        </robot>
        "#;

        let extensions = GazeboExtensionParser::parse(urdf_xml).unwrap();
        assert_eq!(extensions.sensors.len(), 1);

        let sensor = &extensions.sensors[0];
        assert_eq!(sensor.name, "laser");
        assert_eq!(sensor.sensor_type, SensorType::Ray);
        assert_eq!(sensor.link_name, "laser_link");
        assert_eq!(sensor.update_rate, 10.0);
        assert!(sensor.always_on);
        assert!(sensor.visualize);
    }

    #[test]
    fn test_parse_gazebo_plugin() {
        let urdf_xml = r#"
        <robot name="test">
            <gazebo>
                <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
                    <left_joint>left_wheel_joint</left_joint>
                    <right_joint>right_wheel_joint</right_joint>
                </plugin>
            </gazebo>
        </robot>
        "#;

        let extensions = GazeboExtensionParser::parse(urdf_xml).unwrap();
        assert_eq!(extensions.plugins.len(), 1);

        let plugin = &extensions.plugins[0];
        assert_eq!(plugin.name, "diff_drive");
        assert_eq!(plugin.filename, "libgazebo_ros_diff_drive.so");
        assert_eq!(
            plugin.parameters.get("left_joint"),
            Some(&"left_wheel_joint".to_string())
        );
        assert_eq!(
            plugin.parameters.get("right_joint"),
            Some(&"right_wheel_joint".to_string())
        );
    }

    #[test]
    fn test_parse_diff_drive_config() {
        let urdf_xml = r#"
        <robot name="test">
            <gazebo>
                <plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
                    <left_joint>left_wheel_joint</left_joint>
                    <right_joint>right_wheel_joint</right_joint>
                    <wheel_separation>0.17</wheel_separation>
                    <wheel_diameter>0.08</wheel_diameter>
                    <max_wheel_torque>5.0</max_wheel_torque>
                    <max_wheel_acceleration>2.0</max_wheel_acceleration>
                    <command_topic>cmd_vel</command_topic>
                    <odometry_topic>odom</odometry_topic>
                </plugin>
            </gazebo>
        </robot>
        "#;

        let extensions = GazeboExtensionParser::parse(urdf_xml).unwrap();

        // Should detect diff drive plugin
        assert!(extensions.diff_drive.is_some());

        let diff_drive = extensions.diff_drive.unwrap();
        assert_eq!(diff_drive.left_joint, "left_wheel_joint");
        assert_eq!(diff_drive.right_joint, "right_wheel_joint");
        assert_eq!(diff_drive.wheel_separation, 0.17);
        assert_eq!(diff_drive.wheel_diameter, 0.08);
        assert_eq!(diff_drive.max_wheel_torque, 5.0);
        assert_eq!(diff_drive.command_topic, "cmd_vel");
    }

    #[test]
    fn test_diff_drive_detection_various_names() {
        // Test various plugin naming conventions
        let test_cases = [
            ("diff_drive", "libgazebo_ros_diff_drive.so"),
            ("my_controller", "libdiff_drive_controller.so"),
            ("differential_drive_controller", "some_plugin.so"),
        ];

        for (name, filename) in test_cases {
            let plugin = GazeboPlugin {
                name: name.to_string(),
                filename: filename.to_string(),
                parameters: HashMap::new(),
            };
            assert!(
                GazeboExtensionParser::is_diff_drive_plugin(&plugin),
                "Should detect diff drive for name='{}', filename='{}'",
                name,
                filename
            );
        }
    }
}

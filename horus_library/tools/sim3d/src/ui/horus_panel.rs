//! HORUS Communication Panel
//!
//! Provides runtime controls for HORUS topic configuration, connection status,
//! and robot communication settings. Like RViz2's topic selector panel.

use bevy::prelude::*;
use bevy_egui::egui;

use crate::horus_native::{HorusComm, HorusTopicConfig};
use crate::systems::horus_sync::{HorusSyncConfig, HorusSyncStats};
use crate::systems::topic_discovery::{
    LatestMessage, MessageType, TopicScanner, TopicSubscriptions,
};

/// Which tab is active in the HORUS panel
#[derive(Clone, Copy, PartialEq, Default)]
pub enum HorusPanelTab {
    #[default]
    Robots,
    AllTopics,
}

/// Panel configuration resource
#[derive(Resource)]
pub struct HorusPanelConfig {
    /// Show the panel
    pub visible: bool,
    /// Expand all robots in tree view
    pub expand_all: bool,
    /// Show statistics
    pub show_stats: bool,
    /// Show advanced options
    pub show_advanced: bool,
    /// New robot name input
    pub new_robot_name: String,
    /// Active tab
    pub active_tab: HorusPanelTab,
    /// Show only active topics
    pub show_active_only: bool,
}

impl Default for HorusPanelConfig {
    fn default() -> Self {
        Self {
            visible: true,
            expand_all: true,
            show_stats: true,
            show_advanced: false,
            new_robot_name: String::new(),
            active_tab: HorusPanelTab::default(),
            show_active_only: false,
        }
    }
}

/// Topic status for display
#[derive(Clone, Copy, PartialEq)]
pub enum TopicStatus {
    Connected,
    Disconnected,
    Error,
}

impl TopicStatus {
    pub fn color(&self) -> egui::Color32 {
        match self {
            TopicStatus::Connected => egui::Color32::from_rgb(100, 200, 100),
            TopicStatus::Disconnected => egui::Color32::from_rgb(150, 150, 150),
            TopicStatus::Error => egui::Color32::from_rgb(200, 100, 100),
        }
    }

    pub fn icon(&self) -> &'static str {
        match self {
            TopicStatus::Connected => "●",
            TopicStatus::Disconnected => "○",
            TopicStatus::Error => "✗",
        }
    }
}

/// Render the HORUS panel UI
pub fn render_horus_panel_ui(
    ui: &mut egui::Ui,
    panel_config: &mut HorusPanelConfig,
    horus_comm: &HorusComm,
    sync_config: &mut HorusSyncConfig,
    sync_stats: &HorusSyncStats,
    topic_scanner: Option<&TopicScanner>,
    subscriptions: Option<&mut TopicSubscriptions>,
) {
    ui.heading("HORUS Communication");
    ui.separator();

    // Tab bar
    ui.horizontal(|ui| {
        ui.selectable_value(
            &mut panel_config.active_tab,
            HorusPanelTab::Robots,
            "Robots",
        );
        ui.selectable_value(
            &mut panel_config.active_tab,
            HorusPanelTab::AllTopics,
            "All Topics",
        );
    });
    ui.separator();

    match panel_config.active_tab {
        HorusPanelTab::Robots => {
            render_robots_tab(ui, panel_config, horus_comm, sync_config, sync_stats);
        }
        HorusPanelTab::AllTopics => {
            render_all_topics_tab(ui, panel_config, topic_scanner, subscriptions);
        }
    }
}

/// Render the Robots tab content
fn render_robots_tab(
    ui: &mut egui::Ui,
    panel_config: &mut HorusPanelConfig,
    horus_comm: &HorusComm,
    sync_config: &mut HorusSyncConfig,
    sync_stats: &HorusSyncStats,
) {
    // Global sync controls
    ui.horizontal(|ui| {
        ui.checkbox(&mut sync_config.enabled, "Enabled");
        ui.label("Rate:");
        ui.add(
            egui::DragValue::new(&mut sync_config.publish_rate)
                .range(0.0..=1000.0)
                .speed(1.0)
                .suffix(" Hz"),
        );
    });

    ui.separator();

    // Statistics
    if panel_config.show_stats {
        egui::CollapsingHeader::new("Statistics")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Published:");
                    ui.label(format!("{}", sync_stats.messages_published));
                    ui.label("Received:");
                    ui.label(format!("{}", sync_stats.messages_received));
                });
                if sync_stats.publish_errors > 0 {
                    ui.colored_label(
                        egui::Color32::from_rgb(200, 100, 100),
                        format!("Errors: {}", sync_stats.publish_errors),
                    );
                }
            });
        ui.separator();
    }

    // Robot list
    ui.label("Connected Robots:");

    if horus_comm.robot_hubs.is_empty() {
        ui.label("No robots connected");
    } else {
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .max_height(300.0)
            .show(ui, |ui| {
                for (robot_name, hubs) in &horus_comm.robot_hubs {
                    let header = format!("🤖 {}", robot_name);

                    egui::CollapsingHeader::new(header)
                        .default_open(panel_config.expand_all)
                        .show(ui, |ui| {
                            // cmd_vel (subscribe)
                            render_topic_row(
                                ui,
                                "cmd_vel",
                                "SUB",
                                hubs.cmd_vel_sub.is_some(),
                                &format!("{}.cmd_vel", robot_name),
                            );

                            // odom (publish)
                            render_topic_row(
                                ui,
                                "odom",
                                "PUB",
                                hubs.odom_pub.is_some(),
                                &format!("{}.odom", robot_name),
                            );

                            // scan (publish)
                            render_topic_row(
                                ui,
                                "scan",
                                "PUB",
                                hubs.scan_pub.is_some(),
                                &format!("{}.scan", robot_name),
                            );

                            // imu (publish)
                            render_topic_row(
                                ui,
                                "imu",
                                "PUB",
                                hubs.imu_pub.is_some(),
                                &format!("{}.imu", robot_name),
                            );

                            // joint_cmd (subscribe)
                            render_topic_row(
                                ui,
                                "joint_cmd",
                                "SUB",
                                hubs.joint_cmd_sub.is_some(),
                                &format!("{}.joint_cmd", robot_name),
                            );

                            // joint_states (publish)
                            render_topic_row(
                                ui,
                                "joint_states",
                                "PUB",
                                hubs.joint_state_pub.is_some(),
                                &format!("{}.joint_states", robot_name),
                            );
                        });
                }
            });
    }

    ui.separator();

    // Options
    ui.horizontal(|ui| {
        ui.checkbox(&mut panel_config.show_stats, "Stats");
        ui.checkbox(&mut panel_config.expand_all, "Expand All");
        ui.checkbox(&mut panel_config.show_advanced, "Advanced");
    });

    // Advanced options
    if panel_config.show_advanced {
        ui.separator();
        egui::CollapsingHeader::new("Advanced")
            .default_open(false)
            .show(ui, |ui| {
                ui.label("Topic Configuration Presets:");
                ui.horizontal(|ui| {
                    if ui.button("Diff Drive").clicked() {
                        let _ = HorusTopicConfig::diff_drive();
                        // Would need mutable access to reconfigure
                    }
                    if ui.button("Articulated").clicked() {
                        let _ = HorusTopicConfig::articulated();
                    }
                    if ui.button("All").clicked() {
                        let _ = HorusTopicConfig::all();
                    }
                });

                ui.separator();
                ui.label("Add Robot:");
                ui.horizontal(|ui| {
                    ui.text_edit_singleline(&mut panel_config.new_robot_name);
                    if ui.button("Add").clicked() && !panel_config.new_robot_name.is_empty() {
                        // Would need mutable HorusComm to add robot
                        panel_config.new_robot_name.clear();
                    }
                });
            });
    }
}

/// Render the All Topics tab showing discovered topics
fn render_all_topics_tab(
    ui: &mut egui::Ui,
    panel_config: &mut HorusPanelConfig,
    topic_scanner: Option<&TopicScanner>,
    subscriptions: Option<&mut TopicSubscriptions>,
) {
    ui.horizontal(|ui| {
        ui.checkbox(&mut panel_config.show_active_only, "Active only");
        ui.label(format!(
            "{} topics",
            topic_scanner.map(|s| s.topics.len()).unwrap_or(0)
        ));
        if let Some(subs) = &subscriptions {
            if !subs.subscriptions.is_empty() {
                ui.label(format!("({} subscribed)", subs.subscriptions.len()));
            }
        }
    });
    ui.separator();

    let Some(scanner) = topic_scanner else {
        ui.label("Topic scanner not initialized");
        return;
    };

    if scanner.topics.is_empty() {
        ui.label("No topics discovered");
        ui.weak("Topics appear in /dev/shm/horus/topics/");
        return;
    }

    // Collect actions to apply after iteration
    let mut subscribe_topic: Option<(String, MessageType)> = None;
    let mut unsubscribe_topic: Option<String> = None;

    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .max_height(300.0)
        .show(ui, |ui| {
            let topics = if panel_config.show_active_only {
                scanner.active_topics()
            } else {
                scanner.sorted_topics()
            };

            for topic in topics {
                let is_subscribed = subscriptions
                    .as_ref()
                    .map(|s| s.is_subscribed(&topic.name))
                    .unwrap_or(false);

                ui.horizontal(|ui| {
                    // Status indicator
                    let (icon, color) = if topic.is_active {
                        ("●", egui::Color32::from_rgb(100, 200, 100))
                    } else {
                        ("○", egui::Color32::from_rgb(150, 150, 150))
                    };
                    ui.colored_label(color, icon);

                    // Message type badge
                    let type_color = match topic.message_type {
                        MessageType::Twist => egui::Color32::from_rgb(100, 150, 255),
                        MessageType::Odometry => egui::Color32::from_rgb(150, 200, 100),
                        MessageType::Imu => egui::Color32::from_rgb(255, 180, 100),
                        MessageType::LaserScan => egui::Color32::from_rgb(255, 100, 150),
                        MessageType::JointCommand => egui::Color32::from_rgb(200, 150, 255),
                        MessageType::Unknown => egui::Color32::from_rgb(150, 150, 150),
                    };
                    ui.colored_label(
                        type_color,
                        format!("[{}]", topic.message_type.display_name()),
                    );

                    // Topic name
                    ui.label(&topic.name);

                    // Subscribe/Unsubscribe button
                    if topic.message_type != MessageType::Unknown {
                        if is_subscribed {
                            if ui.small_button("✖").on_hover_text("Unsubscribe").clicked() {
                                unsubscribe_topic = Some(topic.name.clone());
                            }
                        } else if ui.small_button("👁").on_hover_text("Subscribe").clicked() {
                            subscribe_topic = Some((topic.name.clone(), topic.message_type));
                        }
                    }

                    // Size and age on the right
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if let Some(modified) = topic.last_modified {
                            if let Ok(elapsed) = modified.elapsed() {
                                ui.weak(format_duration(elapsed));
                            }
                        }
                        ui.weak(format_bytes(topic.size_bytes));
                    });
                });
            }
        });

    // Apply subscription changes and show subscribed topic data
    if let Some(subs) = subscriptions {
        if let Some((topic_name, msg_type)) = subscribe_topic {
            subs.subscribe(&topic_name, msg_type);
        }
        if let Some(topic_name) = unsubscribe_topic {
            subs.unsubscribe(&topic_name);
        }

        // Show subscribed topic data
        if !subs.subscriptions.is_empty() {
            ui.separator();
            ui.label("Subscribed Topics:");

            egui::ScrollArea::vertical()
                .id_salt("subscribed_topics")
                .auto_shrink([false, false])
                .max_height(200.0)
                .show(ui, |ui| {
                    for (topic_name, sub) in &subs.subscriptions {
                        egui::CollapsingHeader::new(topic_name)
                            .default_open(true)
                            .show(ui, |ui| {
                                render_message_data(ui, sub.latest.as_ref());
                            });
                    }
                });
        }
    }
}

/// Render message data visualization
fn render_message_data(ui: &mut egui::Ui, latest: Option<&LatestMessage>) {
    let Some(msg) = latest else {
        ui.weak("Waiting for data...");
        return;
    };

    match msg {
        LatestMessage::Twist(twist) => {
            ui.horizontal(|ui| {
                ui.label("Linear:");
                ui.monospace(format!(
                    "[{:.3}, {:.3}, {:.3}]",
                    twist.linear[0], twist.linear[1], twist.linear[2]
                ));
            });
            ui.horizontal(|ui| {
                ui.label("Angular:");
                ui.monospace(format!(
                    "[{:.3}, {:.3}, {:.3}]",
                    twist.angular[0], twist.angular[1], twist.angular[2]
                ));
            });
        }
        LatestMessage::Odometry(odom) => {
            ui.horizontal(|ui| {
                ui.label("Position:");
                ui.monospace(format!(
                    "x={:.3}, y={:.3}, θ={:.3}",
                    odom.pose.x, odom.pose.y, odom.pose.theta
                ));
            });
            ui.horizontal(|ui| {
                ui.label("Velocity:");
                ui.monospace(format!(
                    "lin=[{:.3}, {:.3}], ang={:.3}",
                    odom.twist.linear[0], odom.twist.linear[1], odom.twist.angular[2]
                ));
            });
        }
        LatestMessage::Imu(imu) => {
            ui.horizontal(|ui| {
                ui.label("Orientation:");
                ui.monospace(format!(
                    "[{:.3}, {:.3}, {:.3}, {:.3}]",
                    imu.orientation[0], imu.orientation[1], imu.orientation[2], imu.orientation[3]
                ));
            });
            ui.horizontal(|ui| {
                ui.label("Angular Vel:");
                ui.monospace(format!(
                    "[{:.3}, {:.3}, {:.3}]",
                    imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]
                ));
            });
            ui.horizontal(|ui| {
                ui.label("Linear Accel:");
                ui.monospace(format!(
                    "[{:.3}, {:.3}, {:.3}]",
                    imu.linear_acceleration[0],
                    imu.linear_acceleration[1],
                    imu.linear_acceleration[2]
                ));
            });
        }
        LatestMessage::LaserScan(scan) => {
            ui.horizontal(|ui| {
                ui.label("Ranges:");
                ui.monospace(format!("{} points", scan.ranges.len()));
            });
            ui.horizontal(|ui| {
                ui.label("Angle:");
                ui.monospace(format!(
                    "{:.2}° to {:.2}°",
                    scan.angle_min.to_degrees(),
                    scan.angle_max.to_degrees()
                ));
            });
            // Show min/max range values
            if !scan.ranges.is_empty() {
                let valid_ranges: Vec<f32> = scan
                    .ranges
                    .iter()
                    .filter(|&&r| r.is_finite() && r > 0.0)
                    .cloned()
                    .collect();
                if !valid_ranges.is_empty() {
                    let min_range = valid_ranges.iter().cloned().fold(f32::INFINITY, f32::min);
                    let max_range = valid_ranges.iter().cloned().fold(0.0_f32, f32::max);
                    ui.horizontal(|ui| {
                        ui.label("Range:");
                        ui.monospace(format!("{:.2}m - {:.2}m", min_range, max_range));
                    });
                }
            }
        }
        LatestMessage::JointCommand(joint) => {
            ui.horizontal(|ui| {
                ui.label("Joints:");
                ui.monospace(format!("{}", joint.joint_count));
            });
            for i in 0..(joint.joint_count as usize).min(8) {
                ui.horizontal(|ui| {
                    ui.label(format!("  [{}]:", i));
                    ui.monospace(format!(
                        "pos={:.3}, vel={:.3}",
                        joint.positions[i], joint.velocities[i]
                    ));
                });
            }
        }
    }
}

/// Format bytes to human readable string
fn format_bytes(bytes: u64) -> String {
    if bytes < 1024 {
        format!("{}B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1}KB", bytes as f64 / 1024.0)
    } else {
        format!("{:.1}MB", bytes as f64 / (1024.0 * 1024.0))
    }
}

/// Format duration to human readable string
fn format_duration(d: std::time::Duration) -> String {
    let secs = d.as_secs();
    if secs < 60 {
        format!("{}s ago", secs)
    } else if secs < 3600 {
        format!("{}m ago", secs / 60)
    } else {
        format!("{}h ago", secs / 3600)
    }
}

/// Render a single topic row with status indicator
fn render_topic_row(
    ui: &mut egui::Ui,
    name: &str,
    direction: &str,
    connected: bool,
    topic_path: &str,
) {
    let status = if connected {
        TopicStatus::Connected
    } else {
        TopicStatus::Disconnected
    };

    ui.horizontal(|ui| {
        ui.colored_label(status.color(), status.icon());
        ui.label(format!("[{}]", direction));
        ui.label(name);
        if connected {
            ui.weak(topic_path);
        }
    });
}

/// System to render the HORUS panel in the dock
pub fn horus_panel_system(
    mut contexts: bevy_egui::EguiContexts,
    mut panel_config: ResMut<HorusPanelConfig>,
    horus_comm: Option<Res<HorusComm>>,
    mut sync_config: ResMut<HorusSyncConfig>,
    sync_stats: Res<HorusSyncStats>,
    topic_scanner: Option<Res<TopicScanner>>,
    mut subscriptions: Option<ResMut<TopicSubscriptions>>,
) {
    if !panel_config.visible {
        return;
    }

    let Some(horus_comm) = horus_comm else {
        return;
    };

    egui::Window::new("HORUS")
        .default_width(350.0)
        .resizable(true)
        .show(contexts.ctx_mut(), |ui| {
            render_horus_panel_ui(
                ui,
                &mut panel_config,
                &horus_comm,
                &mut sync_config,
                &sync_stats,
                topic_scanner.as_deref(),
                subscriptions.as_deref_mut(),
            );
        });
}

/// HORUS panel plugin
pub struct HorusPanelPlugin;

impl Plugin for HorusPanelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HorusPanelConfig>();

        // Add the panel system after EguiContexts are ready
        use bevy_egui::EguiSet;
        app.add_systems(Update, horus_panel_system.after(EguiSet::InitContexts));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic_status() {
        assert_eq!(TopicStatus::Connected.icon(), "●");
        assert_eq!(TopicStatus::Disconnected.icon(), "○");
        assert_eq!(TopicStatus::Error.icon(), "✗");
    }

    #[test]
    fn test_panel_config_default() {
        let config = HorusPanelConfig::default();
        assert!(config.visible);
        assert!(config.expand_all);
        assert!(config.show_stats);
    }
}

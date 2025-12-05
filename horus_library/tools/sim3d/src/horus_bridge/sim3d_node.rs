//! Sim3D HORUS Node - Makes sim3d a proper HORUS Node
//!
//! This module provides a HORUS Node wrapper around the sim3d simulator,
//! allowing it to be registered with the HORUS Scheduler and participate
//! in the HORUS runtime system.

use horus_core::core::{Node, NodeConfig, NodeInfo, TopicMetadata};
use horus_core::error::HorusResult;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Sim3D HORUS Node - wraps the simulator as a HORUS node
///
/// This node represents the entire sim3d simulation as a single HORUS node.
/// It publishes sensor data and subscribes to command topics.
///
/// # Topics Published
/// - `{robot_name}.odom` - Odometry data
/// - `{robot_name}.hf` - Transform frames (HFrame)
/// - `{robot_name}.scan` - LaserScan data
/// - `{robot_name}.pointcloud` - PointCloud2 data
/// - `{robot_name}.joint_states` - JointState data
///
/// # Topics Subscribed
/// - `{robot_name}.cmd_vel` - Velocity commands (Twist)
pub struct Sim3dNode {
    /// Robot name for topic naming
    robot_name: String,
    /// Simulation tick rate (Hz)
    tick_rate: f32,
    /// Last tick time
    last_tick: Instant,
    /// Running flag
    running: bool,
    /// Shared state for communication with Bevy app
    /// (The actual Bevy app runs in its own thread/loop)
    state: Arc<Mutex<Sim3dNodeState>>,
}

/// Shared state between HORUS Node and Bevy simulation
#[derive(Default)]
pub struct Sim3dNodeState {
    /// Number of ticks executed
    pub tick_count: u64,
    /// Is the simulation running
    pub is_running: bool,
    /// Last known robot position
    pub robot_position: [f32; 3],
    /// Last known robot orientation (quaternion)
    pub robot_orientation: [f32; 4],
    /// Pending command velocity
    pub pending_cmd_vel: Option<(f32, f32, f32, f32, f32, f32)>, // linear xyz, angular xyz
    /// Messages published this tick
    pub messages_published: u32,
    /// Messages received this tick
    pub messages_received: u32,
}

impl Default for Sim3dNode {
    fn default() -> Self {
        Self::new("sim3d_robot")
    }
}

impl Sim3dNode {
    /// Create a new Sim3D node
    pub fn new(robot_name: impl Into<String>) -> Self {
        Self {
            robot_name: robot_name.into(),
            tick_rate: 60.0, // Default 60 Hz
            last_tick: Instant::now(),
            running: false,
            state: Arc::new(Mutex::new(Sim3dNodeState::default())),
        }
    }

    /// Create a new Sim3D node with custom tick rate
    pub fn with_tick_rate(mut self, rate: f32) -> Self {
        self.tick_rate = rate;
        self
    }

    /// Get shared state handle for Bevy integration
    pub fn state(&self) -> Arc<Mutex<Sim3dNodeState>> {
        self.state.clone()
    }

    /// Get the robot name
    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }

    /// Get the tick rate
    pub fn tick_rate(&self) -> f32 {
        self.tick_rate
    }

    /// Get the tick period
    pub fn tick_period(&self) -> Duration {
        Duration::from_secs_f32(1.0 / self.tick_rate)
    }

    /// Check if it's time for the next tick
    pub fn should_tick(&self) -> bool {
        self.last_tick.elapsed() >= self.tick_period()
    }

    /// Update tick timing
    fn update_tick_timing(&mut self) {
        self.last_tick = Instant::now();
    }
}

impl Node for Sim3dNode {
    fn name(&self) -> &'static str {
        "sim3d"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> HorusResult<()> {
        ctx.log_info(&format!(
            "Sim3D node initializing for robot: {}",
            self.robot_name
        ));
        ctx.log_info(&format!("Tick rate: {} Hz", self.tick_rate));

        // Mark as running
        self.running = true;
        if let Ok(mut state) = self.state.lock() {
            state.is_running = true;
        }

        ctx.log_info("Sim3D node initialized successfully");
        Ok(())
    }

    fn tick(&mut self, ctx: Option<&mut NodeInfo>) {
        if !self.running {
            return;
        }

        // Rate limiting - only tick at configured rate
        if !self.should_tick() {
            return;
        }

        self.update_tick_timing();

        // Update state
        if let Ok(mut state) = self.state.lock() {
            state.tick_count += 1;
            state.messages_published = 0;
            state.messages_received = 0;
        }

        // Log periodic status
        if let Some(ctx) = ctx {
            if let Ok(state) = self.state.lock() {
                if state.tick_count % 600 == 0 {
                    // Every ~10 seconds at 60Hz
                    ctx.log_info(&format!(
                        "Sim3D tick #{}: pos=[{:.2},{:.2},{:.2}]",
                        state.tick_count,
                        state.robot_position[0],
                        state.robot_position[1],
                        state.robot_position[2]
                    ));
                }
            }
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> HorusResult<()> {
        ctx.log_info("Sim3D node shutting down...");

        self.running = false;
        if let Ok(mut state) = self.state.lock() {
            state.is_running = false;
            ctx.log_info(&format!(
                "Sim3D shutdown complete. Total ticks: {}",
                state.tick_count
            ));
        }

        Ok(())
    }

    fn get_publishers(&self) -> Vec<TopicMetadata> {
        vec![
            TopicMetadata {
                topic_name: format!("{}.odom", self.robot_name),
                type_name: "Odometry".to_string(),
            },
            TopicMetadata {
                topic_name: format!("{}.hf", self.robot_name),
                type_name: "TransformStamped".to_string(),
            },
            TopicMetadata {
                topic_name: format!("{}.scan", self.robot_name),
                type_name: "LaserScan".to_string(),
            },
            TopicMetadata {
                topic_name: format!("{}.pointcloud", self.robot_name),
                type_name: "PointCloud2".to_string(),
            },
            TopicMetadata {
                topic_name: format!("{}.joint_states", self.robot_name),
                type_name: "JointState".to_string(),
            },
        ]
    }

    fn get_subscribers(&self) -> Vec<TopicMetadata> {
        vec![TopicMetadata {
            topic_name: format!("{}.cmd_vel", self.robot_name),
            type_name: "Twist".to_string(),
        }]
    }

    fn priority(&self) -> u32 {
        // Lower priority than control nodes, but higher than logging
        60
    }

    fn get_config(&self) -> NodeConfig {
        let tick_period_ms = (1000.0 / self.tick_rate) as u64;
        NodeConfig {
            max_tick_duration_ms: Some(tick_period_ms * 2), // 2x tick period
            restart_on_failure: true,
            max_restart_attempts: 3,
            restart_delay_ms: 1000,
            enable_logging: true,
            log_level: "INFO".to_string(),
            custom_params: std::collections::HashMap::new(),
        }
    }

    fn is_healthy(&self) -> bool {
        self.running
    }
}

/// Bevy resource to hold the HORUS node state
/// This allows Bevy systems to communicate with the HORUS Node
#[derive(bevy::prelude::Resource)]
pub struct Sim3dNodeHandle {
    state: Arc<Mutex<Sim3dNodeState>>,
    robot_name: String,
}

impl Sim3dNodeHandle {
    /// Create a new handle from a Sim3dNode
    pub fn from_node(node: &Sim3dNode) -> Self {
        Self {
            state: node.state(),
            robot_name: node.robot_name().to_string(),
        }
    }

    /// Create a new handle with robot name
    pub fn new(robot_name: impl Into<String>) -> Self {
        Self {
            state: Arc::new(Mutex::new(Sim3dNodeState::default())),
            robot_name: robot_name.into(),
        }
    }

    /// Get the shared state
    pub fn state(&self) -> Arc<Mutex<Sim3dNodeState>> {
        self.state.clone()
    }

    /// Get robot name
    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }

    /// Update robot position from Bevy simulation
    pub fn update_position(&self, x: f32, y: f32, z: f32) {
        if let Ok(mut state) = self.state.lock() {
            state.robot_position = [x, y, z];
        }
    }

    /// Update robot orientation from Bevy simulation
    pub fn update_orientation(&self, x: f32, y: f32, z: f32, w: f32) {
        if let Ok(mut state) = self.state.lock() {
            state.robot_orientation = [x, y, z, w];
        }
    }

    /// Get pending command velocity
    pub fn take_pending_cmd_vel(&self) -> Option<(f32, f32, f32, f32, f32, f32)> {
        if let Ok(mut state) = self.state.lock() {
            state.pending_cmd_vel.take()
        } else {
            None
        }
    }

    /// Record a published message
    pub fn record_publish(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.messages_published += 1;
        }
    }

    /// Record a received message
    pub fn record_receive(&self) {
        if let Ok(mut state) = self.state.lock() {
            state.messages_received += 1;
        }
    }
}

/// Bevy plugin to integrate Sim3dNode state with Bevy systems
pub struct Sim3dNodePlugin {
    robot_name: String,
}

impl Default for Sim3dNodePlugin {
    fn default() -> Self {
        Self {
            robot_name: "sim3d_robot".to_string(),
        }
    }
}

impl Sim3dNodePlugin {
    /// Create plugin with custom robot name
    pub fn with_robot_name(robot_name: impl Into<String>) -> Self {
        Self {
            robot_name: robot_name.into(),
        }
    }
}

impl bevy::prelude::Plugin for Sim3dNodePlugin {
    fn build(&self, app: &mut bevy::prelude::App) {
        let handle = Sim3dNodeHandle::new(&self.robot_name);
        app.insert_resource(handle);
        app.add_systems(bevy::prelude::Update, sync_node_state_system);

        tracing::info!("Sim3D Node plugin loaded for robot: {}", self.robot_name);
    }
}

/// System to sync Bevy state with HORUS node state
fn sync_node_state_system(
    handle: bevy::prelude::Res<Sim3dNodeHandle>,
    query: bevy::prelude::Query<
        &bevy::prelude::GlobalTransform,
        bevy::prelude::With<crate::robot::robot::Robot>,
    >,
) {
    // Update position from first robot found
    if let Some(transform) = query.iter().next() {
        let translation = transform.translation();
        handle.update_position(translation.x, translation.y, translation.z);

        let (_, rotation, _) = transform.to_scale_rotation_translation();
        handle.update_orientation(rotation.x, rotation.y, rotation.z, rotation.w);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sim3d_node_creation() {
        let node = Sim3dNode::new("test_robot");
        assert_eq!(node.name(), "sim3d");
        assert_eq!(node.robot_name(), "test_robot");
        assert_eq!(node.tick_rate(), 60.0);
    }

    #[test]
    fn test_sim3d_node_with_tick_rate() {
        let node = Sim3dNode::new("test_robot").with_tick_rate(30.0);
        assert_eq!(node.tick_rate(), 30.0);
    }

    #[test]
    fn test_sim3d_node_publishers() {
        let node = Sim3dNode::new("robot1");
        let publishers = node.get_publishers();
        assert_eq!(publishers.len(), 5);
        assert!(publishers.iter().any(|p| p.topic_name == "robot1.odom"));
        assert!(publishers.iter().any(|p| p.topic_name == "robot1.hf"));
        assert!(publishers.iter().any(|p| p.topic_name == "robot1.scan"));
    }

    #[test]
    fn test_sim3d_node_subscribers() {
        let node = Sim3dNode::new("robot1");
        let subscribers = node.get_subscribers();
        assert_eq!(subscribers.len(), 1);
        assert_eq!(subscribers[0].topic_name, "robot1.cmd_vel");
    }

    #[test]
    fn test_sim3d_node_handle() {
        let node = Sim3dNode::new("test_robot");
        let handle = Sim3dNodeHandle::from_node(&node);

        handle.update_position(1.0, 2.0, 3.0);

        if let Ok(state) = handle.state().lock() {
            assert_eq!(state.robot_position, [1.0, 2.0, 3.0]);
        }
    }

    #[test]
    fn test_sim3d_node_config() {
        let node = Sim3dNode::new("test_robot").with_tick_rate(100.0);
        let config = node.get_config();
        assert_eq!(config.max_tick_duration_ms, Some(20)); // 2x of 10ms tick period
        assert!(config.enable_logging);
    }
}

// Type-based Hub implementation for Python bindings
//
// New API matches Rust exactly:
//   from horus import Hub, CmdVel, Pose2D
//   hub = Hub(CmdVel)  # Type determines everything
//
// Network support (NEW):
//   hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
//   hub = Hub(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
//   hub = Hub(CmdVel, endpoint="cmdvel@router")            # Via router
//   hub = Hub(CmdVel, endpoint="cmdvel@*")                 # Multicast

use horus::communication::hub::Hub;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::geometry::Pose2D;
use horus_library::messages::sensor::{Imu, LaserScan, Odometry};
use horus_library::messages::GenericMessage;
use pyo3::prelude::*;
use pyo3::sync::GILOnceCell;
use pyo3::types::{PyDict, PyType};
use serde::{de::DeserializeOwned, Serialize};
use std::sync::{Arc, RwLock};

// ============================================================================
// P0 OPTIMIZATION: Cached Python classes (100x faster than py.import() per call)
// ============================================================================

static CMDVEL_CLASS: GILOnceCell<Py<PyType>> = GILOnceCell::new();
static POSE2D_CLASS: GILOnceCell<Py<PyType>> = GILOnceCell::new();
static IMU_CLASS: GILOnceCell<Py<PyType>> = GILOnceCell::new();
static ODOMETRY_CLASS: GILOnceCell<Py<PyType>> = GILOnceCell::new();
static LASERSCAN_CLASS: GILOnceCell<Py<PyType>> = GILOnceCell::new();

fn get_cmdvel_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    CMDVEL_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("CmdVel")?.downcast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_pose2d_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    POSE2D_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("Pose2D")?.downcast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_imu_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    IMU_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("Imu")?.downcast::<PyType>()?.clone().unbind())
        })
        .map(|c| c.bind(py))
}

fn get_odometry_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    ODOMETRY_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("Odometry")?
                .downcast::<PyType>()?
                .clone()
                .unbind())
        })
        .map(|c| c.bind(py))
}

fn get_laserscan_class(py: Python<'_>) -> PyResult<&Bound<'_, PyType>> {
    LASERSCAN_CLASS
        .get_or_try_init(py, || {
            let m = py.import("horus")?;
            Ok(m.getattr("LaserScan")?
                .downcast::<PyType>()?
                .clone()
                .unbind())
        })
        .map(|c| c.bind(py))
}

// ============================================================================
// P1 OPTIMIZATION: Direct field extraction (bypasses serde, 2-5x faster)
// ============================================================================

/// Extract CmdVel directly from Python object (no serde overhead)
fn extract_cmdvel(py: Python<'_>, obj: &PyObject) -> PyResult<CmdVel> {
    let linear: f32 = obj.getattr(py, "linear")?.extract(py)?;
    let angular: f32 = obj.getattr(py, "angular")?.extract(py)?;
    let timestamp: u64 = obj.getattr(py, "timestamp")?.extract(py).unwrap_or(0);
    Ok(CmdVel::with_timestamp(linear, angular, timestamp))
}

/// Extract Pose2D directly from Python object (no serde overhead)
fn extract_pose2d(py: Python<'_>, obj: &PyObject) -> PyResult<Pose2D> {
    let x: f64 = obj.getattr(py, "x")?.extract(py)?;
    let y: f64 = obj.getattr(py, "y")?.extract(py)?;
    let theta: f64 = obj.getattr(py, "theta")?.extract(py)?;
    let timestamp: u64 = obj.getattr(py, "timestamp")?.extract(py).unwrap_or(0);
    Ok(Pose2D {
        x,
        y,
        theta,
        timestamp,
    })
}

/// Extract Imu directly from Python object (no serde overhead)
fn extract_imu(py: Python<'_>, obj: &PyObject) -> PyResult<Imu> {
    let ax: f64 = obj.getattr(py, "accel_x")?.extract(py)?;
    let ay: f64 = obj.getattr(py, "accel_y")?.extract(py)?;
    let az: f64 = obj.getattr(py, "accel_z")?.extract(py)?;
    let gx: f64 = obj.getattr(py, "gyro_x")?.extract(py)?;
    let gy: f64 = obj.getattr(py, "gyro_y")?.extract(py)?;
    let gz: f64 = obj.getattr(py, "gyro_z")?.extract(py)?;
    let timestamp: u64 = obj.getattr(py, "timestamp")?.extract(py).unwrap_or(0);
    let mut imu = Imu::new();
    imu.linear_acceleration = [ax, ay, az];
    imu.angular_velocity = [gx, gy, gz];
    imu.timestamp = timestamp;
    Ok(imu)
}

/// Create Python CmdVel directly (no serde overhead)
fn cmdvel_to_python(py: Python<'_>, cmd: &CmdVel) -> PyResult<PyObject> {
    let cls = get_cmdvel_class(py)?;
    Ok(cls
        .call1((cmd.linear, cmd.angular, cmd.stamp_nanos))?
        .into())
}

/// Create Python Pose2D directly (no serde overhead)
fn pose2d_to_python(py: Python<'_>, pose: &Pose2D) -> PyResult<PyObject> {
    let cls = get_pose2d_class(py)?;
    Ok(cls
        .call1((pose.x, pose.y, pose.theta, pose.timestamp))?
        .into())
}

/// Create Python Imu directly (no serde overhead)
fn imu_to_python(py: Python<'_>, imu: &Imu) -> PyResult<PyObject> {
    let cls = get_imu_class(py)?;
    Ok(cls
        .call1((
            imu.linear_acceleration[0],
            imu.linear_acceleration[1],
            imu.linear_acceleration[2],
            imu.angular_velocity[0],
            imu.angular_velocity[1],
            imu.angular_velocity[2],
            imu.timestamp,
        ))?
        .into())
}

/// Create Python Odometry directly using dict (faster than serde for complex types)
fn odometry_to_python(py: Python<'_>, odom: &Odometry) -> PyResult<PyObject> {
    let cls = get_odometry_class(py)?;
    let dict = PyDict::new(py);
    dict.set_item("x", odom.pose.x)?;
    dict.set_item("y", odom.pose.y)?;
    dict.set_item("theta", odom.pose.theta)?;
    dict.set_item("linear_velocity", odom.twist.linear[0])?; // Forward velocity
    dict.set_item("angular_velocity", odom.twist.angular[2])?; // Yaw rate
    dict.set_item("timestamp", odom.timestamp)?;
    Ok(cls.call((), Some(&dict))?.into())
}

/// Extract Odometry directly from Python object (no serde overhead)
fn extract_odometry(py: Python<'_>, obj: &PyObject) -> PyResult<Odometry> {
    let x: f64 = obj.getattr(py, "x")?.extract(py)?;
    let y: f64 = obj.getattr(py, "y")?.extract(py)?;
    let theta: f64 = obj.getattr(py, "theta")?.extract(py)?;
    let linear_velocity: f64 = obj
        .getattr(py, "linear_velocity")?
        .extract(py)
        .unwrap_or(0.0);
    let angular_velocity: f64 = obj
        .getattr(py, "angular_velocity")?
        .extract(py)
        .unwrap_or(0.0);
    let timestamp: u64 = obj.getattr(py, "timestamp")?.extract(py).unwrap_or(0);
    let mut odom = Odometry::new();
    odom.pose.x = x;
    odom.pose.y = y;
    odom.pose.theta = theta;
    odom.twist.linear[0] = linear_velocity;
    odom.twist.angular[2] = angular_velocity;
    odom.timestamp = timestamp;
    Ok(odom)
}

/// Create Python LaserScan directly using dict
fn laserscan_to_python(py: Python<'_>, scan: &LaserScan) -> PyResult<PyObject> {
    let cls = get_laserscan_class(py)?;
    let dict = PyDict::new(py);
    dict.set_item("angle_min", scan.angle_min)?;
    dict.set_item("angle_max", scan.angle_max)?;
    dict.set_item("angle_increment", scan.angle_increment)?;
    dict.set_item("range_min", scan.range_min)?;
    dict.set_item("range_max", scan.range_max)?;
    dict.set_item("ranges", scan.ranges.as_slice())?;
    dict.set_item("timestamp", scan.timestamp)?;
    Ok(cls.call((), Some(&dict))?.into())
}

/// Convert a Python object to a Rust type using serde
fn from_python<T: DeserializeOwned>(py: Python, obj: &PyObject) -> PyResult<T> {
    pythonize::depythonize(obj.bind(py)).map_err(|e| {
        pyo3::exceptions::PyTypeError::new_err(format!("Failed to convert from Python: {}", e))
    })
}

/// Convert a Rust type to a Python object using serde
fn to_python<T: Serialize>(py: Python, value: &T) -> PyResult<PyObject> {
    pythonize::pythonize(py, value)
        .map(|o| o.into())
        .map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("Failed to convert to Python: {}", e))
        })
}

/// Internal enum tracking which Rust type the Hub wraps
/// P2 OPTIMIZATION: Uses RwLock instead of Mutex for better read concurrency
enum HubType {
    CmdVel(Arc<RwLock<Hub<CmdVel>>>),
    Pose2D(Arc<RwLock<Hub<Pose2D>>>),
    Imu(Arc<RwLock<Hub<Imu>>>),
    Odometry(Arc<RwLock<Hub<Odometry>>>),
    LaserScan(Arc<RwLock<Hub<LaserScan>>>),
    Generic(Arc<RwLock<Hub<GenericMessage>>>),
}

/// Python Hub - type-safe wrapper that creates the right Rust Hub<T>
///
/// Examples:
///     hub = Hub(CmdVel)       # Creates Hub<CmdVel> - zero overhead!
///     hub = Hub(Pose2D)       # Creates Hub<Pose2D>
///     hub = Hub("custom")     # Generic hub (fallback, slower)
///
/// Network examples:
///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")  # Direct UDP
///     hub = Hub(CmdVel, endpoint="cmdvel@localhost")         # Unix socket
///     hub = Hub(CmdVel, endpoint="cmdvel@router:7777")       # Via router
///     hub = Hub(CmdVel, endpoint="cmdvel@*")                 # Multicast
#[pyclass(name = "Hub")] // Export as "Hub" in Python, not "PyHub"
pub struct PyHub {
    hub_type: HubType,
    topic: String,
    /// Endpoint string used to create this hub (if network)
    endpoint: Option<String>,
    /// Whether this hub uses network transport
    is_network: bool,
}

#[pymethods]
impl PyHub {
    /// Create a new Hub for a specific message type
    ///
    /// Args:
    ///     msg_type: Message class (CmdVel, Pose2D) or string for generic hub
    ///     capacity: Optional buffer capacity (default: 1024 if not specified)
    ///     endpoint: Optional network endpoint string for distributed communication
    ///
    /// Endpoint formats:
    ///     "topic"                    - Local shared memory (default)
    ///     "topic@host:port"          - Direct UDP to specific host
    ///     "topic@localhost"          - Unix domain socket (Unix only)
    ///     "topic@router"             - Via HORUS router (TCP broker)
    ///     "topic@router:port"        - Via router on specific port
    ///     "topic@*"                  - Multicast discovery
    ///
    /// Examples:
    ///     hub = Hub(CmdVel)                                    # Local, default capacity
    ///     hub = Hub(Pose2D, capacity=2048)                     # Local, custom capacity
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000") # Network UDP
    ///     hub = Hub(CmdVel, endpoint="cmdvel@localhost")       # Unix socket
    ///     hub = Hub(CmdVel, endpoint="cmdvel@router")          # Via router
    ///     hub = Hub("custom")                                  # Generic hub
    #[new]
    #[pyo3(signature = (msg_type, capacity=None, endpoint=None))]
    fn new(
        py: Python,
        msg_type: PyObject,
        capacity: Option<usize>,
        endpoint: Option<String>,
    ) -> PyResult<Self> {
        // Get type name from the Python object
        let type_name = if let Ok(name) = msg_type.getattr(py, "__name__") {
            name.extract::<String>(py)?
        } else if let Ok(s) = msg_type.extract::<String>(py) {
            s // String fallback for generic hubs
        } else {
            return Err(pyo3::exceptions::PyTypeError::new_err(
                "Hub() requires a message type (CmdVel, Pose2D) or topic string",
            ));
        };

        // Get topic name from type's __topic_name__, or default to lowercase type name
        let topic = if let Ok(topic_attr) = msg_type.getattr(py, "__topic_name__") {
            topic_attr.extract::<String>(py)?
        } else {
            type_name.to_lowercase()
        };

        // Determine the effective endpoint:
        // - If endpoint is provided, use it directly
        // - Otherwise, use just the topic name (local shared memory)
        let effective_endpoint = endpoint.clone().unwrap_or_else(|| topic.clone());
        let is_network = endpoint.as_ref().is_some_and(|e| e.contains('@'));
        let cap = capacity.unwrap_or(1024);

        // Create the appropriate typed Hub
        // new_with_capacity() automatically parses the endpoint string and creates
        // network backends when needed (e.g., "topic@host:port")
        let hub_type = match type_name.as_str() {
            // P2 OPTIMIZATION: Use RwLock instead of Mutex for better read concurrency
            "CmdVel" => {
                let hub =
                    Hub::<CmdVel>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<CmdVel>: {}",
                            e
                        ))
                    })?;
                HubType::CmdVel(Arc::new(RwLock::new(hub)))
            }
            "Pose2D" => {
                let hub =
                    Hub::<Pose2D>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<Pose2D>: {}",
                            e
                        ))
                    })?;
                HubType::Pose2D(Arc::new(RwLock::new(hub)))
            }
            "Imu" => {
                let hub = Hub::<Imu>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to create Hub<Imu>: {}",
                        e
                    ))
                })?;
                HubType::Imu(Arc::new(RwLock::new(hub)))
            }
            "Odometry" => {
                let hub =
                    Hub::<Odometry>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<Odometry>: {}",
                            e
                        ))
                    })?;
                HubType::Odometry(Arc::new(RwLock::new(hub)))
            }
            "LaserScan" => {
                let hub =
                    Hub::<LaserScan>::new_with_capacity(&effective_endpoint, cap).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<LaserScan>: {}",
                            e
                        ))
                    })?;
                HubType::LaserScan(Arc::new(RwLock::new(hub)))
            }
            _ => {
                // Fallback to GenericMessage for unknown types
                let hub = Hub::<GenericMessage>::new_with_capacity(&effective_endpoint, cap)
                    .map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to create Hub<GenericMessage>: {}",
                            e
                        ))
                    })?;
                HubType::Generic(Arc::new(RwLock::new(hub)))
            }
        };

        Ok(Self {
            hub_type,
            topic,
            endpoint,
            is_network,
        })
    }

    /// Send a message (type must match Hub's type)
    ///
    /// Args:
    ///     message: Message object (CmdVel, Pose2D, etc.)
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully, False otherwise
    ///
    /// Examples:
    ///     hub.send(CmdVel(1.5, 0.5), node)      # With logging
    ///     hub.send(Pose2D(1.0, 2.0, 0.5))       # Without logging
    #[pyo3(signature = (message, node=None))]
    fn send(&self, py: Python, message: PyObject, node: Option<PyObject>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // P1+P2 OPTIMIZATION: Direct field extraction + RwLock write
        let result = match &self.hub_type {
            HubType::CmdVel(hub) => {
                // P1: Direct field extraction (bypasses serde, 2-5x faster)
                let cmd = extract_cmdvel(py, &message)?;
                // P1+P2: Release GIL during IPC + RwLock write
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(cmd, &mut None).is_ok()
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "CmdVel"),
                            );
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, cmd.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            HubType::Pose2D(hub) => {
                // P1: Direct field extraction (bypasses serde, 2-5x faster)
                let pose = extract_pose2d(py, &message)?;
                // P1+P2: Release GIL during IPC + RwLock write
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(pose, &mut None).is_ok()
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "Pose2D"),
                            );
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, pose.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            HubType::Imu(hub) => {
                // P1: Direct field extraction (bypasses serde, 2-5x faster)
                let imu = extract_imu(py, &message)?;
                // P1+P2: Release GIL during IPC + RwLock write
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(imu, &mut None).is_ok()
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ =
                                info.call_method1(py, "register_publisher", (&self.topic, "Imu"));
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, imu.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            HubType::Odometry(hub) => {
                // Odometry uses serde fallback (complex nested structure)
                let odom: Odometry = from_python(py, &message)?;
                // P1+P2: Release GIL during IPC + RwLock write
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(odom, &mut None).is_ok()
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "Odometry"),
                            );
                            use horus::core::LogSummary;
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, odom.log_summary(), ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            HubType::LaserScan(hub) => {
                // LaserScan uses serde fallback (variable-length array)
                let scan: LaserScan = from_python(py, &message)?;

                // P2: Pre-compute log summary before move to avoid cloning Vec<f32>
                use horus::core::LogSummary;
                let log_summary = scan.log_summary();

                // P1+P2: Release GIL during IPC + RwLock write (move scan, no clone)
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(scan, &mut None).is_ok()
                });

                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "LaserScan"),
                            );
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, log_summary, ipc_ns),
                            );
                        }
                    }
                }
                success
            }
            HubType::Generic(hub) => {
                // Convert Python object to MessagePack via pythonize
                let bound = message.bind(py);
                let value: serde_json::Value = pythonize::depythonize(bound).map_err(|e| {
                    pyo3::exceptions::PyTypeError::new_err(format!(
                        "Failed to convert Python object: {}",
                        e
                    ))
                })?;

                // Serialize to MessagePack
                let msgpack_bytes = rmp_serde::to_vec(&value).map_err(|e| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "Failed to serialize to MessagePack: {}",
                        e
                    ))
                })?;

                // Create GenericMessage (with size validation)
                let msg = GenericMessage::new(msgpack_bytes)
                    .map_err(pyo3::exceptions::PyValueError::new_err)?;

                // P2: Pre-compute log summary before move to avoid cloning
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();

                // P1+P2: Release GIL during IPC + RwLock write (move msg, no clone)
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(msg, &mut None).is_ok()
                });

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            // Register publisher for runtime discovery
                            let _ = info.call_method1(
                                py,
                                "register_publisher",
                                (&self.topic, "GenericMessage"),
                            );
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, log_summary, ipc_ns),
                            );
                        }
                    }
                }

                success
            }
        };

        Ok(result)
    }

    /// Send multiple messages at once
    ///
    /// More efficient than calling send() multiple times - single GIL release
    /// for all IPC operations.
    ///
    /// Args:
    ///     messages: List of messages to send
    ///
    /// Returns:
    ///     Number of messages successfully sent
    ///
    /// Example:
    ///     sent = hub.send_many([msg1, msg2, msg3])
    ///     print(f"Sent {sent}/3 messages")
    fn send_many(&self, py: Python, messages: Vec<PyObject>) -> PyResult<usize> {
        if messages.is_empty() {
            return Ok(0);
        }

        match &self.hub_type {
            HubType::CmdVel(hub) => {
                // Extract all messages first (requires GIL)
                let mut cmds = Vec::with_capacity(messages.len());
                for msg in &messages {
                    cmds.push(extract_cmdvel(py, msg)?);
                }
                // Send all in one GIL release
                let hub_ref = hub.clone();
                let sent = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    cmds.into_iter()
                        .filter(|cmd| hub.send(*cmd, &mut None).is_ok())
                        .count()
                });
                Ok(sent)
            }
            HubType::Pose2D(hub) => {
                let mut poses = Vec::with_capacity(messages.len());
                for msg in &messages {
                    poses.push(extract_pose2d(py, msg)?);
                }
                let hub_ref = hub.clone();
                let sent = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    poses
                        .into_iter()
                        .filter(|pose| hub.send(*pose, &mut None).is_ok())
                        .count()
                });
                Ok(sent)
            }
            HubType::Imu(hub) => {
                let mut imus = Vec::with_capacity(messages.len());
                for msg in &messages {
                    imus.push(extract_imu(py, msg)?);
                }
                let hub_ref = hub.clone();
                let sent = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    imus.into_iter()
                        .filter(|imu| hub.send(*imu, &mut None).is_ok())
                        .count()
                });
                Ok(sent)
            }
            HubType::Odometry(hub) => {
                let mut odoms = Vec::with_capacity(messages.len());
                for msg in &messages {
                    odoms.push(extract_odometry(py, msg)?);
                }
                let hub_ref = hub.clone();
                let sent = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    odoms
                        .into_iter()
                        .filter(|odom| hub.send(*odom, &mut None).is_ok())
                        .count()
                });
                Ok(sent)
            }
            HubType::LaserScan(hub) => {
                let mut scans = Vec::with_capacity(messages.len());
                for msg in &messages {
                    scans.push(from_python::<LaserScan>(py, msg)?);
                }
                let hub_ref = hub.clone();
                let sent = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    scans
                        .into_iter()
                        .filter(|scan| hub.send(scan.clone(), &mut None).is_ok())
                        .count()
                });
                Ok(sent)
            }
            HubType::Generic(hub) => {
                let mut msgs = Vec::with_capacity(messages.len());
                for msg in &messages {
                    let bound = msg.bind(py);
                    let value: serde_json::Value = pythonize::depythonize(bound).map_err(|e| {
                        pyo3::exceptions::PyTypeError::new_err(format!(
                            "Failed to convert Python object: {}",
                            e
                        ))
                    })?;
                    let msgpack_bytes = rmp_serde::to_vec(&value).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to serialize: {}",
                            e
                        ))
                    })?;
                    let generic_msg = GenericMessage::new(msgpack_bytes)
                        .map_err(pyo3::exceptions::PyValueError::new_err)?;
                    msgs.push(generic_msg);
                }
                let hub_ref = hub.clone();
                let sent = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    msgs.into_iter()
                        .filter(|msg| hub.send(*msg, &mut None).is_ok())
                        .count()
                });
                Ok(sent)
            }
        }
    }

    /// Receive a message (returns typed object matching Hub's type)
    ///
    /// Args:
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     CmdVel/Pose2D object if available, None otherwise
    ///
    /// Examples:
    ///     cmd = hub.recv(node)       # With logging
    ///     pose = hub.recv()          # Without logging
    #[pyo3(signature = (node=None))]
    fn recv(&self, py: Python, node: Option<PyObject>) -> PyResult<Option<PyObject>> {
        use std::time::Instant;
        let start = Instant::now();

        // P0+P1+P2 OPTIMIZATION: Cached classes + direct conversion + RwLock read
        match &self.hub_type {
            HubType::CmdVel(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read (allows concurrent reads)
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(cmd) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "CmdVel"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, cmd.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    // P0+P1: Direct conversion using cached class (100x faster)
                    Ok(Some(cmdvel_to_python(py, &cmd)?))
                } else {
                    Ok(None)
                }
            }
            HubType::Pose2D(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(pose) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "Pose2D"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, pose.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    // P0+P1: Direct conversion using cached class (100x faster)
                    Ok(Some(pose2d_to_python(py, &pose)?))
                } else {
                    Ok(None)
                }
            }
            HubType::Imu(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(imu) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "Imu"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, imu.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    // P0+P1: Direct conversion using cached class (100x faster)
                    Ok(Some(imu_to_python(py, &imu)?))
                } else {
                    Ok(None)
                }
            }
            HubType::Odometry(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(odom) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "Odometry"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, odom.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    // P0+P1: Direct conversion using cached class + dict (faster for complex types)
                    Ok(Some(odometry_to_python(py, &odom)?))
                } else {
                    Ok(None)
                }
            }
            HubType::LaserScan(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(scan) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "LaserScan"),
                                );
                                use horus::core::LogSummary;
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, scan.log_summary(), ipc_ns),
                                );
                            }
                        }
                    }
                    // P0+P1: Direct conversion using cached class + dict (faster for arrays)
                    Ok(Some(laserscan_to_python(py, &scan)?))
                } else {
                    Ok(None)
                }
            }
            HubType::Generic(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(msg) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    // Log if node provided
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                // Register subscriber for runtime discovery
                                let _ = info.call_method1(
                                    py,
                                    "register_subscriber",
                                    (&self.topic, "GenericMessage"),
                                );
                                use horus::core::LogSummary;
                                let log_msg = msg.log_summary();
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, log_msg, ipc_ns),
                                );
                            }
                        }
                    }

                    // Deserialize MessagePack to serde_json::Value
                    let data = msg.data();
                    let value: serde_json::Value = rmp_serde::from_slice(&data).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to deserialize MessagePack: {}",
                            e
                        ))
                    })?;

                    // Convert serde Value to Python object
                    let py_obj = pythonize::pythonize(py, &value).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to convert to Python: {}",
                            e
                        ))
                    })?;

                    Ok(Some(py_obj.into()))
                } else {
                    Ok(None)
                }
            }
        }
    }

    /// Get the topic name
    fn topic(&self) -> String {
        self.topic.clone()
    }

    /// Send raw bytes (for generic Python hubs)
    ///
    /// Args:
    ///     data: Raw bytes to send
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully
    #[pyo3(signature = (data, node=None))]
    fn send_bytes(&self, py: Python, data: Vec<u8>, node: Option<PyObject>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Generic hubs only - wrap bytes in GenericMessage
        match &self.hub_type {
            HubType::Generic(hub) => {
                let msg =
                    GenericMessage::new(data).map_err(pyo3::exceptions::PyValueError::new_err)?;
                let hub = hub.write().unwrap();
                let success = hub.send(msg, &mut None).is_ok();

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            use horus::core::LogSummary;
                            let log_msg = msg.log_summary();
                            let _ =
                                info.call_method1(py, "log_pub", (&self.topic, log_msg, ipc_ns));
                        }
                    }
                }

                Ok(success)
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send_bytes() only supported for generic hubs",
            )),
        }
    }

    /// Send data with metadata (for generic Python hubs)
    ///
    /// Args:
    ///     data: Raw bytes to send
    ///     metadata: Metadata string (e.g., "json", "pickle", "numpy")
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully
    #[pyo3(signature = (data, _metadata, node=None))]
    fn send_with_metadata(
        &self,
        py: Python,
        data: Vec<u8>,
        _metadata: String,
        node: Option<PyObject>,
    ) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Create GenericMessage with metadata
        match &self.hub_type {
            HubType::Generic(hub) => {
                let msg = if _metadata.is_empty() {
                    GenericMessage::new(data)
                } else {
                    GenericMessage::with_metadata(data, _metadata)
                }
                .map_err(pyo3::exceptions::PyValueError::new_err)?;

                // P2: Pre-compute log summary before move to avoid cloning
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();

                // P1+P2: Release GIL during IPC + RwLock write (move msg, no clone)
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(msg, &mut None).is_ok()
                });

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, log_summary, ipc_ns),
                            );
                        }
                    }
                }

                Ok(success)
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send_with_metadata() only supported for generic hubs",
            )),
        }
    }

    /// Send numpy array (for generic Python hubs)
    ///
    /// Args:
    ///     data: Numpy array (as bytes from Python)
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     True if sent successfully
    #[pyo3(signature = (data, node=None))]
    fn send_numpy(&self, py: Python, data: PyObject, node: Option<PyObject>) -> PyResult<bool> {
        use std::time::Instant;
        let start = Instant::now();

        // Extract numpy array bytes using buffer protocol
        match &self.hub_type {
            HubType::Generic(hub) => {
                // Try to get bytes from the numpy array
                let bytes: Vec<u8> = if let Ok(bytes_obj) = data.call_method0(py, "tobytes") {
                    bytes_obj.extract(py)?
                } else {
                    // Fallback: try to extract as bytes directly
                    data.extract(py)?
                };

                let msg =
                    GenericMessage::new(bytes).map_err(pyo3::exceptions::PyValueError::new_err)?;

                // P2: Pre-compute log summary before move to avoid cloning
                use horus::core::LogSummary;
                let log_summary = msg.log_summary();

                // P1+P2: Release GIL during IPC + RwLock write (move msg, no clone)
                let hub_ref = hub.clone();
                let success = py.allow_threads(|| {
                    let hub = hub_ref.write().unwrap();
                    hub.send(msg, &mut None).is_ok()
                });

                // Log if node provided
                if let Some(node_obj) = &node {
                    let ipc_ns = start.elapsed().as_nanos() as u64;
                    if let Ok(info) = node_obj.getattr(py, "info") {
                        if !info.is_none(py) {
                            let _ = info.call_method1(
                                py,
                                "log_pub",
                                (&self.topic, log_summary, ipc_ns),
                            );
                        }
                    }
                }

                Ok(success)
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "send_numpy() only supported for generic hubs",
            )),
        }
    }

    /// Receive data with metadata (for generic Python hubs)
    ///
    /// Args:
    ///     node: Optional Node for automatic logging with IPC timing
    ///
    /// Returns:
    ///     Tuple of (bytes, metadata_str, timestamp) or None
    #[pyo3(signature = (node=None))]
    fn recv_with_metadata(
        &self,
        py: Python,
        node: Option<PyObject>,
    ) -> PyResult<Option<(PyObject, String, f64)>> {
        use std::time::Instant;
        let start = Instant::now();

        match &self.hub_type {
            HubType::Generic(hub) => {
                // P1+P2: Release GIL during IPC + RwLock read (allows concurrent reads)
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(msg) = msg_opt {
                    let ipc_ns = start.elapsed().as_nanos() as u64;

                    // Log if node provided
                    if let Some(node_obj) = &node {
                        if let Ok(info) = node_obj.getattr(py, "info") {
                            if !info.is_none(py) {
                                use horus::core::LogSummary;
                                let log_msg = msg.log_summary();
                                let _ = info.call_method1(
                                    py,
                                    "log_sub",
                                    (&self.topic, log_msg, ipc_ns),
                                );
                            }
                        }
                    }

                    // Use current time as timestamp (GenericMessage doesn't have timestamp field)
                    use std::time::{SystemTime, UNIX_EPOCH};
                    let timestamp = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs_f64();

                    // Check if message has metadata, otherwise default to "json"
                    let metadata = msg.metadata().unwrap_or_else(|| "json".to_string());

                    // Convert Vec<u8> to Python bytes object
                    let data = msg.data();
                    let py_bytes = pyo3::types::PyBytes::new(py, &data).into();

                    Ok(Some((py_bytes, metadata, timestamp)))
                } else {
                    Ok(None)
                }
            }
            _ => Err(pyo3::exceptions::PyTypeError::new_err(
                "recv_with_metadata() only supported for generic hubs",
            )),
        }
    }

    /// Check if this hub is a generic hub (supports metadata methods)
    ///
    /// Returns:
    ///     True if this is a generic hub, False if it's a typed hub
    fn is_generic(&self) -> PyResult<bool> {
        Ok(matches!(self.hub_type, HubType::Generic(_)))
    }

    // === Network Information Methods ===

    /// Check if this hub uses network transport
    ///
    /// Returns:
    ///     True if the hub is communicating over the network, False for local shared memory
    ///
    /// Example:
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    ///     print(hub.is_network())  # True
    #[getter]
    fn is_network_hub(&self) -> bool {
        self.is_network
    }

    /// Get the endpoint string (if network hub)
    ///
    /// Returns:
    ///     The endpoint string used to create this hub, or None for local hubs
    ///
    /// Example:
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    ///     print(hub.endpoint)  # "cmdvel@192.168.1.5:9000"
    #[getter]
    fn get_endpoint(&self) -> Option<String> {
        self.endpoint.clone()
    }

    /// Get transport type information
    ///
    /// Returns:
    ///     String describing the transport: "shared_memory", "unix_socket", "udp_direct",
    ///     "batch_udp", "multicast", or "router"
    ///
    /// Example:
    ///     hub = Hub(CmdVel, endpoint="cmdvel@192.168.1.5:9000")
    ///     print(hub.transport_type)  # "udp_direct" or "batch_udp"
    #[getter]
    fn transport_type(&self) -> String {
        if !self.is_network {
            return "shared_memory".to_string();
        }

        // Check endpoint pattern to determine transport type
        if let Some(ref ep) = self.endpoint {
            if ep.contains("@localhost") {
                #[cfg(unix)]
                return "unix_socket".to_string();
                #[cfg(not(unix))]
                return "udp_direct".to_string();
            } else if ep.contains("@router") {
                return "router".to_string();
            } else if ep.contains("@*") {
                return "multicast".to_string();
            } else if ep.contains("@") {
                // Direct UDP (or batch UDP on Linux)
                #[cfg(target_os = "linux")]
                return "batch_udp".to_string();
                #[cfg(not(target_os = "linux"))]
                return "udp_direct".to_string();
            }
        }

        "shared_memory".to_string()
    }

    /// Get hub statistics as a dictionary
    ///
    /// Returns:
    ///     Dictionary with keys: messages_sent, messages_received, send_failures, recv_failures
    ///
    /// Example:
    ///     stats = hub.stats()
    ///     print(f"Sent: {stats['messages_sent']}, Received: {stats['messages_received']}")
    fn stats(&self) -> PyResult<pyo3::Py<pyo3::types::PyDict>> {
        Python::with_gil(|py| {
            let dict = pyo3::types::PyDict::new(py);

            // P2: Get metrics using RwLock read (allows concurrent stats calls)
            let (sent, received, send_failures, recv_failures) = match &self.hub_type {
                HubType::CmdVel(hub) => {
                    let h = hub.read().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::Pose2D(hub) => {
                    let h = hub.read().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::Imu(hub) => {
                    let h = hub.read().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::Odometry(hub) => {
                    let h = hub.read().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::LaserScan(hub) => {
                    let h = hub.read().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
                HubType::Generic(hub) => {
                    let h = hub.read().unwrap();
                    let m = h.get_metrics();
                    (
                        m.messages_sent,
                        m.messages_received,
                        m.send_failures,
                        m.recv_failures,
                    )
                }
            };

            dict.set_item("messages_sent", sent)?;
            dict.set_item("messages_received", received)?;
            dict.set_item("send_failures", send_failures)?;
            dict.set_item("recv_failures", recv_failures)?;
            dict.set_item("is_network", self.is_network)?;
            dict.set_item("transport", self.transport_type())?;

            Ok(dict.into())
        })
    }

    // === Batch Operations (Clean API) ===

    /// Receive up to N messages at once
    ///
    /// More efficient than calling recv() N times - single GIL release
    /// for all IPC operations.
    ///
    /// Args:
    ///     n: Maximum number of messages to receive
    ///
    /// Returns:
    ///     List of messages (may be fewer than N if not available)
    ///
    /// Example:
    ///     messages = hub.recv_many(10)  # Get up to 10 messages
    ///     for msg in messages:
    ///         process(msg)
    #[pyo3(signature = (n))]
    fn recv_many(&self, py: Python, n: usize) -> PyResult<Vec<PyObject>> {
        let mut results = Vec::with_capacity(n);

        match &self.hub_type {
            HubType::CmdVel(hub) => {
                let hub_ref = hub.clone();
                let messages: Vec<_> = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    (0..n).filter_map(|_| hub.recv(&mut None)).collect()
                });
                for cmd in messages {
                    results.push(cmdvel_to_python(py, &cmd)?);
                }
            }
            HubType::Pose2D(hub) => {
                let hub_ref = hub.clone();
                let messages: Vec<_> = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    (0..n).filter_map(|_| hub.recv(&mut None)).collect()
                });
                for pose in messages {
                    results.push(pose2d_to_python(py, &pose)?);
                }
            }
            HubType::Imu(hub) => {
                let hub_ref = hub.clone();
                let messages: Vec<_> = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    (0..n).filter_map(|_| hub.recv(&mut None)).collect()
                });
                for imu in messages {
                    results.push(imu_to_python(py, &imu)?);
                }
            }
            HubType::Odometry(hub) => {
                let hub_ref = hub.clone();
                let messages: Vec<_> = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    (0..n).filter_map(|_| hub.recv(&mut None)).collect()
                });
                for odom in messages {
                    results.push(odometry_to_python(py, &odom)?);
                }
            }
            HubType::LaserScan(hub) => {
                let hub_ref = hub.clone();
                let messages: Vec<_> = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    (0..n).filter_map(|_| hub.recv(&mut None)).collect()
                });
                for scan in messages {
                    results.push(to_python(py, &scan)?);
                }
            }
            HubType::Generic(hub) => {
                let hub_ref = hub.clone();
                let messages: Vec<_> = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    (0..n).filter_map(|_| hub.recv(&mut None)).collect()
                });
                for msg in messages {
                    let data = msg.data();
                    let value: serde_json::Value = rmp_serde::from_slice(&data).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to deserialize: {}",
                            e
                        ))
                    })?;
                    let py_obj = pythonize::pythonize(py, &value).map_err(|e| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!(
                            "Failed to convert to Python: {}",
                            e
                        ))
                    })?;
                    results.push(py_obj.into());
                }
            }
        }

        Ok(results)
    }

    /// Drain all available messages from the hub
    ///
    /// Receives all pending messages in a single operation.
    /// More efficient than polling recv() until None.
    ///
    /// Returns:
    ///     List of all available messages
    ///
    /// Example:
    ///     messages = hub.drain()  # Get everything available
    ///     print(f"Received {len(messages)} messages")
    fn drain(&self, py: Python) -> PyResult<Vec<PyObject>> {
        // Use a reasonable max to prevent infinite loops
        self.recv_many(py, 1000)
    }

    /// Receive raw bytes without deserialization (GenericMessage only)
    ///
    /// Returns the raw MessagePack bytes for custom parsing.
    /// For typed messages (CmdVel, Pose2D, etc.), use recv() instead
    /// as they're already optimized for direct field access.
    ///
    /// Returns:
    ///     bytes if message available (GenericMessage only)
    ///     None if no message or not a GenericMessage hub
    ///
    /// Example:
    ///     # For custom/large data where you control deserialization:
    ///     hub = Hub("sensor_data")  # Generic topic
    ///     if raw := hub.view():
    ///         # Parse with your own logic (numpy, struct, etc.)
    ///         data = msgpack.unpackb(raw)
    fn view(&self, py: Python) -> PyResult<Option<PyObject>> {
        match &self.hub_type {
            HubType::Generic(hub) => {
                // Release GIL during IPC operation
                let hub_ref = hub.clone();
                let msg_opt = py.allow_threads(|| {
                    let hub = hub_ref.read().unwrap();
                    hub.recv(&mut None)
                });
                if let Some(msg) = msg_opt {
                    // Return raw bytes - skip JSON deserialization
                    let bytes = pyo3::types::PyBytes::new(py, &msg.data());
                    Ok(Some(bytes.into()))
                } else {
                    Ok(None)
                }
            }
            _ => {
                // For typed messages, view() doesn't provide benefit
                // since we need to construct Python objects anyway
                Err(pyo3::exceptions::PyTypeError::new_err(
                    "view() is only available for generic (untyped) hubs. \
                     For typed messages (CmdVel, Pose2D, etc.), use recv() which is already optimized.",
                ))
            }
        }
    }

    /// String representation
    fn __repr__(&self) -> String {
        let transport = self.transport_type();
        if self.is_network {
            format!(
                "Hub(topic='{}', endpoint='{}', transport='{}')",
                self.topic,
                self.endpoint.as_deref().unwrap_or("unknown"),
                transport
            )
        } else {
            format!("Hub(topic='{}', transport='{}')", self.topic, transport)
        }
    }
}

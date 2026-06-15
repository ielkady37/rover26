#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__EulerAngles() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__EulerAngles__init(msg: *mut EulerAngles) -> bool;
    fn interfaces__msg__EulerAngles__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EulerAngles>, size: usize) -> bool;
    fn interfaces__msg__EulerAngles__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EulerAngles>);
    fn interfaces__msg__EulerAngles__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EulerAngles>, out_seq: *mut rosidl_runtime_rs::Sequence<EulerAngles>) -> bool;
}

// Corresponds to interfaces__msg__EulerAngles
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EulerAngles {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// degrees
    pub yaw: f32,

    /// degrees
    pub pitch: f32,

    /// degrees
    pub roll: f32,

}



impl Default for EulerAngles {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__EulerAngles__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__EulerAngles__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EulerAngles {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__EulerAngles__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__EulerAngles__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__EulerAngles__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EulerAngles {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EulerAngles where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/EulerAngles";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__EulerAngles() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__EncoderRevolutions() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__EncoderRevolutions__init(msg: *mut EncoderRevolutions) -> bool;
    fn interfaces__msg__EncoderRevolutions__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EncoderRevolutions>, size: usize) -> bool;
    fn interfaces__msg__EncoderRevolutions__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EncoderRevolutions>);
    fn interfaces__msg__EncoderRevolutions__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EncoderRevolutions>, out_seq: *mut rosidl_runtime_rs::Sequence<EncoderRevolutions>) -> bool;
}

// Corresponds to interfaces__msg__EncoderRevolutions
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EncoderRevolutions {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// motor 1 net revolutions (forward positive)
    pub enc1_net_rev: f32,

    /// motor 2 net revolutions (forward positive)
    pub enc2_net_rev: f32,

}



impl Default for EncoderRevolutions {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__EncoderRevolutions__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__EncoderRevolutions__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EncoderRevolutions {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__EncoderRevolutions__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__EncoderRevolutions__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__EncoderRevolutions__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EncoderRevolutions {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EncoderRevolutions where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/EncoderRevolutions";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__EncoderRevolutions() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__ActuatorCommand() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__ActuatorCommand__init(msg: *mut ActuatorCommand) -> bool;
    fn interfaces__msg__ActuatorCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ActuatorCommand>, size: usize) -> bool;
    fn interfaces__msg__ActuatorCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ActuatorCommand>);
    fn interfaces__msg__ActuatorCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ActuatorCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<ActuatorCommand>) -> bool;
}

// Corresponds to interfaces__msg__ActuatorCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActuatorCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Motor 1
    /// 0=forward  1=reverse
    pub m1_dir: u8,

    /// 0=off      1=on
    pub m1_brake: u8,

    /// 0.0–1.0 duty cycle
    pub m1_speed: f32,

    /// Motor 2
    /// 0=forward  1=reverse
    pub m2_dir: u8,

    /// 0=off      1=on
    pub m2_brake: u8,

    /// 0.0–1.0 duty cycle
    pub m2_speed: f32,

    /// Laser
    /// 0=off  1=on
    pub laser: u8,

    /// Flash
    /// 0=off  1=on
    pub flash: u8,

    /// Servo
    /// degrees
    pub servo: f32,

}



impl Default for ActuatorCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__ActuatorCommand__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__ActuatorCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ActuatorCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__ActuatorCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__ActuatorCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__ActuatorCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ActuatorCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ActuatorCommand where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/ActuatorCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__ActuatorCommand() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__LogEntry() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__LogEntry__init(msg: *mut LogEntry) -> bool;
    fn interfaces__msg__LogEntry__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LogEntry>, size: usize) -> bool;
    fn interfaces__msg__LogEntry__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LogEntry>);
    fn interfaces__msg__LogEntry__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LogEntry>, out_seq: *mut rosidl_runtime_rs::Sequence<LogEntry>) -> bool;
}

// Corresponds to interfaces__msg__LogEntry
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LogEntry {
    /// SUCC | INFO | WARN | ERR
    pub level: rosidl_runtime_rs::String,

    /// Name of the class that called the logger
    pub source_class: rosidl_runtime_rs::String,

    /// nodes | services | helpers | general | …
    pub category: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for LogEntry {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__LogEntry__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__LogEntry__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LogEntry {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__LogEntry__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__LogEntry__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__LogEntry__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LogEntry {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LogEntry where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/LogEntry";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__LogEntry() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__HealthReport() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__HealthReport__init(msg: *mut HealthReport) -> bool;
    fn interfaces__msg__HealthReport__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HealthReport>, size: usize) -> bool;
    fn interfaces__msg__HealthReport__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HealthReport>);
    fn interfaces__msg__HealthReport__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HealthReport>, out_seq: *mut rosidl_runtime_rs::Sequence<HealthReport>) -> bool;
}

// Corresponds to interfaces__msg__HealthReport
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HealthReport {
    /// 0-100 %
    pub cpu_percent: f32,

    /// 0-100 %
    pub ram_percent: f32,

    /// degrees Celsius (-1 if unavailable)
    pub cpu_temp_celsius: f32,

    /// avg ping RTT to default gateway in ms  (-1 if unavailable)
    pub wifi_latency_ms: f32,

    /// Mbps (-1 if unavailable)
    pub wifi_bandwidth_mbps: f32,

    /// root partition 0-100 %
    pub disk_percent: f32,

    /// 0-100 % (-1 if no battery)
    pub battery_percent: f32,

    /// true if charging / on AC power
    pub battery_plugged: bool,

    /// default gateway that was pinged (e.g. 192.168.1.1)
    pub gateway_ip: rosidl_runtime_rs::String,

    /// "ServiceName:ok" | "ServiceName:error:<reason>"
    pub health_checks: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for HealthReport {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__HealthReport__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__HealthReport__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HealthReport {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__HealthReport__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__HealthReport__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__HealthReport__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HealthReport {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HealthReport where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/HealthReport";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__HealthReport() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__Joystick() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__Joystick__init(msg: *mut Joystick) -> bool;
    fn interfaces__msg__Joystick__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Joystick>, size: usize) -> bool;
    fn interfaces__msg__Joystick__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Joystick>);
    fn interfaces__msg__Joystick__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Joystick>, out_seq: *mut rosidl_runtime_rs::Sequence<Joystick>) -> bool;
}

// Corresponds to interfaces__msg__Joystick
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Joystick {
    /// Yaw
    pub left_x_axis: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_y_axis: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_x_axis: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_y_axis: f64,

    /// Analog Acceleration
    pub r2_axis: f64,

    /// Analog Brake/Reverse
    pub l2_axis: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_x: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_o: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_tri: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_rect: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_r1: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_r2: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_r3: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_l1: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_l2: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_l3: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_left: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_right: bool,

    /// Use for Servo Up
    pub button_top: bool,

    /// Use for Servo Down
    pub button_bot: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_options: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_share: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_pad: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub button_ps: bool,

}



impl Default for Joystick {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__Joystick__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__Joystick__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Joystick {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__Joystick__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__Joystick__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__Joystick__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Joystick {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Joystick where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/Joystick";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__Joystick() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__TargetStatus() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__TargetStatus__init(msg: *mut TargetStatus) -> bool;
    fn interfaces__msg__TargetStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TargetStatus>, size: usize) -> bool;
    fn interfaces__msg__TargetStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TargetStatus>);
    fn interfaces__msg__TargetStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TargetStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<TargetStatus>) -> bool;
}

// Corresponds to interfaces__msg__TargetStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TargetStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub is_found: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence_score: f32,

}



impl Default for TargetStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__TargetStatus__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__TargetStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TargetStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__TargetStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__TargetStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__TargetStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TargetStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TargetStatus where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/TargetStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__TargetStatus() }
  }
}



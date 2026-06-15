#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "rover26__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rover26__msg__LaneStatus() -> *const std::ffi::c_void;
}

#[link(name = "rover26__rosidl_generator_c")]
extern "C" {
    fn rover26__msg__LaneStatus__init(msg: *mut LaneStatus) -> bool;
    fn rover26__msg__LaneStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LaneStatus>, size: usize) -> bool;
    fn rover26__msg__LaneStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LaneStatus>);
    fn rover26__msg__LaneStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LaneStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<LaneStatus>) -> bool;
}

// Corresponds to rover26__msg__LaneStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaneStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_angle: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub steering_magnitude: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub direction: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu_yaw_rate: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub imu_accel_x: f32,

    /// Lane polynomial coefficients [a, b, c] for x = a*y^2 + b*y + c
    pub left_coeffs: [f32; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_coeffs: [f32; 3],

}



impl Default for LaneStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rover26__msg__LaneStatus__init(&mut msg as *mut _) {
        panic!("Call to rover26__msg__LaneStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LaneStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rover26__msg__LaneStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rover26__msg__LaneStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rover26__msg__LaneStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LaneStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LaneStatus where Self: Sized {
  const TYPE_NAME: &'static str = "rover26/msg/LaneStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rover26__msg__LaneStatus() }
  }
}


#[link(name = "rover26__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__rover26__msg__ObstacleStatus() -> *const std::ffi::c_void;
}

#[link(name = "rover26__rosidl_generator_c")]
extern "C" {
    fn rover26__msg__ObstacleStatus__init(msg: *mut ObstacleStatus) -> bool;
    fn rover26__msg__ObstacleStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ObstacleStatus>, size: usize) -> bool;
    fn rover26__msg__ObstacleStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ObstacleStatus>);
    fn rover26__msg__ObstacleStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ObstacleStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ObstacleStatus>) -> bool;
}

// Corresponds to rover26__msg__ObstacleStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ObstacleStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !rover26__msg__ObstacleStatus__init(&mut msg as *mut _) {
        panic!("Call to rover26__msg__ObstacleStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ObstacleStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rover26__msg__ObstacleStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rover26__msg__ObstacleStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { rover26__msg__ObstacleStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ObstacleStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ObstacleStatus where Self: Sized {
  const TYPE_NAME: &'static str = "rover26/msg/ObstacleStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__rover26__msg__ObstacleStatus() }
  }
}



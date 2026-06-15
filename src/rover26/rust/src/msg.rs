#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to rover26__msg__LaneStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LaneStatus::default())
  }
}

impl rosidl_runtime_rs::Message for LaneStatus {
  type RmwMsg = super::msg::rmw::LaneStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        steering_angle: msg.steering_angle,
        steering_magnitude: msg.steering_magnitude,
        direction: msg.direction,
        left_detected: msg.left_detected,
        right_detected: msg.right_detected,
        confidence: msg.confidence,
        imu_yaw_rate: msg.imu_yaw_rate,
        imu_accel_x: msg.imu_accel_x,
        left_coeffs: msg.left_coeffs,
        right_coeffs: msg.right_coeffs,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      steering_angle: msg.steering_angle,
      steering_magnitude: msg.steering_magnitude,
      direction: msg.direction,
      left_detected: msg.left_detected,
      right_detected: msg.right_detected,
      confidence: msg.confidence,
      imu_yaw_rate: msg.imu_yaw_rate,
      imu_accel_x: msg.imu_accel_x,
        left_coeffs: msg.left_coeffs,
        right_coeffs: msg.right_coeffs,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      steering_angle: msg.steering_angle,
      steering_magnitude: msg.steering_magnitude,
      direction: msg.direction,
      left_detected: msg.left_detected,
      right_detected: msg.right_detected,
      confidence: msg.confidence,
      imu_yaw_rate: msg.imu_yaw_rate,
      imu_accel_x: msg.imu_accel_x,
      left_coeffs: msg.left_coeffs,
      right_coeffs: msg.right_coeffs,
    }
  }
}


// Corresponds to rover26__msg__ObstacleStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ObstacleStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ObstacleStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ObstacleStatus {
  type RmwMsg = super::msg::rmw::ObstacleStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}



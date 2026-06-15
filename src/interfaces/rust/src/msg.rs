#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to interfaces__msg__EulerAngles

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EulerAngles {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// degrees
    pub yaw: f32,

    /// degrees
    pub pitch: f32,

    /// degrees
    pub roll: f32,

}



impl Default for EulerAngles {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EulerAngles::default())
  }
}

impl rosidl_runtime_rs::Message for EulerAngles {
  type RmwMsg = super::msg::rmw::EulerAngles;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        yaw: msg.yaw,
        pitch: msg.pitch,
        roll: msg.roll,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      yaw: msg.yaw,
      pitch: msg.pitch,
      roll: msg.roll,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      yaw: msg.yaw,
      pitch: msg.pitch,
      roll: msg.roll,
    }
  }
}


// Corresponds to interfaces__msg__EncoderRevolutions

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EncoderRevolutions {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// motor 1 net revolutions (forward positive)
    pub enc1_net_rev: f32,

    /// motor 2 net revolutions (forward positive)
    pub enc2_net_rev: f32,

}



impl Default for EncoderRevolutions {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EncoderRevolutions::default())
  }
}

impl rosidl_runtime_rs::Message for EncoderRevolutions {
  type RmwMsg = super::msg::rmw::EncoderRevolutions;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        enc1_net_rev: msg.enc1_net_rev,
        enc2_net_rev: msg.enc2_net_rev,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      enc1_net_rev: msg.enc1_net_rev,
      enc2_net_rev: msg.enc2_net_rev,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      enc1_net_rev: msg.enc1_net_rev,
      enc2_net_rev: msg.enc2_net_rev,
    }
  }
}


// Corresponds to interfaces__msg__ActuatorCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ActuatorCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ActuatorCommand::default())
  }
}

impl rosidl_runtime_rs::Message for ActuatorCommand {
  type RmwMsg = super::msg::rmw::ActuatorCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        m1_dir: msg.m1_dir,
        m1_brake: msg.m1_brake,
        m1_speed: msg.m1_speed,
        m2_dir: msg.m2_dir,
        m2_brake: msg.m2_brake,
        m2_speed: msg.m2_speed,
        laser: msg.laser,
        flash: msg.flash,
        servo: msg.servo,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      m1_dir: msg.m1_dir,
      m1_brake: msg.m1_brake,
      m1_speed: msg.m1_speed,
      m2_dir: msg.m2_dir,
      m2_brake: msg.m2_brake,
      m2_speed: msg.m2_speed,
      laser: msg.laser,
      flash: msg.flash,
      servo: msg.servo,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      m1_dir: msg.m1_dir,
      m1_brake: msg.m1_brake,
      m1_speed: msg.m1_speed,
      m2_dir: msg.m2_dir,
      m2_brake: msg.m2_brake,
      m2_speed: msg.m2_speed,
      laser: msg.laser,
      flash: msg.flash,
      servo: msg.servo,
    }
  }
}


// Corresponds to interfaces__msg__LogEntry

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LogEntry {
    /// SUCC | INFO | WARN | ERR
    pub level: std::string::String,

    /// Name of the class that called the logger
    pub source_class: std::string::String,

    /// nodes | services | helpers | general | …
    pub category: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for LogEntry {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LogEntry::default())
  }
}

impl rosidl_runtime_rs::Message for LogEntry {
  type RmwMsg = super::msg::rmw::LogEntry;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        level: msg.level.as_str().into(),
        source_class: msg.source_class.as_str().into(),
        category: msg.category.as_str().into(),
        message: msg.message.as_str().into(),
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        level: msg.level.as_str().into(),
        source_class: msg.source_class.as_str().into(),
        category: msg.category.as_str().into(),
        message: msg.message.as_str().into(),
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      level: msg.level.to_string(),
      source_class: msg.source_class.to_string(),
      category: msg.category.to_string(),
      message: msg.message.to_string(),
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to interfaces__msg__HealthReport

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub gateway_ip: std::string::String,

    /// "ServiceName:ok" | "ServiceName:error:<reason>"
    pub health_checks: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,

}



impl Default for HealthReport {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HealthReport::default())
  }
}

impl rosidl_runtime_rs::Message for HealthReport {
  type RmwMsg = super::msg::rmw::HealthReport;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cpu_percent: msg.cpu_percent,
        ram_percent: msg.ram_percent,
        cpu_temp_celsius: msg.cpu_temp_celsius,
        wifi_latency_ms: msg.wifi_latency_ms,
        wifi_bandwidth_mbps: msg.wifi_bandwidth_mbps,
        disk_percent: msg.disk_percent,
        battery_percent: msg.battery_percent,
        battery_plugged: msg.battery_plugged,
        gateway_ip: msg.gateway_ip.as_str().into(),
        health_checks: msg.health_checks
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      cpu_percent: msg.cpu_percent,
      ram_percent: msg.ram_percent,
      cpu_temp_celsius: msg.cpu_temp_celsius,
      wifi_latency_ms: msg.wifi_latency_ms,
      wifi_bandwidth_mbps: msg.wifi_bandwidth_mbps,
      disk_percent: msg.disk_percent,
      battery_percent: msg.battery_percent,
      battery_plugged: msg.battery_plugged,
        gateway_ip: msg.gateway_ip.as_str().into(),
        health_checks: msg.health_checks
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cpu_percent: msg.cpu_percent,
      ram_percent: msg.ram_percent,
      cpu_temp_celsius: msg.cpu_temp_celsius,
      wifi_latency_ms: msg.wifi_latency_ms,
      wifi_bandwidth_mbps: msg.wifi_bandwidth_mbps,
      disk_percent: msg.disk_percent,
      battery_percent: msg.battery_percent,
      battery_plugged: msg.battery_plugged,
      gateway_ip: msg.gateway_ip.to_string(),
      health_checks: msg.health_checks
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
    }
  }
}


// Corresponds to interfaces__msg__Joystick

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Joystick::default())
  }
}

impl rosidl_runtime_rs::Message for Joystick {
  type RmwMsg = super::msg::rmw::Joystick;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        left_x_axis: msg.left_x_axis,
        left_y_axis: msg.left_y_axis,
        right_x_axis: msg.right_x_axis,
        right_y_axis: msg.right_y_axis,
        r2_axis: msg.r2_axis,
        l2_axis: msg.l2_axis,
        button_x: msg.button_x,
        button_o: msg.button_o,
        button_tri: msg.button_tri,
        button_rect: msg.button_rect,
        button_r1: msg.button_r1,
        button_r2: msg.button_r2,
        button_r3: msg.button_r3,
        button_l1: msg.button_l1,
        button_l2: msg.button_l2,
        button_l3: msg.button_l3,
        button_left: msg.button_left,
        button_right: msg.button_right,
        button_top: msg.button_top,
        button_bot: msg.button_bot,
        button_options: msg.button_options,
        button_share: msg.button_share,
        button_pad: msg.button_pad,
        button_ps: msg.button_ps,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      left_x_axis: msg.left_x_axis,
      left_y_axis: msg.left_y_axis,
      right_x_axis: msg.right_x_axis,
      right_y_axis: msg.right_y_axis,
      r2_axis: msg.r2_axis,
      l2_axis: msg.l2_axis,
      button_x: msg.button_x,
      button_o: msg.button_o,
      button_tri: msg.button_tri,
      button_rect: msg.button_rect,
      button_r1: msg.button_r1,
      button_r2: msg.button_r2,
      button_r3: msg.button_r3,
      button_l1: msg.button_l1,
      button_l2: msg.button_l2,
      button_l3: msg.button_l3,
      button_left: msg.button_left,
      button_right: msg.button_right,
      button_top: msg.button_top,
      button_bot: msg.button_bot,
      button_options: msg.button_options,
      button_share: msg.button_share,
      button_pad: msg.button_pad,
      button_ps: msg.button_ps,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      left_x_axis: msg.left_x_axis,
      left_y_axis: msg.left_y_axis,
      right_x_axis: msg.right_x_axis,
      right_y_axis: msg.right_y_axis,
      r2_axis: msg.r2_axis,
      l2_axis: msg.l2_axis,
      button_x: msg.button_x,
      button_o: msg.button_o,
      button_tri: msg.button_tri,
      button_rect: msg.button_rect,
      button_r1: msg.button_r1,
      button_r2: msg.button_r2,
      button_r3: msg.button_r3,
      button_l1: msg.button_l1,
      button_l2: msg.button_l2,
      button_l3: msg.button_l3,
      button_left: msg.button_left,
      button_right: msg.button_right,
      button_top: msg.button_top,
      button_bot: msg.button_bot,
      button_options: msg.button_options,
      button_share: msg.button_share,
      button_pad: msg.button_pad,
      button_ps: msg.button_ps,
    }
  }
}


// Corresponds to interfaces__msg__TargetStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TargetStatus::default())
  }
}

impl rosidl_runtime_rs::Message for TargetStatus {
  type RmwMsg = super::msg::rmw::TargetStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        is_found: msg.is_found,
        confidence_score: msg.confidence_score,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      is_found: msg.is_found,
      confidence_score: msg.confidence_score,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      is_found: msg.is_found,
      confidence_score: msg.confidence_score,
    }
  }
}



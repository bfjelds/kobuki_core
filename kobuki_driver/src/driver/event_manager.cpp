/**
 * @file /kobuki_driver/src/driver/event_manager.cpp
 *
 * @brief Implementation of the event black magic.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/kobuki_driver/event_manager.hpp"
#include "../../include/kobuki_driver/modules/battery.hpp"
#include "../../include/kobuki_driver/packets/core_sensors.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Implementation
*****************************************************************************/

void EventManager::init ( const std::string &sigslots_namespace ) {
#ifdef ROS2
  auto node = rclcpp::Node::make_shared("kobuki_sensors");
  bumper_pub = node->create_publisher<kobuki_msgs::msg::BumperEvent>("events/bumper", rmw_qos_profile_sensor_data);
  cliff_pub = node->create_publisher<kobuki_msgs::msg::CliffEvent>("events/cliff", rmw_qos_profile_sensor_data);
  drop_pub = node->create_publisher<kobuki_msgs::msg::WheelDropEvent>("events/drop", rmw_qos_profile_sensor_data);
  button_pub = node->create_publisher<kobuki_msgs::msg::ButtonEvent>("events/button", rmw_qos_profile_sensor_data);
  power_pub = node->create_publisher<kobuki_msgs::msg::PowerSystemEvent>("events/power", rmw_qos_profile_sensor_data);
  input_pub = node->create_publisher<kobuki_msgs::msg::DigitalInputEvent>("events/input", rmw_qos_profile_sensor_data);
  robot_pub = node->create_publisher<kobuki_msgs::msg::RobotStateEvent>("events/robot", rmw_qos_profile_sensor_data);
#else
  sig_button_event.connect(sigslots_namespace + std::string("/button_event"));
  sig_bumper_event.connect(sigslots_namespace + std::string("/bumper_event"));
  sig_cliff_event.connect(sigslots_namespace  + std::string("/cliff_event"));
  sig_wheel_event.connect(sigslots_namespace  + std::string("/wheel_event"));
  sig_power_event.connect(sigslots_namespace  + std::string("/power_event"));
  sig_input_event.connect(sigslots_namespace  + std::string("/input_event"));
  sig_robot_event.connect(sigslots_namespace  + std::string("/robot_event"));
#endif // ROS2
}

/**
 * Update with incoming data and emit events if necessary.
 * @param new_state  Updated core sensors state
 * @param cliff_data Cliff sensors readings (we include them as an extra information on cliff events)
 */
void EventManager::update(const CoreSensors::Data &new_state, const std::vector<uint16_t> &cliff_data) {
  if (last_state.buttons != new_state.buttons)
  {
    // ------------
    // Button Event
    // ------------

    // Note that the touch pad means at most one button can be pressed
    // at a time.
#ifdef ROS2
	  kobuki_msgs::msg::ButtonEvent event;
#else
	  ButtonEvent event;
#endif // ROS2

    // Check changes in each button state's; even if this block of code
    // supports it, two buttons cannot be pressed simultaneously
    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::Button0) {
      event.button = ButtonEvent::Button0;
      if (new_state.buttons & CoreSensors::Flags::Button0) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
#ifdef ROS2
	  button_pub->publish(event);
#else
	  sig_button_event.emit(event);
#endif // ROS2
    }

    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::Button1) {
      event.button = ButtonEvent::Button1;
      if (new_state.buttons & CoreSensors::Flags::Button1) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
#ifdef ROS2
	  button_pub->publish(event);
#else
	  sig_button_event.emit(event);
#endif // ROS2
	}

    if ((new_state.buttons ^ last_state.buttons) & CoreSensors::Flags::Button2) {
      event.button = ButtonEvent::Button2;
      if (new_state.buttons & CoreSensors::Flags::Button2) {
        event.state = ButtonEvent::Pressed;
      } else {
        event.state = ButtonEvent::Released;
      }
#ifdef ROS2
	  button_pub->publish(event);
#else
	  sig_button_event.emit(event);
#endif // ROS2
	}
  }

  // ------------
  // Bumper Event
  // ------------

  if (last_state.bumper != new_state.bumper)
  {
#ifdef ROS2
	  kobuki_msgs::msg::BumperEvent event;
#else
	  BumperEvent event;
#endif // ROS2

    // Check changes in each bumper state's and raise an event if so
    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::LeftBumper) {
      event.bumper = BumperEvent::Left;
      if (new_state.bumper & CoreSensors::Flags::LeftBumper) {
        event.state = BumperEvent::Pressed;
      } else {
        event.state = BumperEvent::Released;
      }
#ifdef ROS2
	  bumper_pub->publish(event);
#else
	  sig_bumper_event.emit(event);
#endif // ROS2
	  printf("BUMPER event event.state=%d \r\n", event.state);
    }

    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::CenterBumper) {
      event.bumper = BumperEvent::Center;
      if (new_state.bumper & CoreSensors::Flags::CenterBumper) {
        event.state = BumperEvent::Pressed;
      } else {
        event.state = BumperEvent::Released;
      }
#ifdef ROS2
	  bumper_pub->publish(event);
#else
	  sig_bumper_event.emit(event);
#endif // ROS2
	  printf("BUMPER event event.state=%d \r\n", event.state);
    }

    if ((new_state.bumper ^ last_state.bumper) & CoreSensors::Flags::RightBumper) {
      event.bumper = BumperEvent::Right;
      if (new_state.bumper & CoreSensors::Flags::RightBumper) {
        event.state = BumperEvent::Pressed;
      } else {
        event.state = BumperEvent::Released;
      }
#ifdef ROS2
	  bumper_pub->publish(event);
#else
	  sig_bumper_event.emit(event);
#endif // ROS2
	  printf("BUMPER event event.state=%d \r\n", event.state);
    }
  }

  // ------------
  // Cliff Event
  // ------------

  if (last_state.cliff != new_state.cliff)
  {
#ifdef ROS2
	  kobuki_msgs::msg::CliffEvent event;
#else
	  CliffEvent event;
#endif // ROS2

    // Check changes in each cliff sensor state's and raise an event if so
    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::LeftCliff) {
      event.sensor = CliffEvent::Left;
      if (new_state.cliff & CoreSensors::Flags::LeftCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      event.bottom = cliff_data[event.sensor];
#ifdef ROS2
	  cliff_pub->publish(event);
#else
	  sig_cliff_event.emit(event);
#endif // ROS2
	  printf("CLIFF event event.state=%d \r\n", event.state);
    }

    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::CenterCliff) {
      event.sensor = CliffEvent::Center;
      if (new_state.cliff & CoreSensors::Flags::CenterCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      event.bottom = cliff_data[event.sensor];
#ifdef ROS2
	  cliff_pub->publish(event);
#else
	  sig_cliff_event.emit(event);
#endif // ROS2
	  printf("CLIFF event event.state=%d \r\n", event.state);
	}

    if ((new_state.cliff ^ last_state.cliff) & CoreSensors::Flags::RightCliff) {
      event.sensor = CliffEvent::Right;
      if (new_state.cliff & CoreSensors::Flags::RightCliff) {
        event.state = CliffEvent::Cliff;
      } else {
        event.state = CliffEvent::Floor;
      }
      event.bottom = cliff_data[event.sensor];
#ifdef ROS2
	  cliff_pub->publish(event);
#else
	  sig_cliff_event.emit(event);
#endif // ROS2
	  printf("CLIFF event event.state=%d \r\n", event.state);
	}
  }

  // ------------
  // Wheel Drop Event
  // ------------

  if (last_state.wheel_drop != new_state.wheel_drop)
  {
#ifdef ROS2
	  kobuki_msgs::msg::WheelDropEvent event;
#else
	  WheelEvent event;
#endif // ROS2

    // Check changes in each wheel_drop sensor state's and raise an event if so
    if ((new_state.wheel_drop ^ last_state.wheel_drop) & CoreSensors::Flags::LeftWheel) {
      event.wheel = WheelEvent::Left;
      if (new_state.wheel_drop & CoreSensors::Flags::LeftWheel) {
        event.state = WheelEvent::Dropped;
      } else {
        event.state = WheelEvent::Raised;
      }
#ifdef ROS2
	  drop_pub->publish(event);
#else
	  sig_wheel_event.emit(event);
#endif // ROS2
	  printf("DROP event event.state=%d \r\n", event.state);
	}

    if ((new_state.wheel_drop ^ last_state.wheel_drop) & CoreSensors::Flags::RightWheel) {
      event.wheel = WheelEvent::Right;
      if (new_state.wheel_drop & CoreSensors::Flags::RightWheel) {
        event.state = WheelEvent::Dropped;
      } else {
        event.state = WheelEvent::Raised;
      }
#ifdef ROS2
	  drop_pub->publish(event);
#else
	  sig_wheel_event.emit(event);
#endif // ROS2
	  printf("DROP event event.state=%d \r\n", event.state);
	}
  }

  // ------------
  // Power System Event
  // ------------

  if (last_state.charger != new_state.charger)
  {
    Battery battery_new(new_state.battery, new_state.charger);
    Battery battery_last(last_state.battery, last_state.charger);

    if (battery_last.charging_state != battery_new.charging_state)
    {
#ifdef ROS2
	  kobuki_msgs::msg::PowerSystemEvent event;
#else
	  PowerEvent event;
#endif // ROS2
      switch (battery_new.charging_state)
      {
        case Battery::Discharging:
          event.event = PowerEvent::Unplugged;
          break;
        case Battery::Charged:
          event.event = PowerEvent::ChargeCompleted;
          break;
        case Battery::Charging:
          if (battery_new.charging_source == Battery::Adapter)
            event.event = PowerEvent::PluggedToAdapter;
          else
            event.event = PowerEvent::PluggedToDockbase;
          break;
      }
#ifdef ROS2
	  power_pub->publish(event);
#else
	  sig_power_event.emit(event);
#endif // ROS2
    }
  }

  if (last_state.battery > new_state.battery)
  {
    Battery battery_new(new_state.battery, new_state.charger);
    Battery battery_last(last_state.battery, last_state.charger);

    if (battery_last.level() != battery_new.level())
    {
#ifdef ROS2
	  kobuki_msgs::msg::PowerSystemEvent event;
#else
	  PowerEvent event;
#endif // ROS2
      switch (battery_new.level())
      {
        case Battery::Low:
          event.event = PowerEvent::BatteryLow;
          break;
        case Battery::Dangerous:
          event.event = PowerEvent::BatteryCritical;
          break;
        default:
          break;
      }
#ifdef ROS2
	  power_pub->publish(event);
#else
	  sig_power_event.emit(event);
#endif // ROS2
	}
  }

  last_state = new_state;
}

/**
 * Emit events if something changed in the digital input port.
 * @param new_digital_input New values on digital input port.
 */
void EventManager::update(const uint16_t &new_digital_input)
{
  if (last_digital_input != new_digital_input)
  {
#ifdef ROS2
	  kobuki_msgs::msg::DigitalInputEvent event;
#else
	  InputEvent event;
#endif // ROS2

    event.values[0] = new_digital_input&0x0001;
    event.values[1] = new_digital_input&0x0002;
    event.values[2] = new_digital_input&0x0004;
    event.values[3] = new_digital_input&0x0008;

#ifdef ROS2
	input_pub->publish(event);
#else
	sig_input_event.emit(event);
#endif // ROS2

    last_digital_input = new_digital_input;
  }
}

/**
 * Emit events if the robot gets online/offline.
 * @param is_plugged Is the USB cable connected?.
 * @param is_alive Is the robot alive?.
 */
void EventManager::update(bool is_plugged, bool is_alive)
{
  RobotEvent::State robot_state =
      (is_plugged && is_alive)?RobotEvent::Online:RobotEvent::Offline;
  if (last_robot_state != robot_state)
  {
#ifdef ROS2
	kobuki_msgs::msg::RobotStateEvent event;
#else
	RobotEvent event;
#endif // ROS2
    event.state = robot_state;

#ifdef ROS2
	robot_pub->publish(event);
#else
	sig_robot_event.emit(event);
#endif // ROS2

    last_robot_state = robot_state;
  }
}

} // namespace kobuki

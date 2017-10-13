/**
 * @file include/kobuki_driver/event_manager.hpp
 *
 * @brief The event manager - sigslot interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_BUTTON_EVENT_HPP_
#define KOBUKI_BUTTON_EVENT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <kobuki_msgs/msg/bumper_event.hpp>
#include <kobuki_msgs/msg/cliff_event.hpp>
#include <kobuki_msgs/msg/led.hpp>
#include <kobuki_msgs/msg/wheel_drop_event.hpp>
#include <kobuki_msgs/msg/button_event.hpp>
#include <kobuki_msgs/msg/power_system_event.hpp>
#include <kobuki_msgs/msg/digital_input_event.hpp>
#include <kobuki_msgs/msg/robot_state_event.hpp>

#include "packets/core_sensors.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Event Structures
*****************************************************************************/

struct ButtonEvent {
  enum State {
    Released,
    Pressed
  } state;
  enum Button {
    Button0,
    Button1,
    Button2
  } button;
};

struct BumperEvent {
  enum State {
    Released,
    Pressed
  } state;
  enum Bumper {
    Left,
    Center,
    Right
  } bumper;
};

struct CliffEvent {
  enum State {
    Floor,
    Cliff
  } state;
  enum Sensor {
    Left,
    Center,
    Right
  } sensor;
  uint16_t bottom;
};

struct WheelEvent {
  enum State {
    Raised,
    Dropped
  } state;
  enum Wheel {
    Left,
    Right
  } wheel;
};

struct PowerEvent {
  enum Event {
    Unplugged         = 0,
    PluggedToAdapter  = 1,
    PluggedToDockbase = 2,
    ChargeCompleted   = 3,
    BatteryLow        = 4,
    BatteryCritical   = 5
  } event;
};

struct InputEvent {
  bool values[4]; /**< Digital on or off for pins 0-3 respectively. **/
};

struct RobotEvent {
  enum State {
    Offline,
    Online,
    Unknown  // at startup
  } state;
};

/*****************************************************************************
** Interfaces
*****************************************************************************/

class kobuki_PUBLIC EventManager {
public:
  EventManager() {
    last_state.buttons    = 0;
    last_state.bumper     = 0;
    last_state.cliff      = 0;
    last_state.wheel_drop = 0;
    last_state.charger    = 0;
    last_state.battery    = 0;
    last_digital_input    = 0;
    last_robot_state      = RobotEvent::Unknown;
  }

  void init(const std::string &sigslots_namespace);
  void update(const CoreSensors::Data &new_state, const std::vector<uint16_t> &cliff_data);
  void update(const uint16_t &digital_input);
  void update(bool is_plugged, bool is_alive);

private:
  CoreSensors::Data last_state;
  uint16_t          last_digital_input;
  RobotEvent::State last_robot_state;

  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::BumperEvent>> bumper_pub;
  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::CliffEvent>> cliff_pub;
  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::WheelDropEvent>> drop_pub;
  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::ButtonEvent>> button_pub;
  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::PowerSystemEvent>> power_pub;
  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::DigitalInputEvent>> input_pub;
  std::shared_ptr<rclcpp::Publisher<kobuki_msgs::msg::RobotStateEvent>> robot_pub;
};


} // namespace kobuki

#endif /* KOBUKI_BUTTON_EVENT_HPP_ */

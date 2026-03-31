#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "mujoco/mujoco.h" // Added back
#include "rclcpp/rclcpp.hpp"
#include "osc_2_in_interface/msg/osc_mujoco_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "walter_msgs/msg/command.hpp"
#include "walter_msgs/msg/motor_command.hpp"
#include "walter_msgs/msg/wheel_motor_command.hpp"

#include "operational-space-control/walter_sr_v2/aliases.h"
#include "operational-space-control/walter_sr_v2/containers.h"

using namespace operational_space_controller::containers;
using namespace operational_space_controller::aliases;

using rclcpp::Node;
using osc_2_in_interface::msg::OSCMujocoState;
using walter_msgs::msg::Command;
using walter_msgs::msg::MotorCommand;

class OSCNode : public Node {
public:
    OSCNode(const std::string& xml_path);
    ~OSCNode();

    void stop_robot(); 

private:
    void state_callback(const OSCMujocoState::SharedPtr msg);
    void timer_callback();
    
    // Added back for Gravity calculations
    void update_mj_data(const State& current_state); 

    void publish_torque_command(bool safety_override_active_local, 
                                std::chrono::time_point<std::chrono::high_resolution_clock> state_read_time_local,
                                const std::vector<double>& torques);

    rclcpp::Subscription<OSCMujocoState>::SharedPtr state_subscriber_;
    rclcpp::Publisher<Command>::SharedPtr torque_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr data_pub_;    
    
    std_msgs::msg::Float64MultiArray data_msg_; 
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex state_mutex_;

    double last_time_ = 0.0;
    bool safety_override_active_ = false;
    bool is_state_received_ = false;    
    State state_;
    std::chrono::time_point<std::chrono::high_resolution_clock> state_read_time_;

    // Added back for MuJoCo
    mjModel* mj_model_;
    mjData* mj_data_;
    std::string xml_path_; 

    double time_wait_for_execution_ms_ = 0.0;        
    double time_mujoco_update_ms_ = 0.0;
};
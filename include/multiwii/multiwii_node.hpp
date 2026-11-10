#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <msp/FlightController.hpp>

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class MultiWiiNode : public rclcpp::Node {
public:
    explicit MultiWiiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MultiWiiNode() override;

    void setup();

private:
    void setParameterCallback();
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters);
    void onStatus(const msp::msg::Status &status);
    void onImu(const msp::msg::RawImu &raw);
    void onAttitude(const msp::msg::Attitude &attitude);
    void onAltitude(const msp::msg::Altitude &altitude);
    void onRc(const msp::msg::Rc &rc);
    void onServo(const msp::msg::Servo &servo);
    void onMotor(const msp::msg::Motor &motor);
    void onAnalog(const msp::msg::Analog &analog);
    void onSonarAltitude(const msp::msg::SonarAltitude &sonar_altitude);
    void rc_override_AERT1234(const mavros_msgs::msg::OverrideRCIn &rc);
    void rc_override_raw(const mavros_msgs::msg::OverrideRCIn &rc);
    void motor_control(const mavros_msgs::msg::ActuatorControl &motors);

    std::unique_ptr<fcu::FlightController> fcu;
    std::vector<std::string> box_names;
    std::size_t arm_bit = 0;
    std::mutex state_mutex;
    std::chrono::steady_clock::time_point last_status{};
    double status_timeout = 2.0;
    mavros_msgs::msg::State state;
    rclcpp::TimerBase::SharedPtr watchdog;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub;
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;

    float acc_1g;
    float gyro_unit;
    float magn_gain;
    float si_unit_1g;
    std::string tf_base_frame;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magn_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_pub;
    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_in_pub;
    rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr servo_pub;
    rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr motors_pub;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sonar_altitude_pub;

    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_in_sub;
    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_in_sub2;
    rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr motor_control_sub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

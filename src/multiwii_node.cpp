#include "multiwii/multiwii_node.hpp"
#include "multiwii/status.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>
#include <utility>

namespace {
double deg2rad(const double deg) {
    return deg / 180.0 * M_PI;
}

double rad2deg(const double rad) {
    return rad / M_PI * 180.0;
}
}  // namespace

MultiWiiNode::MultiWiiNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("multiwii", options) {
    declare_parameter("device", "/dev/ttyUSB0");
    declare_parameter("baudrate", 115200);
    declare_parameter("standard_gravity", 9.80665);
    declare_parameter("acc_1g", 512.0);
    declare_parameter("gyro_unit", 0.244140625);
    declare_parameter("magn_gain", 0.092);
    declare_parameter("tf_base_frame", "map");
    declare_parameter("MSP_RAW_IMU", 50.0);
    declare_parameter("MSP_ATTITUDE", 50.0);
    declare_parameter("MSP_ALTITUDE", 10.0);
    declare_parameter("MSP_RC", 10.0);
    declare_parameter("MSP_SERVO", 1.0);
    declare_parameter("MSP_MOTOR", 10.0);
    declare_parameter("MSP_ANALOG", 1.0);
    declare_parameter("MSP_STATUS", 5.0);
    declare_parameter("status_timeout", 2.0);
    declare_parameter("MSP_SONAR_ALTITUDE", 0.0);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    fcu = std::make_unique<fcu::FlightController>();

    double std_grav;
    if(get_parameter("standard_gravity", std_grav))
        this->si_unit_1g = std_grav;
    else
        RCLCPP_ERROR(get_logger(), "Parameter 'standard_gravity' not set.");

    double acc_1g;
    if(get_parameter("acc_1g", acc_1g))
        this->acc_1g = acc_1g;
    else
        RCLCPP_ERROR(get_logger(), "Parameter 'acc_1g' not set.");

    double gyro_unit;
    if(get_parameter("gyro_unit", gyro_unit))
        this->gyro_unit = gyro_unit;
    else
        RCLCPP_ERROR(get_logger(), "Parameter 'gyro_unit' not set.");

    double magn_gain;
    if(get_parameter("magn_gain", magn_gain))
        this->magn_gain = magn_gain;
    else
        RCLCPP_ERROR(get_logger(), "Parameter 'magn_gain' not set.");

    get_parameter("tf_base_frame", this->tf_base_frame);
}

MultiWiiNode::~MultiWiiNode() {
    // Stop MSP callbacks before destroying publishers.
    fcu.reset();
}

void MultiWiiNode::setup() {
    std::string device;
    int baudrate = 115200;
    if(get_parameter("device", device)) {
        if(!get_parameter("baudrate", baudrate)) {
            RCLCPP_ERROR(get_logger(), "Parameter 'baudrate' not set. Using default baudrate of %i", baudrate);
        }
        else {
            if(baudrate<=0) {
                RCLCPP_ERROR(get_logger(), "'baudrate' must be positive!");
                throw std::runtime_error("baudrate must be positive");
            }
        }
        RCLCPP_INFO(get_logger(), "Connecting to FCU at %s", device.c_str());
    }
    else {
        RCLCPP_ERROR(get_logger(), "Parameter 'device' not set.");
    }

    if(!fcu->connect(device, static_cast<size_t>(baudrate), 1.0)) {
        throw std::runtime_error("Cannot connect to FCU at " + device);
    }

    const auto variant = fcu->getFwVariant();
    if(variant != msp::FirmwareVariant::MWII && variant != msp::FirmwareVariant::CLFL &&
       variant != msp::FirmwareVariant::BAFL && variant != msp::FirmwareVariant::BTFL) {
        throw std::runtime_error("Supported firmware: MultiWii, Cleanflight, Betaflight, Butterflight");
    }
    msp::msg::BoxNames boxes(variant);
    if(!fcu->sendMessage(boxes, 2.0)) throw std::runtime_error("Cannot read MSP_BOXNAMES");
    box_names = boxes.box_names;
    // Classic MultiWii uses its permanent box IDs as status bit numbers.
    if(variant == msp::FirmwareVariant::MWII) {
        msp::msg::BoxIds ids(variant);
        if(!fcu->sendMessage(ids, 2.0) || ids.box_ids.size() != box_names.size())
            throw std::runtime_error("Invalid MultiWii box IDs");
        std::vector<std::string> indexed(32);
        for(std::size_t i = 0; i < box_names.size(); ++i) {
            if(ids.box_ids[i] < 32) indexed[ids.box_ids[i]] = box_names[i];
        }
        box_names = std::move(indexed);
    }
    arm_bit = multiwii::arm_index(box_names);
    get_parameter("status_timeout", status_timeout);
    if(!std::isfinite(status_timeout) || status_timeout <= 0.0)
        throw std::runtime_error("status_timeout must be finite and positive");
    if(!std::isfinite(acc_1g) || acc_1g <= 0 || !std::isfinite(gyro_unit) ||
       gyro_unit <= 0 || !std::isfinite(magn_gain) || magn_gain <= 0 ||
       !std::isfinite(si_unit_1g) || si_unit_1g <= 0)
        throw std::runtime_error("IMU calibration values must be finite and positive");

    // Late subscribers receive fresh status only.
    const auto state_qos = rclcpp::QoS(1).reliable().durability_volatile();
    armed_pub = create_publisher<std_msgs::msg::Bool>("~/armed", state_qos);
    mode_pub = create_publisher<std_msgs::msg::String>("~/flight_mode", state_qos);
    state_pub = create_publisher<mavros_msgs::msg::State>("~/state", state_qos);
    state.mode = "UNKNOWN";
    watchdog = create_wall_timer(std::chrono::milliseconds(100), [this] {
        std::lock_guard<std::mutex> lock(state_mutex);
        if(last_status.time_since_epoch().count() == 0 ||
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_status).count()
               > status_timeout) {
            state.connected = false;
            state.mode = "UNKNOWN";
            state_pub->publish(state); // stamp/armed remain the last confirmed observation
            std_msgs::msg::String mode;
            mode.data = "UNKNOWN";
            mode_pub->publish(mode);
        }
    });

    imu_pub = create_publisher<sensor_msgs::msg::Imu>("~/imu/data", 1);
    magn_pub = create_publisher<sensor_msgs::msg::MagneticField>("~/imu/mag", 1);
    pose_stamped_pub = create_publisher<geometry_msgs::msg::PoseStamped>("~/local_position/pose", 1);
    rpy_pub = create_publisher<geometry_msgs::msg::Vector3>("~/rpy", 1);
    rc_in_pub = create_publisher<mavros_msgs::msg::RCIn>("~/rc/in", 1);
    servo_pub = create_publisher<mavros_msgs::msg::RCOut>("~/rc/servo", 1);
    motors_pub = create_publisher<mavros_msgs::msg::RCOut>("~/motors", 1);
    battery_pub = create_publisher<sensor_msgs::msg::BatteryState>("~/battery",1);
    heading_pub = create_publisher<std_msgs::msg::Float64>("~/global_position/compass_hdg",1);
    altitude_pub = create_publisher<std_msgs::msg::Float64>("~/global_position/rel_alt",1);
    sonar_altitude_pub = create_publisher<std_msgs::msg::Float64>("~/global_position/sonar_alt", 1);

    rc_in_sub = create_subscription<mavros_msgs::msg::OverrideRCIn>(
        "~/rc/override", 1, std::bind(&MultiWiiNode::rc_override_AERT1234, this, std::placeholders::_1)); // AERT1234
    rc_in_sub2 = create_subscription<mavros_msgs::msg::OverrideRCIn>(
        "~/rc/override/raw", 1, std::bind(&MultiWiiNode::rc_override_raw, this, std::placeholders::_1)); // raw channel order
    motor_control_sub = create_subscription<mavros_msgs::msg::ActuatorControl>(
        "~/actuator_control", 1, std::bind(&MultiWiiNode::motor_control, this, std::placeholders::_1));

    fcu->subscribe(&MultiWiiNode::onStatus, this);
    fcu->subscribe(&MultiWiiNode::onImu, this);
    fcu->subscribe(&MultiWiiNode::onAttitude, this);
    fcu->subscribe(&MultiWiiNode::onAltitude, this);
    fcu->subscribe(&MultiWiiNode::onRc, this);
    fcu->subscribe(&MultiWiiNode::onServo, this);
    fcu->subscribe(&MultiWiiNode::onMotor, this);
    fcu->subscribe(&MultiWiiNode::onAnalog, this);
    fcu->subscribe(&MultiWiiNode::onSonarAltitude, this);
    setParameterCallback();
    RCLCPP_INFO(get_logger(), "MSP ready");
}

void MultiWiiNode::setParameterCallback() {
    const auto initial_parameters = get_parameters({
        "MSP_RAW_IMU", "MSP_ALTITUDE", "MSP_ATTITUDE", "MSP_RC",
        "MSP_SERVO", "MSP_MOTOR", "MSP_ANALOG",
        "MSP_STATUS", "MSP_SONAR_ALTITUDE"});
    const auto result = parameters_callback(initial_parameters);
    if(!result.successful) {
        throw std::runtime_error(result.reason);
    }
    parameter_callback = add_on_set_parameters_callback(
        std::bind(&MultiWiiNode::parameters_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MultiWiiNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
    const std::map<std::string, msp::ID> msp_ids = {
        {"MSP_RAW_IMU", msp::ID::MSP_RAW_IMU},
        {"MSP_ALTITUDE", msp::ID::MSP_ALTITUDE},
        {"MSP_ATTITUDE", msp::ID::MSP_ATTITUDE},
        {"MSP_RC", msp::ID::MSP_RC},
        {"MSP_SERVO", msp::ID::MSP_SERVO},
        {"MSP_MOTOR", msp::ID::MSP_MOTOR},
        {"MSP_ANALOG", msp::ID::MSP_ANALOG},
        {"MSP_STATUS", msp::ID::MSP_STATUS},
        {"MSP_SONAR_ALTITUDE", msp::ID::MSP_SONAR_ALTITUDE},
    };

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // Validate the entire request before changing any subscription.
    for(const auto &parameter : parameters) {
        const auto &name = parameter.get_name();
        if(msp_ids.count(name)) {
            const double maximum = (name == "MSP_RC" || name == "MSP_SERVO" ||
                name == "MSP_ANALOG") ? 100.0 : 1000.0;
            if(parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
               !std::isfinite(parameter.as_double()) ||
               parameter.as_double() < 0.0 || parameter.as_double() > maximum) {
                result.successful = false;
                result.reason = name + " must be a double in [0, " +
                    std::to_string(maximum) + "] Hz";
                return result;
            }
        }
    }

    // Connection and calibration parameters require a restart.
    for(const auto &parameter : parameters) {
        if(parameter.get_name() != "use_sim_time" &&
           !msp_ids.count(parameter.get_name())) {
            result.successful = false;
            result.reason = parameter.get_name() + " is startup-only; restart the node";
            return result;
        }
    }
    for(const auto &parameter : parameters) {
        const auto id = msp_ids.find(parameter.get_name());
        if(id != msp_ids.end() && fcu->hasSubscription(id->second)) {
            const auto subscription = fcu->getSubscription(id->second);
            const double rate = parameter.as_double();
            if(rate == 0.0) {
                subscription->stop();
            }
            else {
                subscription->setTimerFrequency(rate);
            }
        }
    }
    return result;
}

void MultiWiiNode::onStatus(const msp::msg::Status &status) {
    std::lock_guard<std::mutex> lock(state_mutex);
    last_status = std::chrono::steady_clock::now();
    state.header.stamp = now();
    state.connected = true;
    state.armed = status.box_mode_flags.count(arm_bit) != 0;
    state.mode = multiwii::flight_mode(box_names, status.box_mode_flags);
    std_msgs::msg::Bool armed;
    armed.data = state.armed;
    std_msgs::msg::String mode;
    mode.data = state.mode;
    armed_pub->publish(armed);
    mode_pub->publish(mode);
    state_pub->publish(state);
}

void MultiWiiNode::onImu(const msp::msg::RawImu &raw) {
    const msp::msg::ImuSI imu(raw, acc_1g, gyro_unit, magn_gain, si_unit_1g);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now();
    imu_msg.header.frame_id = "multiwii";

    // linear acceleration in m/s²
    imu_msg.linear_acceleration.x = imu.acc[0];
    imu_msg.linear_acceleration.y = imu.acc[1];
    imu_msg.linear_acceleration.z = imu.acc[2];

    // angular velocity in rad/s
    imu_msg.angular_velocity.x = deg2rad(imu.gyro[0]);
    imu_msg.angular_velocity.y = deg2rad(imu.gyro[1]);
    imu_msg.angular_velocity.z = deg2rad(imu.gyro[2]);

    // rotation from direction of acceleration and magnetic field
    const Eigen::Vector3d magn(imu.mag[0], imu.mag[1], imu.mag[2]);
    const Eigen::Vector3d lin_acc(imu.acc[0], imu.acc[1], imu.acc[2]);

    const auto east = lin_acc.cross(magn);
    if(lin_acc.squaredNorm() > 1e-12 && east.squaredNorm() > 1e-12) {
        Eigen::Matrix3d rot;
        rot.col(0) = east.cross(lin_acc).normalized();
        rot.col(1) = east.normalized();
        rot.col(2) = lin_acc.normalized();
        const Eigen::Quaterniond orientation(rot);
        imu_msg.orientation.x = orientation.x();
        imu_msg.orientation.y = orientation.y();
        imu_msg.orientation.z = orientation.z();
        imu_msg.orientation.w = orientation.w();
    } else {
        imu_msg.orientation.w = 1.0;
        imu_msg.orientation_covariance[0] = -1.0;
    }

    imu_pub->publish(imu_msg);

    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = now();

    mag_msg.header.frame_id = "multiwii";
    // sensor_msgs/MagneticField uses tesla
    mag_msg.magnetic_field.x = imu.mag[0] * 1e-6;
    mag_msg.magnetic_field.y = imu.mag[1] * 1e-6;
    mag_msg.magnetic_field.z = imu.mag[2] * 1e-6;
    magn_pub->publish(mag_msg);

    std_msgs::msg::Float64 heading;
    heading.data = rad2deg(std::atan2(double(imu.mag[0]), double(imu.mag[1])));
    heading_pub->publish(heading);
}

void MultiWiiNode::onAttitude(const msp::msg::Attitude &attitude) {
    tf2::Quaternion orientation;
    orientation.setRPY(deg2rad(attitude.roll), deg2rad(attitude.pitch), deg2rad(attitude.yaw));
    orientation.normalize();

    geometry_msgs::msg::TransformStamped multiwii_transform;
    multiwii_transform.header.stamp = now();
    multiwii_transform.header.frame_id = "multiwii_cartesian";
    multiwii_transform.child_frame_id = "multiwii";
    multiwii_transform.transform.rotation.x = orientation.x();
    multiwii_transform.transform.rotation.y = orientation.y();
    multiwii_transform.transform.rotation.z = orientation.z();
    multiwii_transform.transform.rotation.w = orientation.w();

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = multiwii_transform.header;
    pose_stamped.pose.orientation = multiwii_transform.transform.rotation;
    pose_stamped_pub->publish(pose_stamped);
    tf_broadcaster->sendTransform(multiwii_transform);

    geometry_msgs::msg::Vector3 rpy;
    rpy.x = attitude.roll;
    rpy.y = attitude.pitch;
    rpy.z = attitude.yaw;
    rpy_pub->publish(rpy);
}

void MultiWiiNode::onAltitude(const msp::msg::Altitude &altitude) {
    std_msgs::msg::Float64 alt; // altitude in meter
    alt.data = altitude.altitude;
    altitude_pub->publish(alt);

    geometry_msgs::msg::TransformStamped multiwii_transform;
    multiwii_transform.header.stamp = now();
    multiwii_transform.header.frame_id = this->tf_base_frame;
    multiwii_transform.child_frame_id = "multiwii_cartesian";
    multiwii_transform.transform.rotation.w = 1.0;
    multiwii_transform.transform.translation.z = altitude.altitude;
    tf_broadcaster->sendTransform(multiwii_transform);
}

void MultiWiiNode::onRc(const msp::msg::Rc &rc) {
    mavros_msgs::msg::RCIn rc_msg;
    rc_msg.header.stamp = now();
    rc_msg.channels = rc.channels;

    rc_in_pub->publish(rc_msg);
}

void MultiWiiNode::onServo(const msp::msg::Servo &servo) {
    mavros_msgs::msg::RCOut rc;
    for(const uint16_t s : servo.servo) {
        rc.channels.push_back(s);
    }
    servo_pub->publish(rc);
}

void MultiWiiNode::onMotor(const msp::msg::Motor &motor) {
    mavros_msgs::msg::RCOut motor_out;
    for(const uint16_t m : motor.motor) {
        motor_out.channels.push_back(m);
    }
    motors_pub->publish(motor_out);
}

void MultiWiiNode::onAnalog(const msp::msg::Analog &analog) {
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = now();
    battery.voltage = analog.vbat;
    battery.current = analog.amperage;

    battery_pub->publish(battery);
}

void MultiWiiNode::onSonarAltitude(const msp::msg::SonarAltitude &sonar_altitude) {
    std_msgs::msg::Float64 alt;
    alt.data = sonar_altitude.altitude_cm / 100.0;
    sonar_altitude_pub->publish(alt);
}

void MultiWiiNode::rc_override_AERT1234(const mavros_msgs::msg::OverrideRCIn &rc) {
    fcu->setRc(rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3],
               rc.channels[4], rc.channels[5], rc.channels[6], rc.channels[7]);
}

void MultiWiiNode::rc_override_raw(const mavros_msgs::msg::OverrideRCIn &rc) {
    std::vector<uint16_t> channels;
    for(const uint16_t c : rc.channels) { channels.push_back(c); }
    fcu->setRc(channels);
}

void MultiWiiNode::motor_control(const mavros_msgs::msg::ActuatorControl &motors) {
    // Motor PWM = 1000 + abs(control) * 1000.

    std::array<uint16_t,msp::msg::N_MOTOR> motor_values;
    for(size_t i(0); i<motor_values.size(); i++) {
        if(!std::isfinite(motors.controls[i]) || std::abs(motors.controls[i]) > 1.0f) {
            RCLCPP_WARN(get_logger(), "Rejected motor controls outside [-1, 1]");
            return;
        }
        motor_values[i] = static_cast<uint16_t>(1000 + std::abs(motors.controls[i]) * 1000);
    }

    fcu->setMotors(motor_values);
}

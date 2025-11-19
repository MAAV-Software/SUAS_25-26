/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/mission_result.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <stdint.h>
#include <Eigen/Dense>
#include "geodetic_conv.hpp"
#include <cmath>

#include <chrono>
#include <iostream>
#include <fstream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Point {
	double east, north, up;
};

void print_point(Point & p) {
	std::cout << p.east << " " << p.north << " " << p.up << " " << std::endl;
}

const double start_coord_lat = 42.092160;
const double start_coord_long = -83.658057;
const double start_coord_alt = 142;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		//init starting coords (!!! SET TO MILAN PARK !!!)
		
		geodetic_converter_.initialiseReference(start_coord_lat, start_coord_long, start_coord_alt);
		
		// publishers
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		//subscribers
		rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
		qos_profile.best_effort();
		global_position_subscriber_ = this->create_subscription<VehicleGlobalPosition>(
			"/fmu/out/vehicle_global_position", qos_profile,
			std::bind(&OffboardControl::global_position_callback, this, std::placeholders::_1));

		// !!! SETTING FILE PATH TO MILAN FOR PROOF OF FLIGHT TEST !!!
		std::string waypointFilePath = "/home/maav/SUAS_24-25/software_ws/src/waypoint_generation/milan_flight_path.txt";
		waypoints = read_waypoints(waypointFilePath, geodetic_converter_);

		// waypoints used for offboard flight to take pictures
		std::string airdropFilePath = "/home/maav/SUAS_24-25/software_ws/src/waypoint_generation/runway_points.txt";
		airdropRunway = read_waypoints(airdropFilePath, geodetic_converter_);
		
		offboard_setpoint_counter_ = 0;
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Mission mode after 10 setpoints
				// originally set to offboard but we changed it
				// this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				switch_to_offboard();
				arm();

				// Arm the vehicle
				this->arm();
				switched_to_offboard_ = true;
			}

			// offboard_control_mode needs to be pavired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			transition_timer++;
			if (transition_timer >= 200) {
				transition_timer = 0;
			}
			// 7.62 m = 25 ft, (should usually be dist to last waypoint)
			// if (get_dist(waypoints[waypoints.size() - 1]) < 15.24) {
			// 	start_timer = true;
			// 	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// 	// maybe instead of do set mode, we can do this (from reference.cpp)
			// 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 3.0, 0.0);
			// }
			// else if (get_dist(waypoints[waypoints.size() - 1]) < 45) {
			// 	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, 2.0, 0.0);
			// }

			// // set to mission mode after we reach 2nd waypoint again
			// if (start_timer && transition_timer < 101) {
			// 	transition_timer++;
			// }

			// if (transition_timer == 100) {
			// 	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);
			// }

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	// subcription to drone's position
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_position_subscriber_;
    
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	bool start_timer = false;
	int transition_timer = 0;

	geodetic_converter::GeodeticConverter geodetic_converter_;

	std::vector<Point> waypoints;
	std::vector<Point> airdropRunway;
	bool switched_to_offboard_ = false;
    const int target_waypoint_ = 11;
	double current_x_ = 0.0;
	double current_y_ = 0.0;
	double current_z_ = 0.0;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void global_position_callback(const VehicleGlobalPosition::SharedPtr msg);
	void switch_to_offboard();
	double get_dist(Point target);
	std::vector<Point> read_waypoints(const std::string& file_path, geodetic_converter::GeodeticConverter &geodetic_converter_);
};

	
	void OffboardControl::global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
	{
		current_x_ = msg->lat; // Latitude
		current_y_ = msg->lon; // Longitude
		current_z_ = msg->alt; // Altitude
	}

	void OffboardControl::switch_to_offboard() {
			px4_msgs::msg::VehicleCommand msg;
			msg.timestamp = this->now().nanoseconds() / 1000;
			msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
			msg.param1 = 1.0;  // Main mode: Custom
			msg.param2 = 6.0;  // Custom mode: Offboard
			msg.target_system = 1;
			msg.target_component = 1;
			msg.source_system = 1;
			msg.source_component = 1;
			msg.from_external = true;

			vehicle_command_publisher_->publish(msg);
			RCLCPP_INFO(this->get_logger(), "Sent Offboard mode command");
	}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	//if (!switched_to_offboard_) return; // Only send offboard commands after switching

	px4_msgs::msg::OffboardControlMode msg;
	msg.timestamp = this->now().nanoseconds() / 1000;
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it flies in a circle 5m off the ground.
 */
void OffboardControl::publish_trajectory_setpoint()
{
    if (!switched_to_offboard_) return;

    static double t = 0.0;          // time accumulator (seconds)
    const double dt = 0.1;          // timer period = 100 ms
    t += dt;

    // Circle parameters
    const double R = 8.0;           // radius of circle in meters
    const double omega = 0.15;      // angular velocity (rad/sec)
    const double center_x = 0.0;    // center of circle (ENU X)
    const double center_y = 0.0;    // center of circle (ENU Y)
    const double altitude = -5.0;   // PX4 uses negative for up (NED Z)

    // Parametric circle position
    double x = center_x + R * cos(omega * t);
    double y = center_y + R * sin(omega * t);

    TrajectorySetpoint msg{};
    msg.timestamp = this->now().nanoseconds() / 1000;

    msg.position = { (float)x, (float)y, (float)altitude };

    // Optional: rotate yaw tangentially
    msg.yaw = atan2(y - center_y, x - center_x);

    // No velocity setpoint (we use position only)
    msg.velocity = {NAN, NAN, NAN};

    trajectory_setpoint_publisher_->publish(msg);
}



/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

// gets distance to target from drone's current position
double OffboardControl::get_dist(Point target) {
	// Have to convert pos to ENU to get dist in meters
	double pos_x, pos_y, pos_z;
	geodetic_converter_.geodetic2Enu(current_x_, current_y_, current_z_, &pos_y, &pos_x, &pos_z);

	// for some reason we need to invert current_z_
	// probably due to NED shenanigans
	double distance_from_target = sqrt(pow((pos_x- target.east), 2) +
									   pow((pos_y - target.north), 2) +
									   pow((-current_z_ - target.up), 2));

	return distance_from_target;
}

// reads in waypoints.txt from waypoint_generation
std::vector<Point> OffboardControl::read_waypoints(const std::string& file_path, geodetic_converter::GeodeticConverter &geodetic_converter_) {
    std::ifstream file(file_path);
    std::vector<Point> waypoints;

    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Point waypoint;
            double lat, lon, alt_ft;
            iss >> lat >> lon >> alt_ft;
			double east, north, up;

			geodetic_converter_.geodetic2Enu(lat, lon, alt_ft*.3048, &north, &east, &up);
            waypoint.east = east;
			waypoint.north = north;
			// trying to give it the original altitude, we think we can just keep it this way
			waypoint.up = alt_ft*.3048*(-1);	
			std::cout << "Added new point: " << waypoint.east << " " << waypoint.north << " " << waypoint.up << std::endl;
            waypoints.push_back(waypoint);
        }	
        file.close();
    } else {
        //RCLCPP_ERROR(this->get_logger(), "Unable to open waypoints file");
    }

    return waypoints;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}

/**
 * \copyright TaCo - Task-Space Control Library.<br> Copyright (c) 2015-2016
 *
 * TaCo is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * TaCo is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the Lesser GNU Lesser General Public
 * License along with TaCo.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Brian Soe <bsoe@stanford.edu>
 * Author: Mikael Jorda <mjorda@stanford.edu>
 * Author: Toki Migimatsu <takatoki@stanford.edu>
 */

#include "RedisDriver.h"
#include "ButterworthFilter.h"
#include "redis/RedisClient.h"

#include <friLBRClient.h>
#include <friUdpConnection.h>
#include <friClientApplication.h>
#include <tinyxml2.h>

#include <string>
#include <iostream>
#include <sstream>


/**********************
 * Driver Application *
 **********************/

int main (int argc, char** argv)
{
	// Usage
	if (argc < 3) {
		std::cout << "Usage: kuka_iiwa_driver [-s KUKA_IIWA_IP] [-p KUKA_IIWA_PORT]" << std::endl
		          << "                        [-rs REDIS_SERVER_IP] [-rp REDIS_SERVER_PORT]" << std::endl
		          << std::endl
		          << "This driver provides a Redis interface for communication with the Kuka IIWA." << std::endl
		          << std::endl
		          << "Optional arguments:" << std::endl
		          << "  -s KUKA_IIWA_IP" << std::endl
		          << "\t\t\t\tKuka IIWA server IP (inferred by default)." << std::endl
		          << "  -p KUKA_IIWA_PORT" << std::endl
		          << "\t\t\t\tKuka IIWA server port (default " << KukaIIWA::DEFAULT_PORT << ")." << std::endl
		          << "  -rs REDIS_SERVER_IP" << std::endl
		          << "\t\t\t\tRedis server IP (default " << RedisServer::DEFAULT_IP << ")." << std::endl
		          << "  -rp REDIS_SERVER_PORT" << std::endl
		          << "\t\t\t\tRedis server port (default " << RedisServer::DEFAULT_PORT << ")." << std::endl
		          << std::endl;
	}

	// Parse arguments
	char *kuka_iiwa_ip = nullptr;
	int kuka_iiwa_port = KukaIIWA::DEFAULT_PORT;
	std::string redis_ip = RedisServer::DEFAULT_IP;
	int redis_port = RedisServer::DEFAULT_PORT;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-s")) {
			// Kuka IIWA server IP
			kuka_iiwa_ip = argv[++i];
		} else if (!strcmp(argv[i], "-p")) {
			// Kuka IIWA server port
			sscanf(argv[++i], "%d", &kuka_iiwa_port);
		} else if (!strcmp(argv[i], "-rs")) {
			// Redis server IP
			redis_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-rp")) {
			// Redis server port
			sscanf(argv[++i], "%d", &redis_port);
		}
	}

	// Create new client and UDP connection
	KUKA::FRI::RedisDriver client(redis_ip, redis_port);
	KUKA::FRI::UdpConnection connection;

	// Connect client application to KUKA Sunrise controller.
	// Parameter NULL means: repeat to the address, which sends the data
	KUKA::FRI::ClientApplication app(connection, client);
	app.connect(kuka_iiwa_port, kuka_iiwa_ip);

	// Execution loop: call the step routine to receive and process FRI packets
	bool success = true;
	while (success) {
		success = app.step();
	}

	// Disconnect from controller
	app.disconnect();

	return 0;
}


/************************************
 * RedisDriver Class Implementation *
 ************************************/

using namespace KukaIIWA;

// Returns the value of elapsed time as a double
static double elapsedTime(timespec start, timespec end) {
	return (end.tv_sec - start.tv_sec) + 1e-9*(end.tv_nsec - start.tv_nsec);
}

// Print a message with the IIWA prefix
static void printMessage(const std::string& message) {
	std::cout << "IIWA Redis Driver : " << message << std::endl;
}

// Print out the specified command mode
static void printCommandMode(KUKA::FRI::EClientCommandMode command_mode) {
	switch (command_mode) {
		case KUKA::FRI::NO_COMMAND_MODE:
			printMessage("FRI command mode is [NO_COMMAND_MODE]\n");
			break;
		case KUKA::FRI::POSITION:
			printMessage("FRI command mode is [POSITION]\n");
			break;
		case KUKA::FRI::WRENCH:
			printMessage("FRI command mode is [WRENCH]\n");
			break;
		case KUKA::FRI::TORQUE:
			printMessage("FRI command mode is [TORQUE]\n");
			break;
	}
}


namespace KUKA {
namespace FRI {

RedisDriver::RedisDriver(const std::string& redis_ip, const int redis_port)
#ifdef USE_KUKA_LBR_DYNAMICS
	: dynamics_(kuka::Robot::LBRiiwa)
#endif
{
	// Set up velocity filter
	velocity_filter_.setDimension(LBRState::NUMBER_OF_JOINTS);
	velocity_filter_.setCutoffFrequency(kCutoffFreq);

	// Connect to Redis server
	redis_.connect(redis_ip, redis_port);

#ifdef USE_KUKA_LBR_DYNAMICS
	// Parse tool from tool.xml
	parseTool();

	// Parse rbdl model from urdf
    bool success = RigidBodyDynamics::Addons::URDFReadFromFile(kModelFilename, &rbdl_model_, false, false);
    if (!success) {
		std::cout << "Error loading model [" << kModelFilename << "]" << std::endl;
		exit(1);
    }
#endif
}


void RedisDriver::onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
{
	LBRClient::onStateChange(oldState, newState);
	// react on state change events
	switch (newState) {
		case KUKA::FRI::IDLE:
			printMessage("FRI switched to state [IDLE]\n");
			break;
		case KUKA::FRI::MONITORING_WAIT:
			printMessage("FRI switched to state [MONITORING_WAIT]\n");
			break;
		case KUKA::FRI::MONITORING_READY:
			t_sample_ = robotState().getSampleTime();
			printMessage("FRI switched to state [MONITORING_READY]\n");
			break;
		case KUKA::FRI::COMMANDING_WAIT:
			printMessage("FRI switched to state [COMMANDING_WAIT]\n");
			break;
		case KUKA::FRI::COMMANDING_ACTIVE:
			printMessage("FRI switched to state [COMMANDING_ACTIVE]\n");
			break;
		default:
			printMessage("FRI switched to state [UNKNOWN]\n");
			break;
	}
}


void RedisDriver::waitForCommand()
{
	// In waitForCommand(), the joint values have to be mirrored. Which is done,
	// by calling the base method.
	LBRClient::waitForCommand();

	// If we want to command torques, we have to command them all the time; even
	// in waitForCommand(). This has to be done due to consistency checks. In
	// this state it is only necessary, that some torque values are sent. The
	// LBR does not take the specific value into account.
	if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
		robotCommand().setJointPosition(robotState().getIpoJointPosition());
		robotCommand().setTorque(arr_command_torques_);
	}
}


void RedisDriver::command()
{
	// In command(), the joint values have to be sent. Which is done by calling
	// the base method.
	LBRClient::command();

	// check for control mode change
	KUKA::FRI::EClientCommandMode fri_command_mode_next = robotState().getClientCommandMode();
	if (fri_command_mode_next != fri_command_mode_) {
		fri_command_mode_ = fri_command_mode_next;
		printCommandMode(fri_command_mode_);
	}

	// get the position and measure torque
	memcpy(arr_q_, robotState().getMeasuredJointPosition(), DOF * sizeof(double));
	memcpy(arr_sensor_torques_, robotState().getMeasuredTorque(), DOF * sizeof(double));

	// get the time
	timespec time;
	time.tv_sec = robotState().getTimestampSec();
	time.tv_nsec = robotState().getTimestampNanoSec();

	// get the velocity
	if (t_prev_.tv_sec == 0 && t_prev_.tv_nsec == 0) {
		// if not initialized, set to zero
		dq_.setZero();
		dq_filtered_.setZero();
	} else {
		// velocity = (position - last position)/(time - last time)
		double dt = elapsedTime(t_prev_, time);
		dq_ = (q_ - q_prev_) / dt;
		dq_filtered_ = velocity_filter_.update(dq_);
	}

	// Send positions, velocities and sensed torques
	redis_.pipeset({
		{KEY_JOINT_POSITIONS,  RedisClient::encodeEigenMatrix(q_)},
		{KEY_JOINT_VELOCITIES, RedisClient::encodeEigenMatrix(dq_filtered_)},
		{KEY_SENSOR_TORQUES,   RedisClient::encodeEigenMatrix(sensor_torques_)}
	});

	// Get commanded torques or joint positions
	try {
		if (fri_command_mode_ == KUKA::FRI::TORQUE) {
			command_torques_ = redis_.getEigenMatrix(KEY_COMMAND_TORQUES);
		} else if (fri_command_mode_ == KUKA::FRI::POSITION) {
			q_des_ = redis_.getEigenMatrix(KEY_DESIRED_JOINT_POSITIONS);
		}
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl
		          << "Going to fail mode." << std::endl;
		exit_program_ = true;
	}

	if (exit_program_) {
		if (exit_counter_ <= 0) exit(0);
		command_torques_ = (-kExitKv * dq_filtered_.array()).matrix();
		exit_counter_--;
	}

	// ADD GRAVITY COMPENSATION FOR TOOL
#ifdef USE_KUKA_LBR_DYNAMICS
	if (fri_command_mode_ == KUKA::FRI::TORQUE) {
		// Find tool Jacobian
		Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, DOF);
		dynamics_.getJacobian(J, q_, tool_com_, DOF);

		// Find tool weight
		Eigen::VectorXd F_grav_tool = Eigen::MatrixXd::Zero(6);
		F_grav_tool(2) = -9.81 * tool_mass_;

		// Compensate for tool weight
		command_torques_ -= J.transpose() * F_grav_tool;
	}
#endif

	// COMPENSATE FOR THE OFFSET IN THE FIRST JOINT
	if (fri_command_mode_ == KUKA::FRI::TORQUE) {
		command_torques_(0) -= 0.3;
	}

	// CHECK COMMAND
	switch (fri_command_mode_) {
		case KUKA::FRI::NO_COMMAND_MODE:
			break;

		case KUKA::FRI::POSITION:
			// Joint limits
			if ((q_des_.array().abs() < JOINT_LIMITS).any()) {
				std::cout << "WARNING : q_des ["
				          << q_des_.transpose() << "] exceeds limits ["
				          << JOINT_LIMITS.transpose() << "]." << std::endl;
				q_des_ = q_des_.array().min(JOINT_LIMITS).max(-JOINT_LIMITS).matrix();
			}
			break;

		case KUKA::FRI::WRENCH:
			printMessage("Wrench control mode not supported by the driver");
			break;

		case KUKA::FRI::TORQUE:

			// ********************************************************************
			// Torque mode safeties here
			// ********************************************************************

			// Torque saturation
			if ((command_torques_.array().abs() < TORQUE_LIMITS).any()) {
				std::cout << "WARNING : command torques ["
				          << command_torques_.transpose() << "] exceeds limits ["
				          << TORQUE_LIMITS.transpose() << "]." << std::endl;
				command_torques_ = command_torques_.array().min(TORQUE_LIMITS).max(-TORQUE_LIMITS).matrix();
			}

			// Jerk saturation
			Eigen::MatrixXd jerk = command_torques_ - command_torques_prev_;
			if ((jerk.array().abs() > JERK_LIMITS).any()) {
				std::cout << "WARNING : jerk ["
				          << jerk.transpose() << "] exceeds limits ["
				          << JERK_LIMITS.transpose() << "]." << std::endl;
				jerk = jerk.array().min(JERK_LIMITS).max(-JERK_LIMITS).matrix();
				command_torques_ = command_torques_prev_ + jerk;
			}

			// Velocity limits
			if ((dq_.array().abs() < VELOCITY_LIMITS).any()) {
				std::cout << "ERROR : dq ["
				          << dq_.transpose() << "] exceeds limits ["
				          << VELOCITY_LIMITS.transpose() << "]." << std::endl
				          << "Going to fail mode." << std::endl;
				exit_program_ = true;
			}

			// Joint limits
			if ((q_.array().abs() < JOINT_LIMITS).any()) {
				std::cout << "ERROR : q ["
				          << q_.transpose() << "] exceeds limits ["
				          << JOINT_LIMITS.transpose() << "]." << std::endl
				          << "Going to fail mode." << std::endl;
				exit_program_ = true;
			}

#ifdef USE_KUKA_LBR_DYNAMICS
			// Wrist height limits
			Eigen::Vector3d pos_wrist = CalcBodyToBaseCoordinates(rbdl_model_, q_, 6, Eigen::Vector3d::Zero(), true);
			if (pos_wrist(2) <= POS_WRIST_LIMITS[0]) {
				std::cout << "ERROR : wrist height ["
				          << pos_wrist(2) << "] exceeds lower limit ["
				          << POS_WRIST_LIMITS[0] << "]." << std::endl
				          << "Going to fail mode." << std::endl;
				exit_program_ = true;
			} else if (pos_wrist(2) >= POS_WRIST_LIMITS[1]) {
				std::cout << "ERROR : wrist height ["
				          << pos_wrist(2) << "] exceeds upper limit ["
				          << POS_WRIST_LIMITS[1] << "]." << std::endl
				          << "Going to fail mode." << std::endl;
				exit_program_ = true;
			}
#endif
			break;
	}

	// SEND COMMAND
	switch (fri_command_mode_) {
		case KUKA::FRI::NO_COMMAND_MODE:
			break;

		case KUKA::FRI::POSITION:
			robotCommand().setJointPosition(arr_q_des_);
			break;

		case KUKA::FRI::WRENCH:
			printMessage("WARNING : Kuka FRI Wrench control mode has not been implemented in driver");
			break;

		case KUKA::FRI::TORQUE:
			memcpy(arr_dither_, robotState().getMeasuredJointPosition(), DOF * sizeof(double));
			for (int i = 0; i < DOF; i++) {
				arr_dither_[i] += 0.1/180.0*M_PI * sin(2*M_PI*5*t_curr_);
			}

			// need to command the position even in torque mode in order to activate the friction compensation
			robotCommand().setJointPosition(arr_dither_);
			robotCommand().setTorque(arr_command_torques_);
			break;

		default:
			printMessage("WARNING : Kuka FRI control mode unrecognized");
			break;
	}

	// Update the previous values
	t_curr_ += t_sample_;
	t_prev_ = time;

	q_prev_ = q_;
	command_torques_prev_ = command_torques_;

	num_iters_++;
}


#ifdef USE_KUKA_LBR_DYNAMICS

void RedisDriver::parseTool()
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(kToolFilename);
	if (!doc.Error()) {
		printMessage("Loading tool file ["+std::string(kToolFilename)+"].");
		try {
			std::string mass = doc.
				FirstChildElement("tool")->
				FirstChildElement("inertial")->
				FirstChildElement("mass")->
				Attribute("value");
			tool_mass_ = std::stod(mass);
			printMessage("Tool mass: "+mass);

			std::stringstream com(doc.
				FirstChildElement("tool")->
				FirstChildElement("inertial")->
				FirstChildElement("origin")->
				Attribute("xyz"));
			com >> tool_com_(0);
			com >> tool_com_(1);
			com >> tool_com_(2);
			std::stringstream ss; ss << tool_com_.transpose();
			printMessage("Tool CoM : "+ss.str());
		}
		catch (const std::exception& e) {
			std::cout << e.what(); // information from length_error printed
			printMessage("WARNING : Failed to parse tool file.");
		}
	} else {
		printMessage("WARNING : Could not load tool file ["+std::string(kToolFilename)+"]");
		doc.PrintError();
	}
}

#endif  // USE_KUKA_LBR_DYNAMICS


} //namespace FRI
} //namespace KUKA

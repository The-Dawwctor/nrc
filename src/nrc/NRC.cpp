#include "NRC.h"

#include <iostream>
#include <fstream>

#include <signal.h>

static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

/**
 * readPointValues()
 * ------------------------------
 * Retrieve all point key values from Redis with pub/sub
 */
void readPointValues(redisAsyncContext* ac, void* reply, void* privdata) {
    if (reply == NULL) return;
    redisReply* r = (redisReply*)reply;

    if (r->type == REDIS_REPLY_ARRAY && string(r->element[0]->str) == "message") {
        // Push to redis since asynchronous can't communicate with synchronous
        *(int*)privdata = 3; // Set controller state to OP_SPACE_GOAL_SET
        RedisClient update_;
        update_.connect("127.0.0.1", 6379);
        update_.set("trajectory", r->element[2]->str);
    }
}

/**
 * NRC::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void NRC::readRedisValues() {
	// Read from Redis current sensor values
	robot->_q = redis_.getEigenMatrix(KEY_JOINT_POSITIONS);
	robot->_dq = redis_.getEigenMatrix(KEY_JOINT_VELOCITIES);

	// // Get current simulation timestamp from Redis
	t_curr_ = stod(redis_.get(KEY_TIMESTAMP));

	// // Read in KP and KV from Redis (can be changed on the fly in Redis)
	kp_pos_ = stoi(redis_.get(KEY_KP_POSITION));
	kv_pos_ = stoi(redis_.get(KEY_KV_POSITION));
	kp_ori_ = stoi(redis_.get(KEY_KP_ORIENTATION));
	kv_ori_ = stoi(redis_.get(KEY_KV_ORIENTATION));
	kp_joint_ = stoi(redis_.get(KEY_KP_JOINT));
	kv_joint_ = stoi(redis_.get(KEY_KV_JOINT));

	// Read frames from OptiTrackClient
    if (!optitrack_.getFrame()) return;

    cout << "Timestamp: " << optitrack_.frameTime() << endl;
    cout << "Rigid body positions:" << endl;
    for (auto& pos : optitrack_.pos_rigid_bodies_) {
        cout << '\t' << pos.transpose() << endl;
    }
    cout << "Marker positions:" << endl;
    for (auto& pos : optitrack_.pos_single_markers_) {
        cout << '\t' << pos.transpose() << endl;
    }
    cout << endl;
}

/**
 * NRC::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void NRC::writeRedisValues() {
	// Send end effector position and desired position
	redis_.setEigenMatrix(KEY_EE_POS, x_);
	redis_.setEigenMatrix(KEY_EE_POS_DES, x_des_);
    redis_.setEigenMatrix(KEY_OBS_POS, Eigen::Vector3d::Zero());

	// Send torques
    redis_.setEigenMatrix(KEY_COMMAND_TORQUES, command_torques_);
}

/**
 * NRC::updateModel()
 * --------------------------
 * Update the robot model and all the relevant member variables.
 */
void NRC::updateModel() {
	// Update the model
	robot->updateModel();

	// Forward kinematics
	robot->position(x_, "link6", Eigen::Vector3d::Zero());
	robot->linearVelocity(dx_, "link6", Eigen::Vector3d::Zero());

	// Jacobians
	robot->Jv(Jv_, "link6", Eigen::Vector3d::Zero());
	robot->nullspaceMatrix(N_, Jv_);

	// Dynamics
	robot->taskInertiaMatrixWithPseudoInv(Lambda_x_, Jv_);
	robot->gravityVector(g_);
}

/**
 * NRC::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
NRC::ControllerStatus NRC::computeJointSpaceControlTorques() {
	// Finish if the robot has converged to q_initial
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	if (q_err.norm() < kToleranceInitQ && dq_err.norm() < kToleranceInitDq) {
		return FINISHED;
	}

	// Compute torques
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;
	command_torques_ = robot->_M * ddq + g_;
	return RUNNING;
}

/**
 * NRC::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
NRC::ControllerStatus NRC::computeOperationalSpaceControlTorques() {
	// PD position control with velocity saturation
    Eigen::Vector3d x_err = x_ - x_des_;
    dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
    double v = kMaxVelocity / dx_des_.norm();
    if (v > 1) v = 1;
    Eigen::Vector3d dx_err = dx_ - v * dx_des_;
    Eigen::Vector3d ddx = -kv_pos_ * dx_err;

    // Reached desired goal position and velocity; need next goal
    if (!completed && x_err.norm() < kToleranceX && dx_err.norm() < kToleranceDx) {
        redis_.command("PUBLISH nrc-next-goal next");
        completed = true;
    }

	// Nullspace posture control and damping
    Eigen::VectorXd q_err = robot->_q - q_des_;
    Eigen::VectorXd dq_err = robot->_dq - dq_des_;
    Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

	// Control torques
    Eigen::Vector3d F_x = Lambda_x_ * ddx;
    Eigen::VectorXd F_posture = robot->_M * ddq;
    command_torques_ = Jv_.transpose() * F_x + N_.transpose() * F_posture + g_;

    return RUNNING;
}

/**
 * NRC::setOperationalSpaceGoals()
 * ----------------------------------------------
 * Sets operational space goals whenever new information received from redis subscription.
 */
NRC::ControllerStatus NRC::setOperationalSpaceGoals() {
    completed = false;
    x_des_ = redis_.getEigenMatrix("trajectory");
}

/**
 * public NRC::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void NRC::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	redis_.connect(kRedisHostname, kRedisPort);

    // Start asynchronous redis client for pub/sub
    sub_ = redisAsyncConnect(kRedisHostname.c_str(), kRedisPort);
    if (sub_->err) {
        cout << "Error: " << sub_->errstr << endl;
        exit(1);
    }
    redisAsyncCommand(sub_, readPointValues, &controller_state_, "SUBSCRIBE nrc-trajectory");
    redisAsyncHandleWrite(sub_);

	// Set up optitrack
	optitrack_.openConnection("123.45.67.89");

	// Set gains in Redis if not initialized
    redis_.set(KEY_KP_POSITION, to_string(kp_pos_));
    redis_.set(KEY_KV_POSITION, to_string(kv_pos_));
    redis_.set(KEY_KP_ORIENTATION, to_string(kp_ori_));
    redis_.set(KEY_KV_ORIENTATION, to_string(kv_ori_));
    redis_.set(KEY_KP_JOINT, to_string(kp_joint_));
    redis_.set(KEY_KV_JOINT, to_string(kv_joint_));
    redis_.setEigenMatrix(KEY_EE_POS_DES, x_des_);
}

/**
 * public NRC::runLoop()
 * -----------------------------
 * NRC state machine
 */
void NRC::runLoop() {
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer_.waitForNextLoop();
		++controller_counter_;

        redisAsyncHandleRead(sub_);

		// Get latest sensor values from Redis and update robot model
        try {
            readRedisValues();
        } catch (std::exception& e) {
            if (controller_state_ != REDIS_SYNCHRONIZATION) {
                std::cout << e.what() << " Aborting..." << std::endl;
                break;
            }
            std::cout << e.what() << " Waiting..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        updateModel();

        switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
            case REDIS_SYNCHRONIZATION:
            if (isnan(robot->_q)) continue;
            cout << "Redis synchronized. Switching to joint space controller." << endl;
            controller_state_ = JOINT_SPACE_INITIALIZATION;
            break;

			// Initialize robot to default joint configuration
            case JOINT_SPACE_INITIALIZATION:
            if (computeJointSpaceControlTorques() == FINISHED) {
                cout << "Joint position initialized. Switching to operational space controller." << endl;
                controller_state_ = OP_SPACE_POSITION_CONTROL;
            }
            break;

			// Control end effector to desired position
            case OP_SPACE_POSITION_CONTROL:
            if (computeOperationalSpaceControlTorques() == FINISHED) {
                controller_state_ = OP_SPACE_GOAL_SET;
            }
            break;

            // Set desired end effector goal position
            case OP_SPACE_GOAL_SET:
            setOperationalSpaceGoals();
            controller_state_ = OP_SPACE_POSITION_CONTROL;
            break;

			// Invalid state. Zero torques and exit program.
            default:
            cout << "Invalid controller state. Stopping controller." << endl;
            g_runloop = false;
            command_torques_.setZero();
            break;
        }

		// Check command torques before sending them
        if (isnan(command_torques_)) {
            cout << "NaN command torques. Sending zero torques to robot." << endl;
            command_torques_.setZero();
        }

		// Send command torques
        writeRedisValues();
    }

	// Zero out torques before quitting
    command_torques_.setZero();
    redis_.setEigenMatrix(KEY_COMMAND_TORQUES, command_torques_);
}

int main(int argc, char** argv) {

	// Parse command line
	if (argc != 4) {
		cout << "Usage: ./nrc <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
		exit(0);
	}
	// Argument 0: executable name
	// Argument 1: <path-to-world.urdf>
	string world_file(argv[1]);
	// Argument 2: <path-to-robot.urdf>
	string robot_file(argv[2]);
	// Argument 3: <robot-name>
	string robot_name(argv[3]);

	// Set up signal handler
	signal(SIGABRT, &stop);
	signal(SIGTERM, &stop);
	signal(SIGINT, &stop);

	// Load robot
	cout << "Loading robot: " << robot_file << endl;
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// Start controller app
	cout << "Initializing app with " << robot_name << endl;
	NRC app(move(robot), robot_name);
	app.initialize();
	cout << "App initialized. Waiting for Redis synchronization." << endl;
	app.runLoop();

	return 0;
}



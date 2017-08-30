#ifndef NRC_H
#define NRC_H

// CS225a
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "optitrack/OptiTrackClient.h"

// Standard
#include <string>
#include <thread>

// External
#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <hiredis/async.h>
#include <model/ModelInterface.h>

class NRC {

public:
    NRC(std::shared_ptr<Model::ModelInterface> robot,
      const std::string &robot_name) :
    robot(robot),
    dof(robot->dof()),
    KEY_COMMAND_TORQUES (RedisServer::KEY_PREFIX + robot_name + "::actuators::fgc"),
    KEY_EE_POS          (RedisServer::KEY_PREFIX + robot_name + "::tasks::ee_pos"),
    KEY_EE_POS_DES      (RedisServer::KEY_PREFIX + robot_name + "::tasks::ee_pos_des"),
    KEY_JOINT_POSITIONS (RedisServer::KEY_PREFIX + robot_name + "::sensors::q"),
    KEY_JOINT_VELOCITIES(RedisServer::KEY_PREFIX + robot_name + "::sensors::dq"),
    KEY_TIMESTAMP       (RedisServer::KEY_PREFIX + robot_name + "::timestamp"),
    KEY_KP_POSITION     (RedisServer::KEY_PREFIX + robot_name + "::tasks::kp_pos"),
    KEY_KV_POSITION     (RedisServer::KEY_PREFIX + robot_name + "::tasks::kv_pos"),
    KEY_KP_ORIENTATION  (RedisServer::KEY_PREFIX + robot_name + "::tasks::kp_ori"),
    KEY_KV_ORIENTATION  (RedisServer::KEY_PREFIX + robot_name + "::tasks::kv_ori"),
    KEY_KP_JOINT        (RedisServer::KEY_PREFIX + robot_name + "::tasks::kp_joint"),
    KEY_KV_JOINT        (RedisServer::KEY_PREFIX + robot_name + "::tasks::kv_joint"),
    KEY_OBS_POS         (RedisServer::KEY_PREFIX + robot_name + "::tasks::obs_pos"),
    command_torques_(dof),
    Jv_(3, dof),
    N_(dof, dof),
    Lambda_x_(3, 3),
    g_(dof),
    q_des_(dof),
    dq_des_(dof),
    controller_state_(REDIS_SYNCHRONIZATION)
    {
        command_torques_.setZero();

        // Home configuration for Kinova Jaco
        q_des_ << 1.672561, 0.479626, 1.945289, 0.625663, 1.696683, 0.25158;
        dq_des_.setZero();

        // Desired end effector position
        x_des_ << 0.3, 0, 0.3;
        dx_des_.setZero();
    }

	/***** Public functions *****/

    void initialize();
    void runLoop();

protected:
	/***** Enums *****/

	// State enum for controller state machine inside runloop()
	enum ControllerState {
		REDIS_SYNCHRONIZATION,
		JOINT_SPACE_INITIALIZATION,
		OP_SPACE_POSITION_CONTROL,
        OP_SPACE_GOAL_SET
    };

	// Return values from computeControlTorques() methods
    enum ControllerStatus {
		RUNNING,  // Not yet converged to goal position
		FINISHED  // Converged to goal position
	};

	/***** Constants *****/

	const int dof;  // Initialized with robot model
	const double kToleranceInitQ  = 0.1;  // Joint space initialization tolerance
	const double kToleranceInitDq = 0.1;  // Joint space initialization tolerance
    const double kToleranceX = 0.1;  // Operational space goal tolerance
    const double kToleranceDx = 0.1;  // Operational space goal tolerance
	const double kMaxVelocity = 0.5;  // Maximum end effector velocity

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const std::string kRedisHostname = "127.0.0.1";
	const int kRedisPort = 6379;

	// Redis keys:
	// - write:
    const std::string KEY_COMMAND_TORQUES;
    const std::string KEY_EE_POS;
    const std::string KEY_EE_POS_DES;
	// - read:
    const std::string KEY_JOINT_POSITIONS;
    const std::string KEY_JOINT_VELOCITIES;
    const std::string KEY_TIMESTAMP;
    const std::string KEY_KP_POSITION;
    const std::string KEY_KV_POSITION;
    const std::string KEY_KP_ORIENTATION;
    const std::string KEY_KV_ORIENTATION;
    const std::string KEY_KP_JOINT;
    const std::string KEY_KV_JOINT;
    // - write obstacle info:
    const std::string KEY_OBS_POS;

    // Miscellaneous constants
    const double AVOID_THRESHOLD = 0.2;
    const std::string EE_LINK_NAME = "link5";

	/***** Member functions *****/

    void readRedisValues();
    void updateModel();
    void writeRedisValues();
    ControllerStatus computeJointSpaceControlTorques();
    ControllerStatus computeOperationalSpaceControlTorques();
    ControllerStatus setOperationalSpaceGoals();

	/***** Member variables *****/

	// Robot
    const std::shared_ptr<Model::ModelInterface> robot;

	// Redis
    RedisClient redis_;
    redisAsyncContext* sub_;

	// Timer
    LoopTimer timer_;
    double t_curr_;
    uint64_t controller_counter_ = 0;

	// OptiTrack
    OptiTrackClient optitrack_;

	// State machine
    ControllerState controller_state_;
    // Ensures next goal not given until current goal reached
    bool completed = false;

	// Controller variables
    Eigen::VectorXd command_torques_;
    Eigen::MatrixXd Jv_;
    Eigen::MatrixXd N_;
    Eigen::MatrixXd Lambda_x_;
    Eigen::VectorXd g_;
    Eigen::Vector3d x_, dx_;
    Eigen::VectorXd q_des_, dq_des_;
    Eigen::Vector3d x_des_, dx_des_;

    // Obstacle Avoidance
    std::vector<Eigen::Vector3d> obstacles;

	// Default gains (used only when keys are nonexistent in Redis)
    double kp_pos_ = 40;
    double kv_pos_ = 10;
    double kp_ori_ = 40;
    double kv_ori_ = 10;
    double kp_joint_ = 40;
    double kv_joint_ = 10;
};

#endif  // NRC_H

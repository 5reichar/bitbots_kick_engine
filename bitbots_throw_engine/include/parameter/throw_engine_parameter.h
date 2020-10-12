#ifndef BITBOTS_THROW_THROW_ENGINE_PARAMETER_H
#define BITBOTS_THROW_THROW_ENGINE_PARAMETER_H

#include <bitbots_throw/throw_engine_paramsConfig.h>

namespace bitbots_throw{
	struct RobotAndWorldParameter{
        // value used as gravity for calculations
        double gravity_;
        // value used as pie for calculations
        double pi_;
        // The height of the robots head, measure from the shoulders
        double head_height_;
        // The height of the robots trunk
        double trunk_height_;
        // The lenght of the legs of the robot
        double leg_length_;
        // The length of the arms of the robot
        double arm_length_;
        // The maximal stall torque the robot arm motor has (in Nm, >= 0)
        double arm_max_stall_torque_;
        // The recommended fraction of the stall torque the robot arm motor should use
        double arm_stall_torque_usage_;
        // the radius of the ball
        double ball_radius_;
        // the weight of the ball
        double ball_weight_;
        // the safety distance between the botton of the trunk and the feet of the robot
        double squat_safety_distance_;

		//////		Constructor
		RobotAndWorldParameter(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            gravity_ = config.gravity;
            pi_ = config.pi;
            head_height_ = config.head_height;
            trunk_height_ = config.trunk_height;
            leg_length_ = config.leg_length;
            arm_length_ = config.arm_length;
            arm_max_stall_torque_ = config.arm_max_stall_torque;
            arm_stall_torque_usage_ = config.arm_stall_torque_usage;
            ball_radius_ = config.ball_radius;
            ball_weight_ = config.ball_weight;
            squat_safety_distance_ = config.squat_safety_distance;
		}
	};

    struct ThrowEngineParameter{
        // Max frequency of engine update rate [hz]
        double engine_frequency_;
        // Publish odom every [int] update of the engine
        int odom_publish_factor_;
        // Timeout time for bioIK [s]
        double bio_ik_time_;
        // Show if the code is run in a simulation or on the robot
        bool simulation_active_;

        //////		Constructor
        ThrowEngineParameter(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            engine_frequency_ = config.engine_frequency;
            odom_publish_factor_ = config.odom_pub_factor;
            bio_ik_time_ = config.bio_ik_time;
        }
    };

    struct ThrowDebugParameter{
        bool debug_active_;
        // path for the file for storage of the debug data will be save
        std::string debug_data_path_;
        // name for the file for storage of the debug data
        std::string debug_data_file_name_;
        // prints the debug additionally to the ros console
        bool print_debug_to_console_;
        // activate the visualization of the limbs points as lines
        bool visualize_movement_;
        // Frequency of the point update rate for the visualization [in hz, > 0]
        double visualization_smoothness_;
        // activate the visualization of the left arm points as arrows
        bool visualize_left_arm_arrows_;
        // activate the visualization of the right arm points as arrows
        bool visualize_right_arm_arrows_;
        // activate the visualization of the left foot points as arrows
        bool visualize_left_foot_arrows_;
        // activate the visualization of the right foot points as arrows
        bool visualize_right_foot_arrows_;
        // Use Gradient colors for the visualization of the arrows
        bool visualize_arrows_use_gradient_;
        // Only visualize the points as arrows. Ovverides visualize_arrows_smoothness.
        bool visualize_only_points_arrows_;
        // Frequency of the point update rate for the visualization [in hz, > 0]
        int visualize_arrows_smoothness_;

        //////		Constructor
        ThrowDebugParameter(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            debug_active_ = config.debug_active;
            debug_data_path_ = config.debug_data_path;
            debug_data_file_name_ = config.debug_data_file_name;
            print_debug_to_console_ = config.print_debug_to_console;
            visualize_movement_ = config.visualize_movement;
            visualization_smoothness_ = config.visualization_smoothness;
            visualize_left_arm_arrows_ = config.visualize_left_arm_arrows;
            visualize_right_arm_arrows_ = config.visualize_right_arm_arrows;
            visualize_left_foot_arrows_ = config.visualize_left_foot_arrows;
            visualize_right_foot_arrows_ = config.visualize_right_foot_arrows;
            visualize_arrows_use_gradient_ = config.visualize_arrows_use_gradient;
            visualize_only_points_arrows_ = config.visualize_only_points_arrows;
            visualize_arrows_smoothness_ = config.visualize_arrows_smoothness;
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_H
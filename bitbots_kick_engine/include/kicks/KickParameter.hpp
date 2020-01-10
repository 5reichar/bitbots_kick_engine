#ifndef KICKPARAMETER_HPP
#define KICKPARAMETER_HPP

#include<vector>

struct struct3d
{
	double x, y, z;
};

struct AngleRequirements
{
	double min_angle;
	double max_angle;
};

struct KickStartPosition
{
	// Name of the type of Kick
	struct3d position;
	// The intervall in wich the angle between the robot and the ball must be for this starting position
	AngleRequirements angle_requiremts_robot_ball;
	// The intervall in wich the angle between the bal and the goal must be for this starting position
	AngleRequirements angle_requiremts_ball_goal;
};

struct KickType
{
	// Name of the type of Kick
	std::string name;
	// The intervall in wich the angle between the robot and the ball must be for this type of kick
	AngleRequirements angle_requiremts_robot_ball;
	// The intervall in wich the angle between the bal and the goal must be for this type of kick
	AngleRequirements angle_requiremts_ball_goal;
};

struct KickParameter
{
	std::vector<KickType> v_kick_types;
	std::vector<KickStartPosition> v_kick_start_positions;
};

struct KickAttributes
{
	// current position of the robots foot before the kick movements begins
	struct3d foot_starting_position;

	// position the robots foot has to move as preparation for the kick
	// if not used "prepare_kick_movement" should be false
	struct3d foot_prepare_for_kick_position;

	// current position of the ball the robot shall kick
	struct3d ball_position;

	// position to wich the ball shall be kicked
	struct3d kick_goal_position;

	// position the robot shall move the foot after making the kick
	// if not used "conclude_kick_movement" should be false
	struct3d foot_ending_position;

	// the calculated angle between the robot and the ball before the kick movement
	double angle_between_robot_and_ball;

	// the calculated angle between the ball and the goal before the kick movement
	double angle_between_ball_and_goal;

	// flag if the ball shall be kicked with the right foot of the robot
	bool kick_ball_with_right;

	// flag to show if "foot_position_for_starting_kick" is set
	bool prepare_kick_movement = false;

	// flag to show if "foot_ending_position" is set
	bool conclude_kick_movement = false;
};

#endif
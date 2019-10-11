struct struct3d
{
	double x, y, z;
};

struct KickParameter
{
	// current position of the robots foot before the kick movements begins
	struct3d foot_starting_position;

	// position the robots foot has to move as preparation for the kick
	// if not used "prepare_kick_movement" should be false
	struct3d foot_position_for_starting_kick;

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

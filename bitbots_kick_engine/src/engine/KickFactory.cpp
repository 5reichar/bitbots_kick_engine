#include <engine/KickFactory.hpp>

#include <math.h>
#include "kicks/SmoothSplineKick.hpp"

KickFactory::KickFactory(std::shared_ptr<KickEngineParameter> sp_engine_parameter)
{
	//TODO: testing

	m_sp_kick_engine_parameter = sp_engine_parameter;
	m_sp_kick_parameter = std::make_shared<KickParameter>();
}

std::shared_ptr<KickParameter> KickFactory::get_kick_parameter()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_parameter;
}

KickAttributes KickFactory::get_last_kicks_attributes()
{
	return m_struc_kick_attributes;
}

std::shared_ptr<bitbots_splines::SplineContainer> KickFactory::make_kick_trajection(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	init_kick_parameter(ball_position, goal_position, final_foot_position);

	auto sp_kick = create_kick();

	if (sp_kick->use_default_calculation_for_kick_stating_position())
	{
		m_struc_kick_attributes.foot_prepare_for_kick_position =
					KickFactoryService::get_kick_preparation_position(m_struc_kick_attributes.angle_between_robot_and_ball,
																	  m_struc_kick_attributes.angle_between_ball_and_goal,
																	  m_sp_kick_parameter);
		m_struc_kick_attributes.prepare_kick_movement = true;
	}

	return ((sp_kick && check_generale_requirements()) ? sp_kick->calculate_trajectories(m_struc_kick_attributes) : nullptr);
}

void KickFactory::init_kick_parameter(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position)
{
	//TODO: testing
	//TODO: cleanup
	auto angle_between_robot_and_ball = KickFactoryService::calculate_angle(ball_position->x, ball_position->y);
	auto kick_ball_with_right = KickFactoryService::check_kicking_with_right(angle_between_robot_and_ball);

	m_struc_kick_attributes.angle_between_robot_and_ball = angle_between_robot_and_ball;
	m_struc_kick_attributes.angle_between_ball_and_goal = KickFactoryService::calculate_angle((goal_position->x - ball_position->x), (goal_position->y - ball_position->y));
	m_struc_kick_attributes.kick_ball_with_right = kick_ball_with_right;
	m_struc_kick_attributes.foot_starting_position = kick_ball_with_right ? get_current_right_foot_position() : get_current_left_foot_position();
	m_struc_kick_attributes.ball_position = *ball_position;
	m_struc_kick_attributes.kick_goal_position = *goal_position;

	if (final_foot_position == nullptr)
	{
		m_struc_kick_attributes.conclude_kick_movement = false;
	}
	else
	{
		m_struc_kick_attributes.foot_ending_position = *final_foot_position;
		m_struc_kick_attributes.conclude_kick_movement = true;
	}
}

bool KickFactory::check_generale_requirements()
{
	//TODO: testing
	//TODO: cleanup

	bool requirements_meet = true;

	// check if the Robot is in the way
	requirements_meet &= !(m_struc_kick_attributes.angle_between_robot_and_ball == 90 && m_struc_kick_attributes.angle_between_ball_and_goal == 270);
	requirements_meet &= !(m_struc_kick_attributes.angle_between_robot_and_ball == 270 && m_struc_kick_attributes.angle_between_ball_and_goal == 90);

	return requirements_meet;
}

struct3d KickFactory::get_current_left_foot_position() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

struct3d KickFactory::get_current_right_foot_position() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

std::shared_ptr<Kick> KickFactory::create_kick()
{
	//TODO: testing
	//TODO: cleanup

	std::shared_ptr<Kick> sp_return_kick;

	KickTypeId enum_kick_type = m_sp_kick_parameter->default_kick_id;

	for (auto it = m_sp_kick_parameter->v_kick_types.begin(); it != m_sp_kick_parameter->v_kick_types.end(); ++it)
	{
		if (it->active
			&& KickFactoryService::check_angle_requirements(m_struc_kick_attributes.angle_between_robot_and_ball, it->angle_requiremts_robot_ball)
			&& KickFactoryService::check_angle_requirements(m_struc_kick_attributes.angle_between_ball_and_goal, it->angle_requiremts_ball_goal))
		{
			enum_kick_type = it->id;
			break;
		}
	}

	switch (enum_kick_type)
	{
	case KickTypeId::beziercurve:
		// TODO: implement
		//break;
	case KickTypeId::linear_spline:
		// TODO: implement
		//break;
	case KickTypeId::cubic_spline:
		// TODO: implement
		//break;
	case KickTypeId::smooth_spline:
		sp_return_kick = std::make_shared<SmoothSplineKick>(m_sp_kick_engine_parameter);
		break;
	default:
		sp_return_kick = std::make_shared<SmoothSplineKick>(m_sp_kick_engine_parameter);
		break;
	}

	return sp_return_kick;
}

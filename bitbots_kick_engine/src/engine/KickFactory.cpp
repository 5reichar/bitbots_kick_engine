#include <engine/KickFactory.hpp>

#include <math.h>
#include "kicks/LinearSplineKick.hpp"
#include "kicks/CubicSplineKick.hpp"
#include "kicks/SmoothSplineKick.hpp"
#include "kicks/BeziercurveKick.hpp"

KickFactory::KickFactory(std::shared_ptr<KickEngineParameter> sp_engine_parameter)
{
	//TODO: testing

	m_sp_kick_engine_parameter = sp_engine_parameter;
	m_sp_kick_parameter = std::make_shared<KickParameter>();
}

std::shared_ptr<KickParameter> KickFactory::getKickParameter()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_parameter;
}

KickAttributes KickFactory::getLastKicksAttributes()
{
	return m_struc_kick_attributes;
}

std::shared_ptr<bitbots_splines::SplineContainer> KickFactory::makeKickTrajection(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	initKickParameter(ball_position, goal_position, final_foot_position);

	auto sp_kick = createKick();

	if (sp_kick->useDefaultCalculationForKickStatingPosition())
	{
		m_struc_kick_attributes.foot_prepare_for_kick_position =
					KickFactoryService::getKickPreparationPosition(m_struc_kick_attributes.angle_between_robot_and_ball,
																	  m_struc_kick_attributes.angle_between_ball_and_goal,
																	  m_sp_kick_parameter);
		m_struc_kick_attributes.prepare_kick_movement = true;
	}

	return ((sp_kick && checkGeneraleRequirements()) ? sp_kick->calculateTrajectories(m_struc_kick_attributes) : nullptr);
}

void KickFactory::initKickParameter(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position)
{
	//TODO: testing
	//TODO: cleanup
	auto angle_between_robot_and_ball = KickFactoryService::calculateAngle(ball_position->x, ball_position->y);
	auto kick_ball_with_right = KickFactoryService::checkKickingWithRight(angle_between_robot_and_ball);

	m_struc_kick_attributes.angle_between_robot_and_ball = angle_between_robot_and_ball;
	m_struc_kick_attributes.angle_between_ball_and_goal = KickFactoryService::calculateAngle((goal_position->x - ball_position->x), (goal_position->y - ball_position->y));
	m_struc_kick_attributes.kick_ball_with_right = kick_ball_with_right;
	m_struc_kick_attributes.foot_starting_position = kick_ball_with_right ? getCurrentRightFootPosition() : getCurrentRightFootPosition();
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

bool KickFactory::checkGeneraleRequirements()
{
	//TODO: testing
	//TODO: cleanup

	// check if the Robot is in the way
	double angle_difference = std::abs(m_struc_kick_attributes.angle_between_robot_and_ball - m_struc_kick_attributes.angle_between_ball_and_goal);

	return angle_difference < 185 && angle_difference > 175;
}

struct3d KickFactory::getCurrentLeftFootPosition() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

struct3d KickFactory::getCurrentRightFootPosition() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

std::shared_ptr<Kick> KickFactory::createKick()
{
	//TODO: testing
	//TODO: cleanup

	std::shared_ptr<Kick> sp_return_kick;

	KickTypeId enum_kick_type = m_sp_kick_parameter->default_kick_id;

	for (auto it = m_sp_kick_parameter->v_kick_types.begin(); it != m_sp_kick_parameter->v_kick_types.end(); ++it)
	{
		if (it->active
			&& KickFactoryService::checkAngleRequirements(m_struc_kick_attributes.angle_between_robot_and_ball, it->angle_requiremts_robot_ball)
			&& KickFactoryService::checkAngleRequirements(m_struc_kick_attributes.angle_between_ball_and_goal, it->angle_requiremts_ball_goal))
		{
			enum_kick_type = it->id;
			break;
		}
	}

	switch (enum_kick_type)
	{
	case KickTypeId::beziercurve:
		sp_return_kick = std::make_shared<BeziercurveKick>(m_sp_kick_engine_parameter);
		break;
	case KickTypeId::linear_spline:
		sp_return_kick = std::make_shared<LinearSplineKick>(m_sp_kick_engine_parameter);
		break;
	case KickTypeId::cubic_spline:
		sp_return_kick = std::make_shared<CubicSplineKick>(m_sp_kick_engine_parameter);
		break;
	case KickTypeId::smooth_spline:
		sp_return_kick = std::make_shared<SmoothSplineKick>(m_sp_kick_engine_parameter);
		break;
	default:
		sp_return_kick = std::make_shared<SmoothSplineKick>(m_sp_kick_engine_parameter);
		break;
	}

	return sp_return_kick;
}

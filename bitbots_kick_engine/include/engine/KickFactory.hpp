#ifndef KICKFACTORY_HPP
#define KICKFACTORY_HPP

#include "KickFactoryService.hpp"
#include "../kicks/Kick.hpp"

class KickFactory
{
public:
	KickFactory(std::shared_ptr<KickEngineParameter> sp_engine_parameter);

	std::shared_ptr<KickParameter> get_kick_parameter();
	KickAttributes get_last_kicks_attributes();

	virtual std::shared_ptr<bitbots_splines::SplineContainer> make_kick_trajection(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position = nullptr);

private:
	bool check_generale_requirements();
	void init_kick_parameter(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);

	struct3d get_current_left_foot_position() const;
	struct3d get_current_right_foot_position() const;

	std::shared_ptr<Kick> create_kick();

	KickAttributes m_struc_kick_attributes;
	std::shared_ptr<KickParameter> m_sp_kick_parameter;
	std::shared_ptr<KickEngineParameter> m_sp_kick_engine_parameter;
};

#endif
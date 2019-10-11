#ifndef KICKFACADE_HPP
#define KICKFACADE_HPP

#include <memory>
#include "KickFactoryService.hpp"
#include "KickEngineParameter.hpp"
#include "../kicks/Kick.hpp"

class KickFactory
{
public:
	KickFactory(std::shared_ptr<KickEngineParameter> sp_parameter);

	KickParameter get_last_kicks_parameter();

	virtual bitbots_splines::SplineContainer* make_kick_trajection(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position = nullptr);

private:
	bool check_generale_requirements();
	void init_kick_parameter(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);

	struct3d get_current_left_foot_position() const;
	struct3d get_current_right_foot_position() const;

	std::shared_ptr<Kick> create_kick();

	KickParameter m_struc_kick_parameter;
	std::shared_ptr<KickEngineParameter> m_sp_kick_engine_parameter;
};

#endif
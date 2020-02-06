#ifndef KICKFACTORY_HPP
#define KICKFACTORY_HPP

#include "KickFactoryService.hpp"
#include "../kicks/Kick.hpp"
#include "../Footstep.hpp"

class KickFactory
{
public:
	KickFactory(std::shared_ptr<KickEngineParameter> sp_engine_parameter, std::shared_ptr<Footstep> footstep);

	std::shared_ptr<KickParameter> getKickParameter();
	KickAttributes getLastKicksAttributes();

	virtual std::shared_ptr<bitbots_splines::SplineContainer> makeKickTrajection(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position = nullptr);

private:
	bool checkGeneraleRequirements();
	void initKickParameter(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);

	struct3d getCurrentLeftFootPosition() const;
	struct3d getCurrentRightFootPosition() const;

	std::shared_ptr<Kick> createKick();

	KickAttributes m_struc_kick_attributes;
	std::shared_ptr<Footstep> m_sp_footstep;
	std::shared_ptr<KickParameter> m_sp_kick_parameter;
	std::shared_ptr<KickEngineParameter> m_sp_kick_engine_parameter;
};

#endif
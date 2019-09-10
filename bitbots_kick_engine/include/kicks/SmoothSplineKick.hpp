#ifndef SMOOTHSPLINEKICK_HPP
#define SMOOTHSPLINEKICK_HPP

#include "Kick.hpp"
#include <SmoothSpline.hpp>

class SmoothSplineKick : public Kick
{
public:

protected:
	virtual void build_trajectories() override;
	virtual bool additional_requirements() override;
	virtual bitbots_splines::SplineContainer init_trajectories() override;


private:
};

#endif
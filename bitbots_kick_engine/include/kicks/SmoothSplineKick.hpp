#ifndef SMOOTHSPLINEKICK_HPP
#define SMOOTHSPLINEKICK_HPP

#include "Kick.hpp"
#include <SmoothSpline.hpp>

class SmoothSplineKick : public Kick
{
public:

protected:
	virtual bool additional_requirements() override;
	virtual void build_trajectories() override;

private:
};

#endif
#ifndef THROW_CURVE_H
#define THROW_CURVE_H

#include <memory>
#include "parameter/throw_parameter.h"
#include "../../bitbots_spline/include/utils/SplineContainer.hpp"

class ThrowCurve
{
public:
	bool calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter);
	std::shared_ptr<bitbots_splines::SplineContainer> get_spline_container() const;

protected:
	virtual bool check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter);

	virtual void calculate_pick_up_ball_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);
	virtual void calculate_throw_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);
	virtual void calculate_throw_conclusion_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);

	void add_point_to_spline(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity = 0, double acceleration = 0);

	std::shared_ptr<bitbots_splines::SplineContainer> sp_spline_container_;
};

#endif
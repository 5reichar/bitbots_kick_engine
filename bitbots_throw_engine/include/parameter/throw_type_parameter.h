#ifndef THROW_TYPE_PARAMETER_H
#define THROW_TYPE_PARAMETER_H

#include <vector>

enum ThrowTypeId
{
	beziercurve = 1,
	linear_spline,
	cubic_spline,
	smooth_spline
};

struct ThrowType
{
	// Id of the type of Kick
	ThrowTypeId id_;
	// Shall this kick be used
	bool active_;
	// Parameter to control if there are two or more throws that are eligable which shall be used first.
	// the rule is: the throw with the lowest level will be used
	int throw_priority_level_;
	// The minimum distance this throw should be used for
	double min_throw_distance_;
	// The maximum distance this throw should be used for
	double max_throw_distance_;
	// Option to override the shares of the movement
	bool override_movement_shares_;
	// The share of the movement cycle dedicated to picking up the ball
	double pick_up_duration_share_;
	// The share of the movement cycle dedicated to prepare the throwing the ball
	double throw_preparation_duration_share_;
	// The share of the movement cycle dedicated to throwing the ball
	double throw_duration_share_;
	// The Angle at which the ball shall thrown
	double throw_anlge_;
};

struct ThrowTypeParameter
{
	ThrowType default_throw_;

	std::vector<ThrowType> v_throw_types_;
};

#endif
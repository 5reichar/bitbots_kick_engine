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
	// The minimum distance this throw should be used for
	double min_throw_distance_;
	// The maximum distance this throw should be used for
	double max_throw_distance_;
};

struct ThrowTypeParameter
{
	ThrowTypeId default_throw_id_;

	std::vector<ThrowType> v_throw_types_;
};

#endif
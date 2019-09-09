/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "spline/Curve.hpp"

namespace bitbots_splines
{

void Curve::addPoint(Curve::Point point)
{
    _points.push_back(point);
    addPointCallBack();
}

void Curve::addPoint(double time, double position, double velocity, double acceleration)
{
	addPoint(Curve::createPoint(time, position, velocity, acceleration));
}

void Curve::addPointCallBack()
{
}

const std::vector<Curve::Point> &Curve::points() const
{
    return _points;
}
std::vector<Curve::Point> &Curve::points()
{
    return _points;
}

} // namespace bitbots_splines

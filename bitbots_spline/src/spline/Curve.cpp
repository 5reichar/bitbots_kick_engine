/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "bitbots_splines/Curve.hpp"

namespace bitbots_splines
{

void Curve::addPoint(Curve::Point point)
{
    _points.push_back(point);
    addPointCallBack();
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

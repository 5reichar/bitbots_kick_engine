/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "spline/Curve.hpp"
#include <string>

namespace bitbots_splines
{

void Curve::add_point(Curve::Point point)
{
    points_.push_back(point);
    add_point_call_back();
}

void Curve::add_point(double time, double position, double velocity, double acceleration)
{
	add_point(Curve::create_point(time, position, velocity, acceleration));
}

const std::vector<Curve::Point> &Curve::points() const
{
    return points_;
}
std::vector<Curve::Point> &Curve::points()
{
    return points_;
}

std::string Curve::get_debug_string()
{
  std::string output;
  int i = 0;
  for (auto &p : points_)
  {
    output += "Point:" + std::to_string(i) + "\n";
    output += "  Time: " + std::to_string(p.time_) + "\n";
    output += "  Pos: " + std::to_string(p.position_) + "\n";
    output += "  Vel: " + std::to_string(p.velocity_) + "\n";
    output += "  Acc: " + std::to_string(p.acceleration_) + "\n";
    i++;
  }
  return output;
}

void Curve::add_point_call_back()
{
}

} // namespace bitbots_splines

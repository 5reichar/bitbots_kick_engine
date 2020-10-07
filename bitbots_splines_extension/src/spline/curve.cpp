/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "spline/curve.h"
#include <string>
#include <sstream>
#include <algorithm>

namespace bitbots_splines
{

void Curve::add_point(Curve::Point point)
{
    points_.push_back(point);
    add_point_call_back();
}

void Curve::add_point(double time, double position)
{
	add_point(time, position, 0.0);
}

void Curve::add_point(double time, double position, double velocity)
{
    add_point(time, position, velocity, 0.0);
}

void Curve::add_point(double time, double position, double velocity, double acceleration)
{
    add_point(Point{time, position, velocity, acceleration});
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

std::string Curve::get_debug_csv()
{
  std::stringstream output;
  std::stringstream time;
  std::stringstream position;
  std::stringstream velocity;
  std::stringstream acceleration;

  int i = 0;
  output << "Point";
  time << "Time";
  position << "Position";
  velocity << "Velocity";
  acceleration << "Acceleration";

  for (auto &p : points_)
  {
    output << ", " << i ;
    time << ", " << p.time_;
    position << ", " << p.position_;
    velocity << ", " << p.velocity_;
    acceleration << ", " << p.acceleration_;
    i++;
  }

  output << std::endl << time.str();
  output << std::endl << position.str();
  output << std::endl << velocity.str();
  output << std::endl << acceleration.str();

  return output.str();
}

double Curve::min() const
{
  double return_value = 9999;

  for (auto & p : points_)
  {
    if (p.time_ < return_value)
    {
      return_value = p.time_;
    }
  }

  return return_value;
}

double Curve::max() const
{
  double return_value = 0;

  for (auto & p : points_)
  {
    if (p.time_ > return_value)
    {
      return_value = p.time_;
    }
  }
  
  return return_value;
}

void Curve::add_point_call_back()
{
}

} // namespace bitbots_splines

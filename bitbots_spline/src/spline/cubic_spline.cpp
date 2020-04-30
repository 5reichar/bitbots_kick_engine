/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#include <stdexcept>
#include <algorithm>
#include <random>
#include <chrono>
#include "spline/cubic_spline.hpp"

namespace bitbots_splines
{

void CubicSpline::add_point(double time, double position, double velocity)
{
    Spline::Point point = {time, position, velocity, 0.0};
    add_point(point);
}

void CubicSpline::random_noise(double stdDevPos, double stdDevVel, bool updateBounds)
{
    //Initialize the generator
    std::mt19937 generator(static_cast<long unsigned int>(
        std::chrono::high_resolution_clock::now()
            .time_since_epoch()
            .count()));
    std::normal_distribution<double> distPos(0.0, stdDevPos);
    std::normal_distribution<double> distVel(0.0, stdDevVel);
    //Apply noise on points
    for (size_t i = 0; i < points_.size(); i++)
    {
        if (!updateBounds && (i == 0 || i == points_.size() - 1))
        {
            continue;
        }
        double deltaPos = distPos(generator);
        double deltaVel = distVel(generator);
        points_[i].position_ += deltaPos;
        points_[i].velocity_ += deltaVel;
    }
    //Recompute the splines
    compute_splines();
}

void CubicSpline::subdivide(unsigned int divider)
{
    std::vector<Point> newPoints;
    for (size_t i = 1; i < points_.size(); i++)
    {
        double t1 = points_[i - 1].time_;
        double t2 = points_[i].time_;
        double length = fabs(t2 - t1);
        if (length < 0.0001)
        {
            continue;
        }
        double step = length / (divider + 1);
        for (double t = t1; t < t2 - step / 2.0; t += step)
        {
            newPoints.push_back({t, Spline::pos(t), Spline::vel(t)});
        }
    }
    newPoints.push_back(points_.back());
    //Replace points container
    points_ = newPoints;
    //Recompute the splines
    compute_splines();
}

void CubicSpline::compute_splines()
{
    Spline::splines_.clear();
    if (points_.size() < 2)
    {
        return;
    }

    std::sort(
        points_.begin(),
        points_.end(),
        [](const Point &p1, const Point &p2) -> bool {
            return p1.time_ < p2.time_;
        });

    for (size_t i = 1; i < points_.size(); i++)
    {
        double time = points_[i].time_ - points_[i - 1].time_;
        if (time > 0.00001)
        {
            Spline::splines_.push_back({polynom_fit(time,
                                                    points_[i - 1].position_, points_[i - 1].velocity_,
                                                    points_[i].position_, points_[i].velocity_),
                                        points_[i - 1].time_,
                                        points_[i].time_});
        }
    }
}

void CubicSpline::import_call_back()
{
    size_t size = Spline::splines_.size();
    if (size == 0)
    {
        return;
    }

    double tBegin = Spline::splines_.front().min;
    points_.push_back({tBegin, Spline::pos(tBegin), Spline::vel(tBegin)});

    for (size_t i = 1; i < size; i++)
    {
        double t1 = Spline::splines_[i - 1].max;
        double t2 = Spline::splines_[i].min;
        double pos1 = Spline::pos(t1);
        double vel1 = Spline::vel(t1);
        double pos2 = Spline::pos(t2);
        double vel2 = Spline::vel(t2);

        if (
            fabs(t2 - t1) < 0.0001 &&
            fabs(pos2 - pos1) < 0.0001 &&
            fabs(vel2 - vel1) < 0.0001)
        {
            points_.push_back({t1, pos1, vel1});
        }
        else
        {
            points_.push_back({t1, pos1, vel1});
            points_.push_back({t2, pos2, vel2});
        }
    }

    double tEnd = Spline::splines_.back().max;
    points_.push_back({tEnd, Spline::pos(tEnd), Spline::vel(tEnd)});
}

Polynom CubicSpline::polynom_fit(double t,
                                 double pos1, double vel1,
                                 double pos2, double vel2) const
{
    if (t <= 0.00001)
    {
        throw std::logic_error(
            "CubicSpline invalid spline interval");
    }
    double t2 = t * t;
    double t3 = t2 * t;
    Polynom p;
    p.getCoefs().resize(4);
    p.getCoefs()[0] = pos1;
    p.getCoefs()[1] = vel1;
    p.getCoefs()[3] = (vel2 - vel1 - 2.0 * (pos2 - pos1 - vel1 * t) / t) / t2;
    p.getCoefs()[2] = (pos2 - pos1 - vel1 * t - p.getCoefs()[3] * t3) / t2;

    return p;
}

} // namespace bitbots_splines

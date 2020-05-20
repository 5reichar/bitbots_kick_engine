/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include "spline/smooth_spline.h"

namespace bitbots_splines
{

void SmoothSpline::add_point(double time, double position,
                            double velocity, double acceleration)
{
    SplineBase::Point point = {time, position, velocity, acceleration};
    add_point(point);
}

void SmoothSpline::compute_splines()
{
    SplineBase::splines_.clear();
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
            SplineBase::splines_.push_back({polynom_fit(time,
                                                        points_[i - 1].position_, points_[i - 1].velocity_, points_[i - 1].acceleration_,
                                                        points_[i].position_, points_[i].velocity_, points_[i].acceleration_),
                                            points_[i - 1].time_,
                                            points_[i].time_});
        }
    }
}

void SmoothSpline::import_call_back()
{
    size_t size = SplineBase::splines_.size();
    if (size == 0)
    {
        return;
    }

    double tBegin = SplineBase::splines_.front().min;
    points_.push_back({tBegin,
                       SplineBase::position(tBegin),
                       SplineBase::velocity(tBegin),
                       SplineBase::acceleration(tBegin)});

    for (size_t i = 1; i < size; i++)
    {
        double t1 = SplineBase::splines_[i - 1].max;
        double t2 = SplineBase::splines_[i].min;
        double pos1 = SplineBase::position(t1);
        double vel1 = SplineBase::velocity(t1);
        double acc1 = SplineBase::acceleration(t1);
        double pos2 = SplineBase::position(t2);
        double vel2 = SplineBase::velocity(t2);
        double acc2 = SplineBase::acceleration(t2);

        if (
            fabs(t2 - t1) < 0.0001 &&
            fabs(pos2 - pos1) < 0.0001 &&
            fabs(vel2 - vel1) < 0.0001 &&
            fabs(acc2 - acc1) < 0.0001)
        {
            points_.push_back({t1, pos1, vel1, acc1});
        }
        else
        {
            points_.push_back({t1, pos1, vel1, acc1});
            points_.push_back({t2, pos2, vel2, acc2});
        }
    }

    double tEnd = SplineBase::splines_.back().max;
    points_.push_back({tEnd,
                       SplineBase::position(tEnd),
                       SplineBase::velocity(tEnd),
                       SplineBase::acceleration(tEnd)});
}

Polynom SmoothSpline::polynom_fit(double t,
                                  double pos1, double vel1, double acc1,
                                  double pos2, double vel2, double acc2) const
{
    if (t <= 0.00001)
    {
        throw std::logic_error(
            "SmoothSpline invalid spline interval");
    }
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    Polynom p;
    p.getCoefs().resize(6);
    p.getCoefs()[0] = pos1;
    p.getCoefs()[1] = vel1;
    p.getCoefs()[2] = acc1 / 2;
    p.getCoefs()[3] = -(-acc2 * t2 + 3 * acc1 * t2 + 8 * vel2 * t + 12 * vel1 * t - 20 * pos2 + 20 * pos1) / (2 * t3);
    p.getCoefs()[4] = (-2 * acc2 * t2 + 3 * acc1 * t2 + 14 * vel2 * t + 16 * vel1 * t - 30 * pos2 + 30 * pos1) / (2 * t4);
    p.getCoefs()[5] = -(-acc2 * t2 + acc1 * t2 + 6 * vel2 * t + 6 * vel1 * t - 12 * pos2 + 12 * pos1) / (2 * t5);

    return p;
}

} // namespace bitbots_splines

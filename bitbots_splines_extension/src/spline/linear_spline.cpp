/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#include <stdexcept>
#include <algorithm>
#include "spline/linear_spline.h"

namespace bitbots_splines
{
void LinearSpline::compute_splines()
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
            Polynom poly(1);
            poly(0) = points_[i - 1].position_;
            poly(1) = (points_[i].position_ - points_[i - 1].position_) / time;
            add_part(poly, points_[i - 1].time_, points_[i].time_);
        }
    }
}

} // namespace bitbots_splines
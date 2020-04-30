/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef SPLINECONTAINER_HPP
#define SPLINECONTAINER_HPP

#include <map>
#include <stdexcept>
#include <fstream>
#include <set>
#include <algorithm>
#include "spline/curve.hpp"

namespace bitbots_splines
{

enum class CurvePurpose
{
    is_double_support,
    is_left_support_foot,
    trunk_position_x,
    trunk_position_y,
    trunk_position_z,
    trunk_axis_x,
    trunk_axis_y,
    trunk_axis_z,
    foot_position_x,
    foot_position_y,
    foot_position_z,
    foot_axis_x,
    foot_axis_y,
    foot_axis_z,
    left_hand_position_x,
    left_hand_position_y,
    left_hand_position_z,
    left_hand_axis_x,
    left_hand_axis_y,
    left_hand_axis_z,
    right_hand_position_x,
    right_hand_position_y,
    right_hand_position_z,
    right_hand_axis_x,
    right_hand_axis_y,
    right_hand_axis_z
};

/*
 * SplineContainer
 *
 * Wrapper for map of generic splines
 * types indexed by string name
 * Implementation of implort/export from files
 */
class SplineContainer
{
public:
    ~SplineContainer()
    {
        for (const auto &it : _container)
        {
            delete it.second;
        }
    }

    /*
     * Return the number of contained splines
     */
    inline size_t size() const
    {
        return _container.size();
    }

    /*
     * Add an empty spline with given name.
     * Variadic arguments allow to pass parameters to
     * spline constructor.
     */
    inline void add(const CurvePurpose &name, Curve *curve)
    {
        if (_container.count(name) != 0)
        {
            throw std::logic_error(
                "SplineContainer spline already added");
        }
        _container[name] = curve;
    }

    /*
     * Return true if given spline
     * name is contained
     */
    inline bool exist(const CurvePurpose &name) const
    {
        return _container.count(name) > 0;
    }

    /*
     * Access to given named spline
     */
    inline const Curve *get(const CurvePurpose &name) const
    {
        if (_container.count(name) == 0)
        {
            throw std::logic_error(
                "SplineContainer invalid name: " + get_purpose_name(name));
        }
        return _container.at(name);
    }
    inline Curve *get(const CurvePurpose &name)
    {
        if (_container.count(name) == 0)
        {
            throw std::logic_error(
                "SplineContainer invalid name: " + get_purpose_name(name));
        }
        return _container.at(name);
    }

    /*
     * Access to internal map containera
     */
    const std::map<CurvePurpose, Curve *> &get() const
    {
        return _container;
    }
    std::map<CurvePurpose, Curve *> &get()
    {
        return _container;
    }

    /*
     * Returns all time points where a point in any spline exists.
     */
    std::vector<double> get_times()
    {
        std::set<double> times;
        std::vector<double> times_sorted;
        // go trough all splines
        for (const auto &sp : _container)
        {
            // go trough all points of the spline
            for (Curve::Point point : sp.second->points())
            {
                times.insert(point.time_);
            }
        }
        //insert set into vector
        times_sorted.insert(times_sorted.end(), times.begin(), times.end());
        std::sort(times_sorted.begin(), times_sorted.end());
        return times_sorted;
    }
    
private:
    /*
     * Spline container indexed
     * by their name
     */
    std::map<CurvePurpose, Curve *> _container;

    static std::string get_purpose_name(CurvePurpose purpose)
    {
        std::string name;
        switch (purpose)
        {
        case CurvePurpose::is_double_support:
            name = "is_double_support:";
            break;
        case CurvePurpose::is_left_support_foot:
            name = "is_left_support_foot:";
            break;
        case CurvePurpose::trunk_position_x:
            name = "trunk_position_x:";
            break;
        case CurvePurpose::trunk_position_y:
            name = "trunk_position_y:";
            break;
        case CurvePurpose::trunk_position_z:
            name = "trunk_position_z:";
            break;
        case CurvePurpose::trunk_axis_x:
            name = "trunk_axis_x:";
            break;
        case CurvePurpose::trunk_axis_y:
            name = "trunk_axis_y:";
            break;
        case CurvePurpose::trunk_axis_z:
            name = "trunk_axis_z:";
            break;
        case CurvePurpose::foot_position_x:
            name = "foot_position_x:";
            break;
        case CurvePurpose::foot_position_y:
            name = "foot_position_y:";
            break;
        case CurvePurpose::foot_position_z:
            name = "foot_position_z:";
            break;
        case CurvePurpose::foot_axis_x:
            name = "foot_axis_x:";
            break;
        case CurvePurpose::foot_axis_y:
            name = "foot_axis_y:";
            break;
        case CurvePurpose::foot_axis_z:
            name = "foot_axis_z:";
            break;

        default:
            name = "unknown_purpose";
            break;
        }

        return name;
    }
};

} // namespace bitbots_splines

#endif

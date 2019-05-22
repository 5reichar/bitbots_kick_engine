/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef SPLINECONTAINER_HPP
#define SPLINECONTAINER_HPP

#include <string>
#include <map>
#include <stdexcept>
#include <fstream>
#include "spline/Curve.hpp"
#include <set>
#include <algorithm>
#include <vector>

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
    foot_axis_z
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
    std::vector<double> getTimes()
    {
        std::set<double> times;
        std::vector<double> times_sorted;
        // go trough all splines
        for (const auto &sp : _container)
        {
            // go trough all points of the spline
            for (Curve::Point point : sp.second->points())
            {
                times.insert(point.time);
            }
        }
        //insert set into vector
        times_sorted.insert(times_sorted.end(), times.begin(), times.end());
        std::sort(times_sorted.begin(), times_sorted.end());
        return times_sorted;
    }

    /*
     * Return minimum and maximum abscisse values
     * of all registered splines parts
     */
    /*
    double min() const
    {
        if (_container.size() == 0)
        {
            return 0.0;
        }
        bool isFirst = true;
        double m = 0.0;
        for (const auto &sp : _container)
        {
            if (isFirst || m > sp.second.min())
            {
                m = sp.second.min();
                isFirst = false;
            }
        }
        return m;
    }
    double max() const
    {
        if (_container.size() == 0)
        {
            return 0.0;
        }
        bool isFirst = true;
        double m = 0.0;
        for (const auto &sp : _container)
        {
            if (isFirst || m < sp.second.max())
            {
                m = sp.second.max();
                isFirst = false;
            }
        }
        return m;
    }

    /*
     * Export to and Import from given file name
     * in "spline" CSV format prefixed with spline name
     */
    /*
    void exportData(const std::string &fileName) const
    {
        if (_container.size() == 0)
        {
            throw std::logic_error("SplineContainer empty");
        }

        std::ofstream file(fileName);
        if (!file.is_open())
        {
            throw std::runtime_error(
                "SplineContainer unable to write file: " + fileName);
        }

        for (const auto &sp : _container)
        {
            file << "'" << sp.first << "' ";
            sp.second->exportData(file);
        }

        file.close();
    }
    void importData(const std::string &fileName)
    {
        std::ifstream file(fileName);
        if (!file.is_open())
        {
            throw std::runtime_error(
                "SplineContainer unable to read file: " + fileName);
        }

        bool isParseError;
        while (file.good())
        {
            isParseError = true;
            //Skip name delimitor
            if (file.peek() != '\'')
                break;
            file.ignore();
            if (!file.good())
                break;
            //Parse spline name
            char name[256];
            file.getline(name, 256, '\'');
            //Import founded spline
            add(std::string(name));
            if (!file.good())
                break;
            _container.at(std::string(name)).importData(file);
            isParseError = false;
            //Skip end line
            while (file.peek() == ' ' || file.peek() == '\n')
            {
                if (!file.good())
                    break;
                file.ignore();
            }
        }
        if (isParseError)
        {
            throw std::logic_error(
                "SplineContainer invalid input format");
        }

        file.close();
    }
    */
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

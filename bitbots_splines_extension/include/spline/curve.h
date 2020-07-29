/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_EXTENSION_CURVE_H
#define BITBOTS_SPLINES_EXTENSION_CURVE_H

#include <vector>
#include <string>

namespace bitbots_splines
{

class Curve
{
public:
    virtual double position(double t) const = 0;
    virtual double velocity(double t) const = 0;
    virtual double acceleration(double t) const = 0;
    virtual double jerk(double t) const = 0;

    virtual double position_mod(double t) const = 0;
    virtual double velocity_mod(double t) const = 0;
    virtual double acceleration_mod(double t) const = 0;
    virtual double jerk_mod(double t) const = 0;

    struct Point
    {
        double time_;
        double position_;
        double velocity_;
        double acceleration_;
    };

    static Point create_point(double time, double position, double velocity = 0.0, double acceleration = 0.0)
    {
        Point point = {time, position, velocity, acceleration};
        return point;
    };

    /**
     * Add a new point
     */
	virtual void add_point(Point point);
	virtual void add_point(double time, double position, double velocity = 0.0, double acceleration = 0.0);

    /**
     * Access to points container
     */
    const std::vector<Point> &points() const;
    std::vector<Point> &points();


    /**
     * Returns a string representation of the Spline to get inside while debugging.
     * @return
     */
    std::string get_debug_string();
    std::string get_debug_csv();

protected:
    virtual void add_point_call_back();

    /**
     * Points container
     */
    std::vector<Point> points_;

private:
};

} // namespace bitbots_splines

#endif

/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef CURVE_HPP
#define CURVE_HPP

#include <vector>

namespace bitbots_splines
{

class Curve
{
public:
    virtual double pos(double t) const = 0;
    virtual double vel(double t) const = 0;
    virtual double acc(double t) const = 0;
    virtual double jerk(double t) const = 0;

    virtual double posMod(double t) const = 0;
    virtual double velMod(double t) const = 0;
    virtual double accMod(double t) const = 0;
    virtual double jerkMod(double t) const = 0;

    struct Point
    {
        double time;
        double position;
        double velocity;
        double acceleration;
    };

    static Point createPoint(double time, double position, double velocity = 0.0, double acceleration = 0.0)
    {
        Point point = {time, position, velocity, acceleration};
        return point;
    };

    /**
     * Add a new point
     */
    virtual void addPoint(Point point);

    /**
     * Access to points container
     */
    const std::vector<Point> &points() const;
    std::vector<Point> &points();

protected:
    /**
     * Points container
     */
    std::vector<Point> _points;

    virtual void addPointCallBack();

private:
};

} // namespace bitbots_splines

#endif

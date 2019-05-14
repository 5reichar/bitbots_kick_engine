#ifndef KICKENGINEBASE_HPP
#define KICKENGINEBASE_HPP

#include "../bitbots_splines/Curve.hpp"

class KickEngineBase
{
public:
    KickEngineBase(/* args */);
    ~KickEngineBase();

protected:
    virtual void create_curves() = 0;

private:
    bitbots_splines::Curve *curve_x;
    bitbots_splines::Curve *curve_y;
    bitbots_splines::Curve *curve_z;
};

#endif
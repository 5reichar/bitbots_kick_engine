#include "KickEngineBase.hpp"

KickEngineBase::KickEngineBase()
    : curve_x(NULL),
      curve_y(NULL),
      curve_z(NULL)
{
}

KickEngineBase::~KickEngineBase()
{
    if (this->curve_x != NULL)
    {
        delete this->curve_x;
    }

    if (this->curve_y != NULL)
    {
        delete this->curve_y;
    }

    if (this->curve_z != NULL)
    {
        delete this->curve_z;
    }
}
/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include <iomanip>
#include <stdexcept>
#include "spline/spline_base.h"

namespace bitbots_splines
{

double SplineBase::position(double t) const
{
    return interpolation(t, &Polynom::pos);
}
double SplineBase::velocity(double t) const
{
    return interpolation(t, &Polynom::vel);
}
double SplineBase::acceleration(double t) const
{
    return interpolation(t, &Polynom::acc);
}
double SplineBase::jerk(double t) const
{
    return interpolation(t, &Polynom::jerk);
}

double SplineBase::position_mod(double t) const
{
    return interpolation_mod(t, &Polynom::pos);
}
double SplineBase::velocity_mod(double t) const
{
    return interpolation_mod(t, &Polynom::vel);
}
double SplineBase::acceleration_mod(double t) const
{
    return interpolation_mod(t, &Polynom::acc);
}
double SplineBase::jerk_mod(double t) const
{
    return interpolation_mod(t, &Polynom::jerk);
}

double SplineBase::min() const
{
    if (splines_.size() == 0)
    {
        return 0.0;
    }
    else
    {
        return splines_.front().min;
    }
}
double SplineBase::max() const
{
    if (splines_.size() == 0)
    {
        return 0.0;
    }
    else
    {
        return splines_.back().max;
    }
}

void SplineBase::add_point_call_back()
{
    compute_splines();
}

void SplineBase::export_data(std::ostream &os) const
{
    for (size_t i = 0; i < splines_.size(); i++)
    {
        os << std::setprecision(17) << splines_[i].min << " ";
        os << std::setprecision(17) << splines_[i].max << " ";
        os << std::setprecision(17) << splines_[i].polynom.getCoefs().size() << " ";
        for (size_t j = 0; j < splines_[i].polynom.getCoefs().size(); j++)
        {
            os << std::setprecision(17) << splines_[i].polynom.getCoefs()[j] << " ";
        }
    }
    os << std::endl;
}
void SplineBase::import_data(std::istream &is)
{
    bool isFormatError;
    while (is.good())
    {
        isFormatError = true;
        double min;
        double max;
        size_t size;
        Polynom p;
        //Load spline interval and degree
        is >> min;
        if (!is.good())
            break;
        is >> max;
        if (!is.good())
            break;
        is >> size;
        //Load polynom coeficients
        p.getCoefs().resize(size);
        for (size_t i = 0; i < size; i++)
        {
            if (!is.good())
                break;
            is >> p.getCoefs()[i];
        }
        //Save spline part
        isFormatError = false;
        splines_.push_back({p, min, max});
        //Exit on line break
        while (is.peek() == ' ')
        {
            if (!is.good())
                break;
            is.ignore();
        }
        if (is.peek() == '\n')
        {
            break;
        }
    }
    if (isFormatError)
    {
        throw std::logic_error(
            "Spline import format invalid");
    }
    //Call possible post import
    import_call_back();
}

size_t SplineBase::size() const
{
    return splines_.size();
}

const SplineBase::Spline_t &SplineBase::part(size_t index) const
{
    return splines_.at(index);
}

void SplineBase::add_part(const Polynom &poly, double min, double max)
{
    splines_.push_back({poly, min, max});
}

void SplineBase::copy_data(const SplineBase &sp)
{
    splines_ = sp.splines_;
    //Call possible post import
    import_call_back();
}

void SplineBase::import_call_back()
{
}

double SplineBase::interpolation(double x, double (Polynom::*func)(double) const) const
{
    //Empty case
    if (splines_.size() == 0)
    {
        return 0.0;
    }
    //Bound asked abscisse into spline range
    if (x <= splines_.front().min)
    {
        x = splines_.front().min;
    }
    if (x >= splines_.back().max)
    {
        x = splines_.back().max;
    }
    //Bijection spline search
    size_t indexLow = 0;
    size_t indexUp = splines_.size() - 1;
    while (indexLow != indexUp)
    {
        size_t index = (indexUp + indexLow) / 2;
        if (x < splines_[index].min)
        {
            indexUp = index - 1;
        }
        else if (x > splines_[index].max)
        {
            indexLow = index + 1;
        }
        else
        {
            indexUp = index;
            indexLow = index;
        }
    }
    //Compute and return spline value
    return (splines_[indexUp].polynom.*func)(x - splines_[indexUp].min);
}

double SplineBase::interpolation_mod(double x,
                                double (Polynom::*func)(double) const) const
{
    if (x < 0.0)
    {
        x = 1.0 + (x - ((int)x / 1));
    }
    else if (x > 1.0)
    {
        x = (x - ((int)x / 1));
    }
    return interpolation(x, func);
}

} // namespace bitbots_splines

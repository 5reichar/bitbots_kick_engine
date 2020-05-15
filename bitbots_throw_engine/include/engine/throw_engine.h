#ifndef THROW_ENGINE_H
#define THROW_ENGINE_H

#include "throws/throw_factory.h"
#include "utility/throw_utilities.h"
#include "parameter/throw_parameter_builder.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"
#include "../../unchanged/bitbots_splines/include/bitbots_splines/abstract_engine.h"

class ThrowEngine : public bitbots_splines::AbstractEngine<ThrowRequest, ThrowResponse>
{
public:
    virtual ThrowResponse update(double dt) override;
    virtual void reset() override;

    // Wrapper to keep style consistence
    void set_goals(const ThrowRequest & request);
    int get_percent_done() const;

    void set_throw_types(std::shared_ptr<ThrowTypeParameter> types);
    void set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter);

private:
    virtual void setGoals(const ThrowRequest & request) override;
    virtual int getPercentDone() const override;

    double time_;

    std::shared_ptr<ThrowCurve> sp_current_throw_;
    std::shared_ptr<ThrowFactory> sp_throw_factory_;
    std::shared_ptr<ThrowTypeParameter> sp_throw_types_;
    std::shared_ptr<ThrowEngineParameter> sp_engine_parameter_;
};

#endif
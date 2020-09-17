#ifndef BITBOTS_THROW_THROW_ENGINE_H
#define BITBOTS_THROW_THROW_ENGINE_H

#include "bitbots_splines/abstract_engine.h"
#include "utility/throw_utilities.h"

#include "throws/throw_factory.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"

namespace bitbots_throw{
    class ThrowEngine : public bitbots_splines::AbstractEngine<ThrowRequest, ThrowResponse>{
    public:
        ThrowEngine();
        ThrowResponse update(double dt) override;
        void reset() override;

        void visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer);

        /**
         * @return percentage of the throw movement completed, thru the update methode.
         */
        int get_percent_done() const;
        int8_t get_movement_stage() const;

        void set_goals(const ThrowRequest & request);
        void set_throw_types(std::shared_ptr<ThrowTypeParameter> & types);
        void set_engine_parameter(std::shared_ptr<ThrowEngineParameter> & parameter);

        std::string get_throw_points_as_string() const;

    private:
        int getPercentDone() const override;
        void setGoals(const ThrowRequest & request) override;

        std::shared_ptr<ThrowService> create_service(const ThrowTypeId throw_type_id, const ThrowRequest & request);

        double time_;

        std::shared_ptr<ThrowMaterial> sp_current_throw_;
        std::shared_ptr<ThrowFactory> sp_throw_factory_;
        std::shared_ptr<ThrowTypeParameter> sp_throw_types_;
        std::shared_ptr<ThrowEngineParameter> sp_engine_parameter_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_H
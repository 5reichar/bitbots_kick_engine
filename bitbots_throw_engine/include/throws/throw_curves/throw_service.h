#ifndef BITBOTS_THROW_THROW_SERVICE_H
#define BITBOTS_THROW_THROW_SERVICE_H

#include "utility/throw_utilities.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"

namespace bitbots_throw{
    class ThrowService{
    public:
        ThrowService(ThrowRequest request, ThrowType throw_type, ThrowEngineParameter engine_parameter);

        virtual Struct3dRPY get_left_arm_start();
        virtual Struct3dRPY get_left_arm_pick_up();
        virtual Struct3dRPY get_left_arm_throw_zenith();
        virtual Struct3dRPY get_left_arm_throw_start();
        virtual Struct3dRPY get_left_arm_ball_at_head_height();
        virtual Struct3dRPY get_left_arm_throw_release();

        virtual Struct3dRPY get_right_arm_start();
        virtual Struct3dRPY get_right_arm_pick_up();
        virtual Struct3dRPY get_right_arm_throw_zenith();
        virtual Struct3dRPY get_right_arm_throw_start();
        virtual Struct3dRPY get_right_arm_ball_at_head_height();
        virtual Struct3dRPY get_right_arm_throw_release();

        virtual Struct3dRPY get_left_foot_start();
        virtual Struct3dRPY get_left_foot_squat();

        virtual Struct3dRPY get_right_foot_start();
        virtual Struct3dRPY get_right_foot_squat();

        virtual double get_movement_time_pick_up_ball();
        virtual double get_movement_time_throw_preparation();
        virtual double get_movement_time_throw();
        virtual double get_movement_time_throw_conclusion();

    private:
        ThrowRequest request_;
        ThrowType throw_type_;
        ThrowEngineParameter engine_parameter_;
    };
}
#endif //BITBOTS_THROW_THROW_SERVICE_H

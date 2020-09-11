#ifndef BITBOTS_THROW_THROW_SERVICE_H
#define BITBOTS_THROW_THROW_SERVICE_H

#include "parameter/throw_parameter.h"

namespace bitbots_throw{
    class ThrowService{
    public:
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

    private:
        ThrowParameter throw_parameter_;
    };
}
#endif //BITBOTS_THROW_THROW_SERVICE_H

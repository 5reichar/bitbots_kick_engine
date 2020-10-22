#ifndef BITBOTS_THROW_THROW_MOVEMENT_POSITION_ONLY_H
#define BITBOTS_THROW_THROW_MOVEMENT_POSITION_ONLY_H

#include "throw_movement.h"

namespace bitbots_throw{
    class ThrowMovementPositionOnly : public ThrowMovement{
    public:
        ThrowMovementPositionOnly(std::shared_ptr<ThrowMaterial> material);

    protected:
        void add_movement_prepare_throw() override;
        void add_movement_throw() override;

    private:
        double throw_start_time_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MOVEMENT_POSITION_ONLY_H

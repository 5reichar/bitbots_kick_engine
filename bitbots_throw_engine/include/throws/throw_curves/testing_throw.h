#ifndef BITBOTS_THROW_TESTING_THROW_H
#define BITBOTS_THROW_TESTING_THROW_H

#include "throw_curve.h"

namespace bitbots_throw{
    class TestingThrow : public ThrowCurve{
    public:
        TestingThrow();

    protected:
        void calculate_pick_up_ball_movement(double & time
                                            ,std::shared_ptr<ThrowParameter> & throw_parameter)
                                            override;
        void calculate_throw_preparation_movement(double & time
                                                 ,std::shared_ptr<ThrowParameter> & throw_parameter)
                                                 override;
        void calculate_throw_movement(double & time
                                     ,std::shared_ptr<ThrowParameter> & throw_parameter)
                                     override ;
        void calculate_throw_conclusion_movement(double & time
                                                ,std::shared_ptr<ThrowParameter> & throw_parameter)
                                                override;

    private:
        int max_prepare_rounds_;
    };

}

#endif //BITBOTS_THROW_TESTING_THROW_H

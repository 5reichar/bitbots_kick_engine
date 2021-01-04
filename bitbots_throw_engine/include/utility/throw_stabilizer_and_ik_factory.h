#ifndef BITBOTS_THROW_THROW_STABILIZER_AND_IK_FACTORY_H
#define BITBOTS_THROW_THROW_STABILIZER_AND_IK_FACTORY_H

#include <map>
#include "throw_ik.h"
#include "throw_stabilizer.h"

namespace bitbots_throw{
    class ThrowStabilizerAndIKFactory{
    public:
        ThrowStabilizerAndIKFactory();
        void set_bio_ik_timeout(double timeout);
        void set_kinematic_model(moveit::core::RobotModelPtr kinematic_model);
        void set_debug_parameter(std::shared_ptr<ThrowDebugParameter> debug_parameter);

        bitbots_splines::JointGoals calculate_joint_goals(ThrowResponse & response);

    private:
        std::vector<ThrowStabilizerData> get_stabilizer_data_left_hand(ThrowResponse & response);
        std::vector<ThrowStabilizerData> get_stabilizer_data_right_hand(ThrowResponse & response);
        std::vector<ThrowStabilizerData> get_stabilizer_data_left_foot(ThrowResponse & response);
        std::vector<ThrowStabilizerData> get_stabilizer_data_right_foot(ThrowResponse & response);

        IKMode last_ik_mode_;
        std::map<std::string, std::shared_ptr<ThrowIK>> map_ik_;
        std::shared_ptr<ThrowDebugParameter> sp_debug_parameter_;
    };
}
#endif //BITBOTS_THROW_THROW_STABILIZER_AND_IK_FACTORY_H

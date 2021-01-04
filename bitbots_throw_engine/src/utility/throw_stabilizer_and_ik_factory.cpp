#include "utility/throw_stabilizer_and_ik_factory.h"
#include "ros_interface/publisher/system_publisher.h"
#include "ros_interface/ros_joint_and_topic_names.h"

namespace bitbots_throw{
    ThrowStabilizerAndIKFactory::ThrowStabilizerAndIKFactory(){
        last_ik_mode_ = IKMode::arms_and_legs_separated;

        map_ik_[RosJointAndTopicNames::get_joint_group_left_arm()] = std::make_shared<ThrowIK>(
                RosJointAndTopicNames::get_joint_group_left_arm()
                , std::vector<std::string>{RosJointAndTopicNames::get_joint_l_elbow(), RosJointAndTopicNames::get_joint_l_shoulder_pitch(),
                   RosJointAndTopicNames::get_joint_l_shoulder_roll()}
                , std::vector<double>{0.0, 0.0, 0.0});

        map_ik_[RosJointAndTopicNames::get_joint_group_right_arm()] = std::make_shared<ThrowIK>(
                RosJointAndTopicNames::get_joint_group_right_arm()
                , std::vector<std::string>{RosJointAndTopicNames::get_joint_r_elbow(), RosJointAndTopicNames::get_joint_r_shoulder_pitch(),
                   RosJointAndTopicNames::get_joint_r_shoulder_roll()}
                , std::vector<double>{0.0, 0.0, 0.0});

        map_ik_[RosJointAndTopicNames::get_joint_group_left_arm()] = std::make_shared<ThrowIK>(
                RosJointAndTopicNames::get_joint_group_left_arm()
                , std::vector<std::string>{RosJointAndTopicNames::get_joint_l_elbow()}
                , std::vector<double>{0.0});

        map_ik_[RosJointAndTopicNames::get_joint_group_right_arm()] = std::make_shared<ThrowIK>(
                RosJointAndTopicNames::get_joint_group_right_arm()
                , std::vector<std::string>{RosJointAndTopicNames::get_joint_r_elbow()}
                , std::vector<double>{0.0});

        map_ik_[RosJointAndTopicNames::get_joint_group_left_leg()] = std::make_shared<ThrowIK>(
                RosJointAndTopicNames::get_joint_group_left_leg()
                , std::vector<std::string>{RosJointAndTopicNames::get_joint_l_hip_pitch(), RosJointAndTopicNames::get_joint_l_knee(),
                   RosJointAndTopicNames::get_joint_l_ankle_pitch()}
                , std::vector<double>{0.7, 1.0, -0.4});

        map_ik_[RosJointAndTopicNames::get_joint_group_right_leg()] = std::make_shared<ThrowIK>(
                RosJointAndTopicNames::get_joint_group_right_leg()
                , std::vector<std::string>{RosJointAndTopicNames::get_joint_r_hip_pitch(), RosJointAndTopicNames::get_joint_r_knee(),
                   RosJointAndTopicNames::get_joint_r_ankle_pitch()}
                , std::vector<double>{-0.7, -1.0, 0.4});
    }

    void ThrowStabilizerAndIKFactory::set_bio_ik_timeout(double timeout){
        for(auto it : map_ik_){
            it.second->set_bio_ik_timeout(timeout);
        }
    }

    void ThrowStabilizerAndIKFactory::set_kinematic_model(moveit::core::RobotModelPtr kinematic_model){
        for(auto it : map_ik_){
            it.second->init(kinematic_model);
        }
    }

    void ThrowStabilizerAndIKFactory::set_debug_parameter(std::shared_ptr<ThrowDebugParameter> debug_parameter){
        sp_debug_parameter_ = debug_parameter;
    }

    bitbots_splines::JointGoals ThrowStabilizerAndIKFactory::calculate_joint_goals(
            bitbots_throw::ThrowResponse & response){
        ThrowStabilizer stabilizer;
        bitbots_splines::JointGoals joint_goals;
        std::vector<std::pair<std::string, std::vector<ThrowStabilizerData>>> ik_data;

        try{
            switch(response.ik_mode_){
                case IKMode::arms_and_legs_separated:
                    if(sp_debug_parameter_->debug_active_ && last_ik_mode_ != response.ik_mode_){
                        SystemPublisher::publish_info("each arm get one ik", "ThrowNode::calculate_joint_goals");
                        last_ik_mode_ = response.ik_mode_;
                    }
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_left_arm()
                                                        , get_stabilizer_data_left_hand(response)));
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_right_arm()
                                                        , get_stabilizer_data_right_hand(response)));
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_left_leg()
                                                        , get_stabilizer_data_left_foot(response)));
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_right_leg()
                                                        , get_stabilizer_data_right_foot(response)));
                    break;
                case IKMode::hands_only_and_legs_separated:
                    if(sp_debug_parameter_->debug_active_ && last_ik_mode_ != response.ik_mode_){
                        SystemPublisher::publish_info("each hand get one ik", "ThrowNode::calculate_joint_goals");
                        last_ik_mode_ = response.ik_mode_;
                    }
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_left_arm()
                                                        , get_stabilizer_data_left_hand(response)));
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_right_arm()
                                                        , get_stabilizer_data_right_hand(response)));
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_left_leg()
                                                        , get_stabilizer_data_left_foot(response)));
                    ik_data.emplace_back(std::make_pair(RosJointAndTopicNames::get_joint_group_right_leg()
                                                        , get_stabilizer_data_right_foot(response)));
                    break;
            }

            for(auto & it : ik_data){
                auto ik_goals = stabilizer.stabilize(it.second);
                auto calc_goals = map_ik_[it.first]->calculate(std::move(ik_goals));
                joint_goals.first.insert(joint_goals.first.end(), calc_goals.first.begin(), calc_goals.first.end());
                joint_goals.second.insert(joint_goals.second.end(), calc_goals.second.begin(), calc_goals.second.end());
            }
        }
        catch(const std::runtime_error & e){
            SystemPublisher::publish_runtime_error(e);

            // maybe add some more diagnostic logic
            joint_goals = bitbots_splines::JointGoals();
        }

        return joint_goals;
    }

    std::vector<ThrowStabilizerData> ThrowStabilizerAndIKFactory::get_stabilizer_data_left_hand(
            ThrowResponse & response){
        return {{response.support_foot_to_left_hand_, RosJointAndTopicNames::get_joint_l_wrist(), RosJointAndTopicNames::get_joint_base_link(), 1}};
    }

    std::vector<ThrowStabilizerData> ThrowStabilizerAndIKFactory::get_stabilizer_data_right_hand(
            ThrowResponse & response){
        return {{response.support_foot_to_right_hand_, RosJointAndTopicNames::get_joint_r_wrist(), RosJointAndTopicNames::get_joint_base_link(), 1}};
    }

    std::vector<ThrowStabilizerData> ThrowStabilizerAndIKFactory::get_stabilizer_data_left_foot(
            ThrowResponse & response){
        return {{response.support_foot_to_left_foot_, RosJointAndTopicNames::get_joint_l_sole(), RosJointAndTopicNames::get_joint_base_link(), 1}};
    }

    std::vector<ThrowStabilizerData> ThrowStabilizerAndIKFactory::get_stabilizer_data_right_foot(
            ThrowResponse & response){
        return {{response.support_foot_to_right_foot_, RosJointAndTopicNames::get_joint_r_sole(), RosJointAndTopicNames::get_joint_base_link(), 1}};
    }
}

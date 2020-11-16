#ifndef BITBOTS_THROW_ROS_JOINT_AND_TOPIC_NAMES_H
#define BITBOTS_THROW_ROS_JOINT_AND_TOPIC_NAMES_H

#include <string>

namespace bitbots_throw{
    class RosJointAndTopicNames{
    public:
        static std::string get_topic_controller_command(){return "/DynamixelController/command";};
        static std::string get_topic_odometry(){return "/throw_odometry";};
        static std::string get_topic_debug(){return "/throw_debug";};
        static std::string get_topic_debug_marker(){return "/throw_debug_marker";};
        static std::string get_topic_support(){return "/throw_support_foot_state";};
        static std::string get_topic_debug_visualization_base(){return "/throw_debug";};
        static std::string get_topic_throw(){return "/throw";};

        static std::string get_topic_suffix_left_hand(){return "_left_hand_marker";};
        static std::string get_topic_suffix_right_hand(){return "_right_hand_marker";};
        static std::string get_topic_suffix_left_foot(){return "_left_foot_marker";};
        static std::string get_topic_suffix_right_foot(){return "_right_foot_marker";};
        static std::string get_topic_suffix_left_hand_arrow(){return "_left_hand_arrow_marker";};
        static std::string get_topic_suffix_right_hand_arrow(){return "_right_hand_arrow_marker";};
        static std::string get_topic_suffix_left_foot_arrow(){return "_left_foot_arrow_marker";};
        static std::string get_topic_suffix_right_foot_arrow(){return "_right_foot_arrow_marker";};

        static std::string get_joint_base_link(){return "base_link";};
        static std::string get_joint_l_wrist(){return "l_wrist";};
        static std::string get_joint_r_wrist(){return "r_wrist";};
        static std::string get_joint_l_sole(){return "l_sole";};
        static std::string get_joint_r_sole(){return "r_sole";};
        static std::string get_joint_head(){return "head";};
        static std::string get_joint_l_elbow(){return "LElbow";};
        static std::string get_joint_l_shoulder_pitch(){return "LShoulderPitch";};
        static std::string get_joint_l_shoulder_roll(){return "LShoulderRoll";};
        static std::string get_joint_r_elbow(){return "RElbow";};
        static std::string get_joint_r_shoulder_pitch(){return "RShoulderPitch";};
        static std::string get_joint_r_shoulder_roll(){return "RShoulderRoll";};
        static std::string get_joint_l_hip_pitch(){return "LHipPitch";};
        static std::string get_joint_l_knee(){return "LKnee";};
        static std::string get_joint_l_ankle_pitch(){return "LAnklePitch";};
        static std::string get_joint_r_hip_pitch(){return "RHipPitch";};
        static std::string get_joint_r_knee(){return "RKnee";};
        static std::string get_joint_r_ankle_pitch(){return "RAnklePitch";};

        static std::string get_joint_group_left_arm(){return "LeftArm";};
        static std::string get_joint_group_right_arm(){return "RightArm";};
        static std::string get_joint_group_left_leg(){return "LeftLeg";};
        static std::string get_joint_group_right_leg(){return "RightLeg";};
    };
}
#endif //BITBOTS_THROW_ROS_JOINT_AND_TOPIC_NAMES_H

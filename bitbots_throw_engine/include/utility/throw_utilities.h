#ifndef BITBOTS_THROW_THROW_UTILITIES_H
#define BITBOTS_THROW_THROW_UTILITIES_H

#include <string>
#include "parameter/struct3d.h"
#include "tf2/LinearMath/Transform.h"
#include "../../bitbots_splines_extension/include/spline/curve.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
    enum class CurvePurpose{
        trunk,
        left_foot,
        right_foot,
        left_hand,
        right_hand,
        arms,
        left_elbow,
        left_shoulder_pitch,
        left_shoulder_roll,
        right_elbow,
        right_shoulder_pitch,
        right_shoulder_roll
    };

    struct ThrowRequest{
        Struct3d ball_position_;
        Struct3d goal_position_;

        Struct3d left_hand_position_;
        Struct3d right_hand_position_;
        Struct3d left_feet_position_;
        Struct3d right_feet_position_;
        Struct3d head_position_;
    };

    struct ThrowResponse{
        tf2::Transform support_foot_to_left_hand_;
        tf2::Transform support_foot_to_right_hand_;
        tf2::Transform support_foot_to_trunk_;
        tf2::Transform support_foot_to_left_foot_;
        tf2::Transform support_foot_to_right_foot_;

        // additional information for visualization
        double phase_;
        double traj_time_;
    };

    static std::string get_joint_name(CurvePurpose joint){
        std::string joint_name;

        switch(joint){
            case CurvePurpose::arms:
                joint_name = "Arms";
                break;
            case CurvePurpose::left_elbow:
                joint_name = "LElbow";
                break;
            case CurvePurpose::left_shoulder_pitch:
                joint_name = "LShoulderPitch";
                break;
            case CurvePurpose::left_shoulder_roll:
                joint_name = "LShoulderRoll";
                break;
            case CurvePurpose::right_elbow:
                joint_name = "RElbow";
                break;
            case CurvePurpose::right_shoulder_pitch:
                joint_name = "RShoulderPitch";
                break;
            case CurvePurpose::right_shoulder_roll:
                joint_name = "RShoulderRoll";
                break;
            default:
                joint_name = "";
                break;
        }

        return joint_name;
    };

    static std::string generate_points_csv(bitbots_splines::Curve * curve){
        std::stringstream output, position;

        output << "time";
        position << "position";
        for(double time = 0.0; time < 2.1; time += 0.1){
            output << ", " << time;
            position << ", " << curve->position(time);
        }
        output << std::endl << position.str() << std::endl;

        return output.str();
    }

    static std::string generate_points_csv(bitbots_splines::PositionHandle handle, double const & duration){
        std::stringstream output, x, y, z;

        output << "time";
        x << "x-position";
        y << "y-position";
        z << "z-position";
        for(double time = 0.0; time < duration; time += 0.1){
            output << ", " << time;
            auto position = handle.get_position(time);
            x << ", " << position.x();
            y << ", " << position.y();
            z << ", " << position.z();
        }
        output << std::endl << x.str() << std::endl << y.str() << std::endl << z.str() << std::endl;

        return output.str();
    }
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_UTILITIES_H
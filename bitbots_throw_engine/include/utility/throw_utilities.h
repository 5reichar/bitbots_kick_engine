#ifndef BITBOTS_THROW_THROW_UTILITIES_H
#define BITBOTS_THROW_THROW_UTILITIES_H

#include <string>
#include "throw_struct.h"
#include "../../bitbots_splines_extension/include/spline/curve.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
    static void add_point(std::shared_ptr<bitbots_splines::PoseHandle> & pose, double const & time, Struct3dRPY const & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        pose->x()->add_point(time, position.x_, velocity.x_, acceleration.x_);
        pose->y()->add_point(time, position.y_, velocity.y_, acceleration.y_);
        pose->z()->add_point(time, position.z_, velocity.z_, acceleration.z_);
        pose->roll()->add_point(time, position.roll_, velocity.roll_, acceleration.roll_);
        pose->pitch()->add_point(time, position.pitch_, velocity.pitch_, acceleration.pitch_);
        pose->yaw()->add_point(time, position.yaw_, velocity.yaw_, acceleration.yaw_);
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

    static std::string generate_points_csv(bitbots_splines::PoseHandle handle, double const & duration){
        std::stringstream output, x, y, z, roll, pitch, yaw;

        output << "time";
        x << "x-position";
        y << "y-position";
        z << "z-position";
        roll << "roll-position";
        pitch << "pitch-position";
        yaw << "yaw-position";
        for(double time = 0.0; time < duration; time += 0.1){
            output << ", " << time;
            auto position = handle.get_position(time);
            x << ", " << position.x();
            y << ", " << position.y();
            z << ", " << position.z();
            auto orientation = handle.get_orientation(time);
            roll << ", " << orientation.x();
            pitch << ", " << orientation.y();
            yaw << ", " << orientation.z();
        }
        output << std::endl << x.str() << std::endl << y.str() << std::endl << z.str() << std::endl;
        output << std::endl << roll.str() << std::endl << pitch.str() << std::endl << yaw.str() << std::endl;

        return output.str();
    }

    template<class c> std::shared_ptr<bitbots_splines::PoseHandle> create_pose_handle(){
        return std::make_shared<bitbots_splines::PoseHandle>(
                std::make_shared<c>() // x
                ,std::make_shared<c>() // y
                ,std::make_shared<c>() // z
                ,std::make_shared<c>() // roll
                ,std::make_shared<c>() // pitch
                ,std::make_shared<c>()); // yaw
    }
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_UTILITIES_H
#ifndef BITBOTS_THROW_THROW_CURVE_H
#define BITBOTS_THROW_THROW_CURVE_H

#include <memory>
#include "parameter/throw_parameter.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
	class ThrowCurve{
	public:
		ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
		          ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
				  ,std::shared_ptr<bitbots_splines::PoseHandle> trunk
                  ,std::shared_ptr<bitbots_splines::PoseHandle> left_feet
                  ,std::shared_ptr<bitbots_splines::PoseHandle> right_feet);

		/**
		 * Calculates the trajectory for the throw movement.
		 *
		 * @param throw_parameter Container with all values need for the calculate.
		 * @return duration of the movement. Returns 0.0 if the check for the parameter fails.
		 */
		double calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter);
		[[nodiscard]] std::shared_ptr<bitbots_splines::PoseHandle> get_pose_left_hand() const;
		[[nodiscard]] std::shared_ptr<bitbots_splines::PoseHandle> get_pose_right_hand() const;
        [[nodiscard]] std::shared_ptr<bitbots_splines::PoseHandle> get_pose_trunk() const;
        [[nodiscard]] std::shared_ptr<bitbots_splines::PoseHandle> get_pose_left_feet() const;
        [[nodiscard]] std::shared_ptr<bitbots_splines::PoseHandle> get_pose_right_feet() const;

	protected:
		virtual bool check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter);

		virtual void calculate_pick_up_ball_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);
        virtual void calculate_throw_preparation_movement(double & time
                                                         ,std::shared_ptr<ThrowParameter> & throw_parameter);
        virtual void calculate_throw_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);
		virtual void calculate_throw_conclusion_movement(double & time
		                                                ,std::shared_ptr<ThrowParameter> & throw_parameter);

        virtual void add_points(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                               ,double const & time
                               ,Struct3dRPY const & values);

		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_hand_;
		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_hand_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_trunk_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_feet_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_feet_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_CURVE_H
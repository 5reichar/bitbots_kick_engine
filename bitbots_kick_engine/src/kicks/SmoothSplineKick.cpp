#include <SmoothSplineKick.hpp>

bool SmoothSplineKick::additional_requirements()
{
	return false;
}

void SmoothSplineKick::build_trajectories()
{
	// save the current trunk state to use it later
	if (!startStep)
	{
		saveCurrentTrunkState();
	}
	else
	{
		_trunkPosAtLast.y() -= _footstep.getNext().y();
		//trunkPos = Eigen::Rotation2Dd(-_footstep.getNext().z()).toRotationMatrix() * trunkPos;
	}

	if (startStep)
	{
		// update support foot and compute odometry
		_footstep.stepFromOrders(Eigen::Vector3d());
	}
	else
	{
		_footstep.stepFromOrders(orders);
	}

	//Reset the trajectories
	_trajs = bitbots_splines::TrajectoriesInit();
	//Set up the trajectories for the half cycle (single step)
	double halfPeriod = 1.0 / (2.0 * _params.freq);
	// full period (double step) is needed for trunk splines
	double period = 2.0 * halfPeriod;

	//Time length of double and single support phase during the half cycle
	double doubleSupportLength = _params.doubleSupportRatio * halfPeriod;
	double singleSupportLength = halfPeriod - doubleSupportLength;

	//Sign of support foot with respect to lateral
	double supportSign = (_footstep.isLeftSupport() ? 1.0 : -1.0);

	//The trunk trajectory is defined for a
	//complete cycle to handle trunk phase shift
	//Trunk phase shift is done due to the length of the double 
	//support phase and can be adjusted optionally by a parameter
	// 0.5halfPeriod to be acyclic to the feet, 0.5doubleSupportLength to keep the double support phase centered between feet
	double timeShift = -0.5 * halfPeriod + 0.5 * doubleSupportLength + _params.trunkPhase * halfPeriod;


	//Only move the trunk on the first half cycle after a walk enable
	if (startStep) {
		doubleSupportLength = halfPeriod;
		singleSupportLength = 0.0;
	}
	//Set double support phase
	point("is_double_support", 0.0, 1.0);
	point("is_double_support", doubleSupportLength, 1.0);
	point("is_double_support", doubleSupportLength, 0.0);
	point("is_double_support", halfPeriod, 0.0);

	//Set support foot
	point("is_left_support_foot", 0.0, _footstep.isLeftSupport());
	point("is_left_support_foot", halfPeriod, _footstep.isLeftSupport());

	//Flying foot position
	point("foot_pos_x", 0.0, _footstep.getLast().x());
	point("foot_pos_x", doubleSupportLength, _footstep.getLast().x());
	if (kickStep)
	{
		point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.kickPhase, _footstep.getNext().x() + _params.kickLength, _params.kickVel);
	}
	else
	{
		point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase * _params.footOvershootPhase, _footstep.getNext().x() + (_footstep.getNext().x() - _footstep.getLast().x()) * _params.footOvershootRatio);
	}
	point("foot_pos_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, _footstep.getNext().x());
	point("foot_pos_x", halfPeriod, _footstep.getNext().x());

	point("foot_pos_y", 0.0, _footstep.getLast().y());
	point("foot_pos_y", doubleSupportLength, _footstep.getLast().y());
	point("foot_pos_y", doubleSupportLength + singleSupportLength * _params.footPutDownPhase * _params.footOvershootPhase, _footstep.getNext().y() + (_footstep.getNext().y() - _footstep.getLast().y()) * _params.footOvershootRatio);
	point("foot_pos_y", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, _footstep.getNext().y());
	point("foot_pos_y", halfPeriod, _footstep.getNext().y());

	point("foot_pos_z", 0.0, 0.0);
	point("foot_pos_z", doubleSupportLength, 0.0);
	point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footApexPhase - 0.5 * _params.footZPause * singleSupportLength, _params.footRise);
	point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footApexPhase + 0.5 * _params.footZPause * singleSupportLength, _params.footRise);
	point("foot_pos_z", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, _params.footPutDownZOffset);
	point("foot_pos_z", halfPeriod, 0.0);

	//Flying foot orientation
	point("foot_axis_x", 0.0, 0.0);
	point("foot_axis_x", doubleSupportLength + 0.1 * singleSupportLength, 0.0);
	point("foot_axis_x", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, _params.footPutDownRollOffset * supportSign);
	point("foot_axis_x", halfPeriod, 0.0);

	point("foot_axis_y", 0.0, 0.0);
	point("foot_axis_y", halfPeriod, 0.0);

	point("foot_axis_z", 0.0, _footstep.getLast().z());
	point("foot_axis_z", doubleSupportLength, _footstep.getLast().z());
	point("foot_axis_z", doubleSupportLength + singleSupportLength * _params.footPutDownPhase, _footstep.getNext().z());
	point("foot_axis_z", halfPeriod, _footstep.getNext().z());


	//Half pause length of trunk swing 
	//lateral oscillation
	double pauseLength = 0.5 * _params.trunkPause * halfPeriod;

	//Trunk support foot and next 
	//support foot external 
	//oscillating position 
	Eigen::Vector2d trunkPointSupport(_params.trunkXOffset + _params.trunkXOffsetPCoefForward * _footstep.getNext().x() + _params.trunkXOffsetPCoefTurn * fabs(_footstep.getNext().z()), _params.trunkYOffset);
	Eigen::Vector2d trunkPointNext(_footstep.getNext().x() + _params.trunkXOffset + _params.trunkXOffsetPCoefForward * _footstep.getNext().x() + _params.trunkXOffsetPCoefTurn * fabs(_footstep.getNext().z()), _footstep.getNext().y() + _params.trunkYOffset);
	//Trunk middle neutral (no swing) position
	Eigen::Vector2d trunkPointMiddle = 0.5 * trunkPointSupport + 0.5 * trunkPointNext;
	//Trunk vector from middle to support apex
	Eigen::Vector2d trunkVect = trunkPointSupport - trunkPointMiddle;
	//Apply swing amplitude ratio
	trunkVect.y() *= _params.trunkSwing;
	//Trunk support and next apex position
	Eigen::Vector2d trunkApexSupport = trunkPointMiddle + trunkVect;
	Eigen::Vector2d trunkApexNext = trunkPointMiddle - trunkVect;
	//Trunk forward velocity
	double trunkVelSupport = (_footstep.getNext().x() - _footstep.getLast().x()) / period;
	double trunkVelNext = _footstep.getNext().x() / halfPeriod;

	//Trunk position
	point("trunk_pos_x", 0.0, _trunkPosAtLast.x(), _trunkVelAtLast.x(), _trunkAccAtLast.x());
	point("trunk_pos_x", halfPeriod + timeShift, trunkApexSupport.x(), trunkVelSupport);
	point("trunk_pos_x", period + timeShift, trunkApexNext.x(), trunkVelNext);

	point("trunk_pos_y", 0.0, _trunkPosAtLast.y(), _trunkVelAtLast.y(), _trunkAccAtLast.y());
	point("trunk_pos_y", halfPeriod + timeShift - pauseLength, trunkApexSupport.y());
	point("trunk_pos_y", halfPeriod + timeShift + pauseLength, trunkApexSupport.y());
	point("trunk_pos_y", period + timeShift - pauseLength, trunkApexNext.y());
	point("trunk_pos_y", period + timeShift + pauseLength, trunkApexNext.y());

	point("trunk_pos_z", 0.0, _trunkPosAtLast.z(), _trunkVelAtLast.z(), _trunkAccAtLast.z());
	point("trunk_pos_z", halfPeriod + timeShift, _params.trunkHeight);
	point("trunk_pos_z", period + timeShift, _params.trunkHeight);

	//Define trunk yaw target 
	//orientation position and velocity
	//in euler angle and convertion
	//to axis vector
	Eigen::Vector3d eulerAtSuport(0.0, _params.trunkPitch + _params.trunkPitchPCoefForward * _footstep.getNext().x() + _params.trunkPitchPCoefTurn * fabs(_footstep.getNext().z()), 0.5 * _footstep.getLast().z() + 0.5 * _footstep.getNext().z());
	Eigen::Vector3d eulerAtNext(0.0, _params.trunkPitch + _params.trunkPitchPCoefForward * _footstep.getNext().x() + _params.trunkPitchPCoefTurn * fabs(_footstep.getNext().z()), _footstep.getNext().z());
	Eigen::Matrix3d matAtSupport = bitbots_splines::EulerIntrinsicToMatrix(eulerAtSuport);
	Eigen::Matrix3d matAtNext = bitbots_splines::EulerIntrinsicToMatrix(eulerAtNext);
	Eigen::Vector3d axisAtSupport = bitbots_splines::MatrixToAxis(matAtSupport);
	Eigen::Vector3d axisAtNext = bitbots_splines::MatrixToAxis(matAtNext);
	Eigen::Vector3d axisVel(0.0, 0.0, bitbots_splines::AngleDistance(_footstep.getLast().z(), _footstep.getNext().z()) / period);

	//Trunk orientation
	point("trunk_axis_x", 0.0, _trunkAxisPosAtLast.x(), _trunkAxisVelAtLast.x(), _trunkAxisAccAtLast.x());
	point("trunk_axis_x", halfPeriod + timeShift, axisAtSupport.x(), axisVel.x());
	point("trunk_axis_x", period + timeShift, axisAtNext.x(), axisVel.x());

	point("trunk_axis_y", 0.0, _trunkAxisPosAtLast.y(), _trunkAxisVelAtLast.y(), _trunkAxisAccAtLast.y());
	point("trunk_axis_y", halfPeriod + timeShift, axisAtSupport.y(), axisVel.y());
	point("trunk_axis_y", period + timeShift, axisAtNext.y(), axisVel.y());

	point("trunk_axis_z", 0.0, _trunkAxisPosAtLast.z(), _trunkAxisVelAtLast.z(), _trunkAxisAccAtLast.z());
	point("trunk_axis_z", halfPeriod + timeShift, axisAtSupport.z(), axisVel.z());
	point("trunk_axis_z", period + timeShift, axisAtNext.z(), axisVel.z());
}

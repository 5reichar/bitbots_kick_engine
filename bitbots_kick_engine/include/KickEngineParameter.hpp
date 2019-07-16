struct KickEngineParameter
{
	//Full walk cycle frequency
	//(in Hz, > 0)
	double freq;
	//Length of double support phase in half cycle
	//(ratio, [0:1])
	double doubleSupportRatio;
	//Lateral distance between the feet center
	//(in m, >= 0)
	double footDistance;
	//Maximum flying foot height
	//(in m, >= 0)
	double footRise;
	// Pause of Z movement on highest point
	//(single support cycle ratio, [0,1])
	double footZPause;
	//Let the foot's downward trajectory end above the ground
	//this is helpful if the support leg bends
	//(in m, >= 0)
	double footPutDownZOffset;
	//Phase time for moving the foot from Z offset to ground,
	// also used for X and Y since they should not move after contact to the ground
	//(phase between apex and single support end [0:1])
	double footPutDownPhase;
	//Phase of flying foot apex
	//(single support cycle phase, [0:1])
	double footApexPhase;
	//Foot X/Y overshoot in ratio of step length
	//(ratio, >= 0)
	double footOvershootRatio;
	//Foot X/Y overshoot phase
	//(single support cycle phase, [footApexPhase:1]
	double footOvershootPhase;
	//Height of the trunk from ground
	//(in m, > 0)
	double trunkHeight;
	//Trunk pitch orientation
	//(in rad)
	double trunkPitch;
	//Phase offset of trunk oscillation
	//(half cycle phase, [0:1])
	double trunkPhase;
	//Trunk forward offset
	//(in m)
	double trunkXOffset;
	//Trunk lateral offset
	//(in m)
	double trunkYOffset;
	//Trunk lateral oscillation amplitude ratio
	//(ratio, >= 0)
	double trunkSwing;
	//Trunk swing pause length in phase at apex
	//(half cycle ratio, [0:1])
	double trunkPause;
	//Trunk forward offset proportional to forward step
	//(in 1)
	double trunkXOffsetPCoefForward;
	//Trunk forward offset proportional to rotation step
	//(in m/rad)
	double trunkXOffsetPCoefTurn;
	//Trunk pitch orientation proportional to forward step
	//(in rad/m)
	double trunkPitchPCoefForward;
	//Trunk pitch orientation proportional to rotation step
	//(in 1)
	double trunkPitchPCoefTurn;
	double trunkYOnlyInDoubleSupport;
	double kickLength;
	double kickPhase;
	double footPutDownRollOffset;
	double kickVel;
	double pauseDuration;

	double engineFrequency;

	bool phaseResetActive;
	double groundMinPressure;
	bool copStopActive;
	double ioPressureThreshold;
	double fbPressureThreshold;

	bool imuActive;
	double imu_pitch_threshold;
	double imu_roll_threshold;

	Eigen::Vector3d _max_step;
	double _max_step_xy;
};

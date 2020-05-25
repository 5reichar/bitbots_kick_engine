#ifndef STRUCT3D_H
#define STRUCT3D_H

struct Struct3d
{
	double x_;
	double y_;
	double z_;


	//////		Constructor
	Struct3d(double x, double y, double z) 
			: x_{x}, y_{y}, z_{z}
	{}
};

struct Struct3dRPY
{
	double roll_;
	double pitch_;
	double yaw_;


	//////		Constructor
	Struct3dRPY(double roll, double pitch, double yaw)
				: roll_{roll}, pitch_{pitch}, yaw_{yaw}
	{}
};

#endif



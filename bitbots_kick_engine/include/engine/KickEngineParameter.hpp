#ifndef KICKENGINEPARAMETER_HPP
#define KICKENGINEPARAMETER_HPP

struct KickEngineParameter
{
	//Full walk cycle frequency
	//(in Hz, > 0)
	double freq;
	//Lateral distance between the feet center
	//(in m, >= 0)
	double footDistance;
	double engineFrequency;
};

#endif
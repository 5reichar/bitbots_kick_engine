#ifndef THROW_NODE_PARAMETER_H
#define THROW_NODE_PARAMETER_H

struct ThrowNodeParameter
{
    bool debug_active_;
    // Max frequency of engine update rate [hz]
	double engine_frequency_;
    // Publish odom every [int] update of the engine
    int odom_publish_factor_;
    // Timeout time for bioIK [s]
    double bio_ik_time_;
};

#endif
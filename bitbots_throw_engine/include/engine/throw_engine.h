#ifndef THROW_ENGINE_H
#define THROW_ENGINE_H

#include "throws/throw_factory.h"

class ThrowEngine
{
public:
    ThrowEngine();

	virtual bool throw_ball(struct3d & ball_position, struct3d & goal_position);

private:
    std::shared_ptr<ThrowFactory> sp_throw_factory_;
};

#endif
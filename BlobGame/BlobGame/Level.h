#pragma once

#include "Box2D\Box2D.h"

#include "Blob.h"


class Level
{
public:
	b2Vec2* gravity;
	b2World* world;
	Blob* blob;

	Level();
	~Level();

	void Step(float32 physics_tick);
	
};


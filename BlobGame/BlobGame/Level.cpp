#include "Level.h"



Level::Level()
{
	gravity = new b2Vec2(0.0f, -1.0f);
	world = new b2World(*gravity);

	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(0.0f, 0.0f);
	bodyDef.angle = 0.0f;
	bodyDef.angularVelocity = 1.0f;
	b2Body* body = world->CreateBody(&bodyDef);
	b2PolygonShape boxShape;
	boxShape.SetAsBox(0.8f, 0.5f);
	b2FixtureDef boxFixtureDef;
	boxFixtureDef.shape = &boxShape;
	boxFixtureDef.density = 1.0f;
	body->CreateFixture(&boxFixtureDef);

	blob = new Blob(world);
	
}


Level::~Level()
{
	delete blob;
	//delete world;
	delete gravity;
}

void Level::Step(float32 physics_tick) {
	world->Step(physics_tick, 6, 2);
}

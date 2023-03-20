#pragma once

#include "GL\glew.h"
#include <GLFW/glfw3.h> // for timing only - remove later

#include "Box2D\Box2D.h"

#include "Shader.h"

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include "hullfinder3.h"

#include "ConcaveHull.h"


class Blob
{
private:
	b2World* world; //owner
	b2ParticleSystem* particleSystem;
	b2CircleShape* shape;
	b2ParticleGroupDef* pd;
	const b2ParticleSystemDef particleSystemDef;
	size_t particleCount;
	size_t elements_count, vertices_size; //elements (not particles!) in array "vertices", size of array
	b2Vec2 center;
	size_t bezier_frags = 10;

	GLfloat* vertices;
	GLuint VBO, VAO;

	Shader* shader;

	//HullFinder hullFinder;
	std::vector<b2Vec2> hull;
	//PointVector hull2;
	std::vector<b2Vec2> positions; // of particles
	//PointVector positions2;
	std::vector<b2Vec2> cpa; // bezier cubic control points for smoother hull
	std::vector<b2Vec2> cpb;
	std::vector<b2Vec2> cpc; // not used, except for in findControlPoints (unused)
	std::vector<std::vector<double>> bezierCoeffs;
	std::vector<b2Vec2> hull_vec;
	std::vector<b2Vec2> hull_vec_norm; //same as hull_vec, but normalized
	std::vector<double> hull_vec_dist; // length of hull_vec
	std::vector<b2Vec2> hull_c; //controlling lines for bezier curves. sets direction at hull points as average of direction to adjacent points

	//void findControlPoints();

	void findControlPoints2();

	void findBezierCoeffs(int n);

	Point intersection(Point p0, Point p1, Point v0, Point v1); //intersection between lines through p0/p1 and with directions v0/v1
	b2Vec2 intersection(b2Vec2 p0, b2Vec2 p1, b2Vec2 v0, b2Vec2 v1); //intersection between lines through p0/p1 and with directions v0/v1
	
public:
	Blob(b2World* world);
	~Blob();

	b2Vec2* getPositionBuffer();
	size_t getParticleCount();

	void draw(glm::mat4 projection);
	void findHull(); //Testing function for hullfinder
	
};


#include "Blob.h"
#include <random>
#include <assert.h>




Blob::Blob(b2World* world)
{
	this->world = world;
	
	particleSystem = world->CreateParticleSystem(&particleSystemDef);
	particleSystem->SetRadius(0.05f); // radius of particles

	shape = new b2CircleShape();
	shape->m_p.Set(0, 2.0f);
	shape->m_radius = 0.5f;
	pd = new b2ParticleGroupDef();

	pd->flags = b2_springParticle;
	pd->groupFlags = b2_solidParticleGroup;
	pd->shape = shape;
	pd->color.Set(255, 0, 0, 255);
	particleSystem->CreateParticleGroup(*pd);

	particleCount = particleSystem->GetParticleCount();
	elements_count = (particleCount + 2) * 6; // 2 extra for (0,0) and repeat first for last in triangle fan; 6: xyzrgb, consider removing zrgb once no longer needed for debugging;
	vertices_size = elements_count * sizeof(GLfloat); 

	findBezierCoeffs(bezier_frags);

	shader = new Shader("core.vert", "core.frag");

	vertices = new GLfloat[elements_count * (4 + bezier_frags)]; //5: positions, hull, cpa, cpb, b0-b9

	center.x = 0.0f;
	center.y = 0.0f;

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices_size * (4 + bezier_frags), vertices, GL_STREAM_DRAW);
	//Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	//Color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}


Blob::~Blob()
{
	world->DestroyParticleSystem(particleSystem);
	delete shader;
	delete vertices;
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
}

b2Vec2 * Blob::getPositionBuffer()
{
	return particleSystem->GetPositionBuffer();
}

size_t Blob::getParticleCount()
{
	return particleSystem->GetParticleCount();
}

/*void Blob::findControlPoints() { // not used.
	cpa.resize(hull.size());
	cpb.resize(hull.size());
	cpc.resize(hull.size());
	
	Point vi, vim1, vim2; // v_i, v_i-1, v_i-2
	Point pi, pip1, pim1; //p_i, p_i+1
	double d2; //squared distance from p to cpa/cpb
	double d2max = 100.0f; // max d2. lower near 0 to flatten bezier curves
	double f = 0.5f; // 0 = cpa/cpb at p, 1 = cpa/cpb at cpc
	pi = hull[hull.size() - 1];
	pip1 = hull[0];
	pim1 = hull[hull.size() - 2];
	vi = pip1 - pi;
	vim1 = pi - pim1;
	vim2 = pim1 - hull[hull.size() - 3];
	for (size_t i = 0; i < hull.size(); i++) {
		vim2 = vim1;
		vim1 = vi;
		pim1 = pi;
		pi = pip1;
		pip1 = hull[i + 1 < hull.size() ? i + 1 : i + 1 - hull.size()];
		vi = pip1 - pi;
		int im1 = i > 0 ? i - 1 : hull.size() - 1;
		cpc[im1] = intersection(pi, pim1, vi, vim2);
		// place cpa/b in between p and cpc according to f. check dist. if not ok, ... use dmax
		if (isnan(cpc[im1].x)) {
			cpa[im1] = pim1;
			cpb[im1] = pi;
			continue;
		}
		// cpa
		cpa[im1] = cpc[im1] * f + pim1 * (1 - f);
		d2 = pim1.DistanceSquared(cpa[im1]);
		if (d2 > d2max) {
			double overshoot_factor = sqrt(d2 / d2max);
			cpa[im1] = cpc[im1] * (f / overshoot_factor) + pim1 * (1 - (f / overshoot_factor));
		}
		//repeat for cpb
		cpb[im1] = cpc[im1] * f + pi * (1 - f);
		d2 = pi.DistanceSquared(cpa[im1]);
		if (d2 > d2max) {
			double overshoot_factor = sqrt(d2 / d2max);
			cpb[im1] = cpc[im1] * (f / overshoot_factor) + pi * (1 - (f / overshoot_factor));
		}
	}
}*/

void Blob::findControlPoints2() {
	assert("Hull vector does not contain endpoint identical to startpoint!", hull[0] == hull[hull.size() - 1]);

	size_t hs = hull.size() - 1; // -1 because last item in hull == first
	cpa.resize(hs);
	cpb.resize(hs);
	hull_vec.resize(hs);
	hull_vec_norm.resize(hs);
	hull_vec_dist.resize(hs);
	hull_c.resize(hs);

	// find hull vectors
	for (size_t i = 0; i < hs; i++)
		hull_vec[i] = hull[i + 1] - hull[i];

	// find hull vector lengths
	for (size_t i = 0; i < hs; i++) {
		double x = hull_vec[i].x;
		double y = hull_vec[i].y;
		hull_vec_dist[i] = sqrt(x * x + y * y);
	}
		
	// find normalized hull vectors
	for (size_t i = 0; i < hs; i++)
		hull_vec_norm[i] = hull_vec[i] / hull_vec_dist[i];

	// find controlling lines
	for (size_t i = 1; i < hs; i++)
		hull_c[i] = hull_vec_norm[i - 1] + hull_vec_norm[i];
	hull_c[0] = hull_vec_norm[hs - 1] + hull_vec_norm[0];

	// normalize controlling lines
	for (size_t i = 0; i < hs; i++)
		hull_c[i].Normalize();

	// place control points
	for (size_t i = 0; i < hs; i++)
		cpa[i] = hull[i] + hull_c[i] * hull_vec_dist[i] * 0.33f;
	for (size_t i = 0; i < hs - 1; i++)
		cpb[i] = hull[i + 1] - hull_c[i + 1] * hull_vec_dist[i] * 0.33f;
	cpb[hs - 1] = hull[0] - hull_c[0] * hull_vec_dist[hs - 1] * 0.33f;
}

// Fragment a cubic bezier curve into n bits using these coeffs
void Blob::findBezierCoeffs(int n) {
	bezierCoeffs.resize(n);
	for (int i = 0; i < n; i++) {
		double t = (1.0f / n) * i;
		double t2 = 1.0f - t;
		bezierCoeffs[i].resize(4);
		bezierCoeffs[i][0] = t2 * t2 * t2;
		bezierCoeffs[i][1] = 3 * t2 * t2 * t;
		bezierCoeffs[i][2] = 3 * t2 * t * t;
		bezierCoeffs[i][3] = t * t * t;
	}

}

Point Blob::intersection(Point p0, Point p1, Point v0, Point v1) {
	// intersection of line with direction v0 through p0 and line with direction v1 through p1
	// intersection is at p1 + k1 * v1
	double d = v1.x * v0.y - v1.y * v0.x;
	if (d != 0) {
		double k1 = (v0.y * (p0.x - p1.x) + v0.x * (p1.y - p0.y)) / d;
		return Point(p1.x + v1.x * k1, p1.y + v1.y * k1);
	}
	std::cout << "Error: cannot find intersection" << std::endl;
	return Point(_Nan._Double, _Nan._Double);
}

b2Vec2 Blob::intersection(b2Vec2 p0, b2Vec2 p1, b2Vec2 v0, b2Vec2 v1) {
	// intersection of line with direction v0 through p0 and line with direction v1 through p1
	// intersection is at p1 + k1 * v1
	double d = v1.x * v0.y - v1.y * v0.x;
	if (d != 0) {
		double k1 = (v0.y * (p0.x - p1.x) + v0.x * (p1.y - p0.y)) / d;
		return b2Vec2(p1.x + v1.x * k1, p1.y + v1.y * k1);
	}
	std::cout << "Error: cannot find intersection" << std::endl;
	return b2Vec2(_Nan._Double, _Nan._Double);
}

void Blob::draw(glm::mat4 projection) {
	std::cout << "particle count: " << getParticleCount() << std::endl;
	b2Vec2* positionBuffer = getPositionBuffer();
	positions = std::vector<b2Vec2>(positionBuffer, positionBuffer + /*sizeof b2Vec2 **/ getParticleCount());
	std::cout << "sizeof p: " << sizeof positions;
	/*positions.resize(getParticleCount());
	for (size_t i = 0; i < getParticleCount(); i++) {
		positions[i].x = positionBuffer[i].x;
		positions[i].y = positionBuffer[i].y;
	}
	std::cout << " sizeof p2: " << sizeof positions;*/

	double t = glfwGetTime(), t2;
	hull = HullFinder::FindConvexHull(positions); // find convex hull. first and last value are equal
	//hull = HullFinder::FindConcaveHull(positions, 3.0); //larger value = more convex
	/*positions2.resize(positions.size());
	for (int i = 0; i < positions.size(); i++) {
		positions2[i].x = positions[i].x;
		positions2[i].y = positions[i].y;
		positions2[i].id = 0;
	}
	hull2 = ConcaveHull::ConcaveHullAlg(positions2, 10, true);
	hull.resize(hull2.size());
	for (int i = 0; i < hull2.size(); i++) {
		hull[i].x = hull2[i].x;
		hull[i].y = hull2[i].y;
	}*/
	t2 = glfwGetTime();
	std::cout << "FindHull time: " << (t2 - t) * 1000 << " ms ";
	
	findControlPoints2();
	t = glfwGetTime();
	std::cout << "FindCP time: " << (t - t2) * 1000 << " ms ";
	// all points
	for (size_t i = 0; i < getParticleCount(); i++) {
		vertices[i * 6] = positionBuffer[i].x;
		vertices[i * 6 + 1] = positionBuffer[i].y;
		vertices[i * 6 + 2] = 0.0f;
		vertices[i * 6 + 3] = i < 50 ? 1.0f : 0.5f; // edgep: 0, 1, 6, 7, 8
		vertices[i * 6 + 4] = i < 50 ? 1.0f : 0.5f;
		vertices[i * 6 + 5] = i < 50 ? 1.0f : 0.5f;
	}
	// hull points
	for (size_t i = 0; i < hull.size(); i++) {
		vertices[elements_count + i * 6] = (float)hull[i].x;
		vertices[elements_count + i * 6 + 1] = (float)hull[i].y;
		vertices[elements_count + i * 6 + 2] = 0.0f;
		vertices[elements_count + i * 6 + 3] = 0.0f;
		vertices[elements_count + i * 6 + 4] = (i % 2) * 0.5f;
		vertices[elements_count + i * 6 + 5] = 0.0f;
	}
	// hull bezier curve
	vertices[elements_count * 4] = 0.0f;
	vertices[elements_count * 4 + 1] = 1.2f;
	vertices[elements_count * 4 + 2] = 0.0f;
	vertices[elements_count * 4 + 3] = 0.3f;
	vertices[elements_count * 4 + 4] = 0.3f;
	vertices[elements_count * 4 + 5] = 0.9f;
	for (size_t i = 0; i < hull.size() - 1; i++) {
		for (size_t j = 0; j < bezier_frags; j++) {
			vertices[6 + elements_count * 4 + i * 6 * bezier_frags + j * 6] = (float)(hull[i].x * bezierCoeffs[j][0] + cpa[i].x * bezierCoeffs[j][1] + cpb[i].x * bezierCoeffs[j][2] + hull[i + 1 < hull.size() ? i + 1 : 0].x * bezierCoeffs[j][3]);
			vertices[6 + elements_count * 4 + i * 6 * bezier_frags + j * 6 + 1] = (float)(hull[i].y * bezierCoeffs[j][0] + cpa[i].y * bezierCoeffs[j][1] + cpb[i].y * bezierCoeffs[j][2] + hull[i + 1 < hull.size() ? i + 1 : 0].y * bezierCoeffs[j][3]);
			vertices[6 + elements_count * 4 + i * 6 * bezier_frags + j * 6 + 2] = 0.0f;
			vertices[6 + elements_count * 4 + i * 6 * bezier_frags + j * 6 + 3] = 0.5f + (j % 2) *0.5;
			vertices[6 + elements_count * 4 + i * 6 * bezier_frags + j * 6 + 4] = 0.0f;
			vertices[6 + elements_count * 4 + i * 6 * bezier_frags + j * 6 + 5] = 0.0f;
		}
	}
	vertices[6 + elements_count * 4 + (hull.size() - 1) * 6 * bezier_frags] = vertices[6 + elements_count * 4];
	vertices[6 + elements_count * 4 + (hull.size() - 1) * 6 * bezier_frags + 1] = vertices[6 + elements_count * 4 + 1];
	vertices[6 + elements_count * 4 + (hull.size() - 1) * 6 * bezier_frags + 2] = vertices[6 + elements_count * 4 + 2];
	vertices[6 + elements_count * 4 + (hull.size() - 1) * 6 * bezier_frags + 3] = vertices[6 + elements_count * 4 + 3];
	vertices[6 + elements_count * 4 + (hull.size() - 1) * 6 * bezier_frags + 4] = vertices[6 + elements_count * 4 + 4];
	vertices[6 + elements_count * 4 + (hull.size() - 1) * 6 * bezier_frags + 5] = vertices[6 + elements_count * 4 + 5];

	// control points
	/*for (size_t i = 0; i < hull.size() - 1; i++) {
		vertices[elements_count * 2 + i * 6] = (float)cpa[i].x;
		vertices[elements_count * 2 + i * 6 + 1] = (float)cpa[i].y;
		vertices[elements_count * 2 + i * 6 + 2] = 0.0f;
		vertices[elements_count * 2 + i * 6 + 3] = 0.5f;
		vertices[elements_count * 2 + i * 6 + 4] = 1.0f;
		vertices[elements_count * 2 + i * 6 + 5] = 0.5f;
	}
	for (size_t i = 0; i < hull.size() - 1; i++) {
		vertices[elements_count * 3 + i * 6] = (float)cpb[i].x;
		vertices[elements_count * 3 + i * 6 + 1] = (float)cpb[i].y;
		vertices[elements_count * 3 + i * 6 + 2] = 0.0f;
		vertices[elements_count * 3 + i * 6 + 3] = 0.5f;
		vertices[elements_count * 3 + i * 6 + 4] = 0.5f;
		vertices[elements_count * 3 + i * 6 + 5] = 1.0f;
	}*/

	//draw
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_size * (4 + bezier_frags), vertices);
	shader->Use();

	glm::mat4 model;
	glm::mat4 view;
	model = glm::rotate(model, (GLfloat)/*glfwGetTime() * 1.0f*/0, glm::vec3(0.5f, 1.0f, 0.0f));
	view = glm::translate(view, glm::vec3(0.0f, -1.0f, -1.0f));

	GLint modelLoc = glGetUniformLocation(shader->Program, "model");
	GLint viewLoc = glGetUniformLocation(shader->Program, "view");
	GLint projLoc = glGetUniformLocation(shader->Program, "projection");

	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

	glBindVertexArray(VAO);
	glDrawArrays(GL_POINTS, 0, particleCount); //base vertices
	//glDrawArrays(GL_LINE_STRIP, particleCount, hull.size()); //hull points	
	//glDrawArrays(GL_POINTS, particleCount * 2, hull.size()); //cpab
	//glDrawArrays(GL_POINTS, particleCount * 3, hull.size());
	glDrawArrays(GL_TRIANGLE_FAN, (particleCount + 2) * 4, (hull.size() - 1) * bezier_frags + 2); //bezier hull
	glBindVertexArray(0);

	//std::cout << "Draw time: " << (glfwGetTime() - t2) * 1000 << " ms ";
}

void Blob::findHull() {
	std::vector<b2Vec2> points;
	points.resize(100);
	std::uniform_real_distribution<double> unif(-3.0, 3.0);
	std::default_random_engine re;
	for (int i = 0; i < 100; i++) {
		points[i].x = unif(re);
		points[i].y = unif(re);
	}
	HullFinder::FindConvexHull(points);
}

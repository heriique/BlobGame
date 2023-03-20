#ifndef HULLFINDER3_H
#define HULLFINDER3_H

// replacing Point with b2Vec2

#include <math.h>
#include <vector>
#include <algorithm>
#include <deque>
#include <Box2D\Box2D.h>


class Point
{
public:
	double x;
	double y;
	Point();
	Point(double _x, double _y);
	Point & operator=(const Point & other);
	Point operator+(const Point & other) const;
	Point operator-(const Point & other) const;
	Point operator*(double k) const;
	Point operator/(double k) const;
	bool operator==(const Point & other) const;
	bool operator!=(const Point & other) const;

	double DotProduct(const Point & other) const; //b2Dot(a,b)
	double DistanceSquared(const Point & to) const; //b2DistanceSquared(a,b)
	double Distance(const Point & to) const; //b2Distance(a,b)
	double DecisionDistance(const std::deque<Point*> & points) const;
	Point normalize(); // normalize vector from (0,0) to Point          //b2Vec2::Normalize()
};

class HullFinder
{
public:
	HullFinder();

	static double IsLeft(b2Vec2 p0, b2Vec2 p1, b2Vec2 p2);
	static bool IsPointInsidePolygon(b2Vec2 v, const std::vector<b2Vec2> &polygon);
	static bool CheckEdgeIntersection(const b2Vec2 &p0, const b2Vec2 &p1, const b2Vec2 &p2, const b2Vec2 &p3);
	static bool CheckEdgeIntersection(const std::vector<b2Vec2> &hull, b2Vec2 curEdgeStart, b2Vec2 curEdgeEnd, b2Vec2 checkEdgeStart, b2Vec2 checkEdgeEnd);
	static b2Vec2 NearestInnerPoint(b2Vec2 edgeStart, b2Vec2 edgeEnd, const std::vector<b2Vec2> &points, const std::vector<b2Vec2> &hull, bool* found);
	static std::vector<b2Vec2> FindConvexHull(const std::vector<b2Vec2> &points);
	static std::vector<b2Vec2> FindConcaveHull(const std::vector<b2Vec2> &points, double N);
	static double DecisionDistance(b2Vec2 & p, const std::deque<b2Vec2*> &points);
	static double DistanceToSegment(b2Vec2 p, b2Vec2 segmentStart, b2Vec2 segmentEnd);
};

#endif // HULLFINDER3_H

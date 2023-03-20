#ifndef HULLFINDER2_H
#define HULLFINDER2_H

#include <math.h>
#include <vector>
#include <algorithm>
#include <deque>


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

	double DotProduct(const Point & other) const;
	double DistanceSquared(const Point & to) const;
	double Distance(const Point & to) const;
	double Distance(const Point & segmentStart, const Point & segmentEnd) const;
	double DecisionDistance(const std::deque<Point*> & points) const;
	Point normalize(); // normalize vector from (0,0) to Point
};

class HullFinder
{
public:
	HullFinder();

	static double IsLeft(Point p0, Point p1, Point p2);
	static bool IsPointInsidePolygon(Point v, const std::vector<Point> & polygon);//
	static bool CheckEdgeIntersection(const Point & p1, const Point & p2, const Point & p3, const Point & p4);
	static bool CheckEdgeIntersection(const std::vector<Point> & hull, Point curEdgeStart, Point curEdgeEnd, Point checkEdgeStart, Point checkEdgeEnd);//
	static Point NearestInnerPoint(Point edgeStart, Point edgeEnd, const std::vector<Point> & points, const std::vector<Point> & hull, bool * found);
	static std::vector<Point> FindConvexHull(const std::vector<Point> &points);//
	static std::vector<Point> FindConcaveHull(const std::vector<Point> & points, double N);//
};

#endif // HULLFINDER2_H

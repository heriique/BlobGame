#include "hullfinder3.h"
#include <iostream>

Point::Point()
{
	x = 0;
	y = 0;
}

Point::Point(double _x, double _y)
{
	x = _x;
	y = _y;
}

Point & Point::operator=(const Point & other)
{
	x = other.x;
	y = other.y;
	return *this;
}

Point Point::operator+(const Point & other) const
{
	return Point(x + other.x, y + other.y);
}

Point Point::operator-(const Point & other) const
{
	return Point(x - other.x, y - other.y);
}

Point Point::operator*(double k) const
{
	return Point(x * k, y * k);
}

Point Point::operator/(double k) const
{
	return Point(x / k, y / k);
}

bool Point::operator==(const Point & other) const
{
	return x == other.x && y == other.y;
}

bool Point::operator!=(const Point & other) const {
	return x != other.x || y != other.y;
}

double Point::DotProduct(const Point & other) const
{
	return x * other.x + y * other.y;
}

double Point::DistanceSquared(const Point & to) const
{
	return (double)((to.x - x) * (to.x - x) + (to.y - y) * (to.y - y));
}

double Point::Distance(const Point & to) const
{
	return sqrt(DistanceSquared(to));
}



double Point::DecisionDistance(const std::deque<Point*> & points) const
{
	Point result = *(points[0]);
	double dst = Distance(*(points[0]));
	for (unsigned int i = 1; i < points.size(); i++) {
		Point cur = *(points[i]);
		double curDistance = Distance(cur);
		if (curDistance < dst) {
			result = cur;
			dst = curDistance;
		}
	}
	return dst;
}

Point Point::normalize()
{
	double d = sqrt(x * x + y * y);
	return Point(x / d, y / d);
}

HullFinder::HullFinder()
{
}


double HullFinder::IsLeft(b2Vec2 p0, b2Vec2 p1, b2Vec2 p2) 
{
	return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

bool HullFinder::IsPointInsidePolygon(b2Vec2 v, const std::vector<b2Vec2>& polygon)
{
	bool result = false;
	int j = polygon.size() - 1;
	for (unsigned int i = 0; i < polygon.size(); i++)
	{
		if ((polygon[i].y < v.y && polygon[j].y > v.y) || (polygon[j].y < v.y && polygon[i].y > v.y))
		{
			if (polygon[i].x + (v.y - polygon[i].y) / (polygon[j].y - polygon[i].y) * (polygon[j].x - polygon[i].x) < v.x)
			{
				result = !result;
			}
		}
		j = i;
	}
	return result;
}

bool HullFinder::CheckEdgeIntersection(const b2Vec2 & p0, const b2Vec2 & p1, const b2Vec2 & p2, const b2Vec2 & p3)
{
	double s1_x = p1.x - p0.x;
	double s1_y = p1.y - p0.y;
	double s2_x = p3.x - p2.x;
	double s2_y = p3.y - p2.y;
	double s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.y)) / (-s2_x * s1_y + s1_x * s2_y);
	double t = (s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.x)) / (-s2_x * s1_y + s1_x * s2_y);
	return (s > 0 && s < 1 && t > 0 && t < 1);
}

bool HullFinder::CheckEdgeIntersection(const std::vector<b2Vec2>& hull, b2Vec2 curEdgeStart, b2Vec2 curEdgeEnd, b2Vec2 checkEdgeStart, b2Vec2 checkEdgeEnd)
{
	for (int i = 0; i < hull.size() - 2; i++) {
		int e1 = i;
		int e2 = i + 1;
		b2Vec2 p1 = hull[e1];
		b2Vec2 p2 = hull[e2];

		if (curEdgeStart == p1 && curEdgeEnd == p2) {
			continue;
		}

		if (CheckEdgeIntersection(checkEdgeStart, checkEdgeEnd, p1, p2)) {
			return true;
		}
	}
	return false;
}

b2Vec2 HullFinder::NearestInnerPoint(b2Vec2 edgeStart, b2Vec2 edgeEnd, const std::vector<b2Vec2>& points, const std::vector<b2Vec2>& hull, bool * found)
{
	b2Vec2 result;
	double distance = 0;
	*found = false;

	for (b2Vec2 p : points) {
		// Skip points that are already in he hull
		/*if (hull.contains(p)) {
		continue;
		}*/
		if (std::find(hull.begin(), hull.end(), p) != hull.end()) {
			continue;
		}
		/*if (!IsPointInsidePolygon(p, hull)) {
		continue;
		}*/

		double d = b2Distance(edgeStart, edgeEnd);
		//double d = p.Distance(edgeStart, edgeEnd);
		bool skip = false;
		for (int i = 0; !skip && i < hull.size() - 1; i++) {
			double dTmp = HullFinder::DistanceToSegment(p, hull[i], hull[i + 1]);
			//double dTmp = p.Distance(hull[i], hull[i + 1]);
			skip |= dTmp < d;
		}
		if (skip) {
			continue;
		}

		if (!(*found) || distance > d) {
			result = p;
			distance = d;
			*found = true;
		}
	}
	return result;
}



std::vector<b2Vec2> HullFinder::FindConvexHull(const std::vector<b2Vec2> &points) {
	std::vector<b2Vec2> P = points;
	std::vector<b2Vec2> H;
	H.clear();

	// Sort P by x and y
	for (int i = 0; i < P.size(); i++) {
		for (int j = i + 1; j < P.size(); j++) {
			if (P[j].x < P[i].x || (P[j].x == P[i].x && P[j].y < P[i].y)) {
				b2Vec2 tmp = P[i];
				P[i] = P[j];
				P[j] = tmp;
			}
		}
	}

	// the output array H[] will be used as the stack
	int i;                 // array scan index

						   // Get the indices of points with min x-coord and min|max y-coord
	int minmin = 0, minmax;
	double xmin = P[0].x;
	for (i = 1; i < P.size(); i++)
		if (P[i].x != xmin) break;
	minmax = i - 1;
	if (minmax == P.size() - 1) {       // degenerate case: all x-coords == xmin
		H.push_back(P[minmin]);
		if (P[minmax].y != P[minmin].y) // a  nontrivial segment
			H.push_back(P[minmax]);
		H.push_back(P[minmin]);            // add polygon endpoint
		return H;
	}

	// Get the indices of points with max x-coord and min|max y-coord
	int maxmin, maxmax = P.size() - 1;
	double xmax = P.back().x;
	for (i = P.size() - 2; i >= 0; i--)
		if (P[i].x != xmax) break;
	maxmin = i + 1;

	// Compute the lower hull on the stack H
	H.push_back(P[minmin]);      // push  minmin point onto stack
	i = minmax;
	while (++i <= maxmin)
	{
		// the lower line joins P[minmin]  with P[maxmin]
		if (IsLeft(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
			continue;           // ignore P[i] above or on the lower line

		while (H.size() > 1)         // there are at least 2 points on the stack
		{
			// test if  P[i] is left of the line at the stack top
			if (IsLeft(H[H.size() - 2], H.back(), P[i]) > 0)
				break;         // P[i] is a new hull  vertex
			H.pop_back();         // pop top point off  stack
		}
		H.push_back(P[i]);        // push P[i] onto stack
	}

	//std::cout << "2. P.size= " << P.size() << ", H.size= " << H.size() << std::endl;

	// Next, compute the upper hull on the stack H above  the bottom hull
	if (maxmax != maxmin)      // if  distinct xmax points
		H.push_back(P[maxmax]);  // push maxmax point onto stack
	int bot = H.size();                  // the bottom point of the upper hull stack
	i = maxmin;
	while (--i >= minmax)
	{
		// the upper line joins P[maxmax]  with P[minmax]
		if (IsLeft(P[maxmax], P[minmax], P[i]) >= 0 && i > minmax)
			continue;           // ignore P[i] below or on the upper line

		while (H.size() > bot)     // at least 2 points on the upper stack
		{
			// test if  P[i] is left of the line at the stack top
			if (IsLeft(H[H.size() - 2], H.back(), P[i]) > 0)
				break;         // P[i] is a new hull  vertex
			H.pop_back();         // pop top point off stack
		}
		H.push_back(P[i]);        // push P[i] onto stack
	}
	if (minmax != minmin)
		H.push_back(P[minmin]);  // push  joining endpoint onto stack

	return H;
}



std::vector<b2Vec2> HullFinder::FindConcaveHull(const std::vector<b2Vec2> &points, double N) {
	//std::cout << "before concave list";
	std::vector<b2Vec2> concaveList = FindConvexHull(points);
	//std::cout << "after concave list";


	for (int i = 0; i < concaveList.size() - 1; i++) {
		// Find the nearest inner point pk ∈ G from the edge (ci1, ci2);
		b2Vec2 ci1 = concaveList[i];
		b2Vec2 ci2 = concaveList[i + 1];

		bool found;
		b2Vec2 pk = NearestInnerPoint(ci1, ci2, points, concaveList, &found);

		if (!found || (std::find(concaveList.begin(), concaveList.end(), pk) != concaveList.end())) {
			continue;
		}

		double eh = b2Distance(ci1, ci2);
		//double eh = ci1.Distance(ci2);  // the lenght of the edge
		std::deque<b2Vec2*> tmp;
		tmp.push_back(&ci1);
		tmp.push_back(&ci2);
		double dd = HullFinder::DecisionDistance(pk, tmp);
		//double dd = pk.DecisionDistance(tmp);

		if (eh / dd > N) {
			// Check that new candidate edge will not intersect existing edges.
			bool intersects = CheckEdgeIntersection(concaveList, ci1, ci2, ci1, pk);
			intersects |= CheckEdgeIntersection(concaveList, ci1, ci2, pk, ci2);
			if (!intersects) {
				concaveList.insert(concaveList.begin() + i + 1, pk);
				i--;
			}
		}
	}
	return concaveList;
}

double HullFinder::DecisionDistance(b2Vec2 & p, const std::deque<b2Vec2*>& points)
{
	b2Vec2 result = *(points[0]);
	float32 dst = b2Distance(p, *(points[0]));
	for (unsigned int i = 1; i < points.size(); i++) {
		b2Vec2 cur = *(points[i]);
		double curDistance = b2Distance(p, cur);
		if (curDistance < dst) {
			result = cur;
			dst = curDistance;
		}
	}
	return dst;
}


double HullFinder::DistanceToSegment(b2Vec2 p, b2Vec2 segmentStart, b2Vec2 segmentEnd)
{
	const double l2 = b2DistanceSquared(segmentStart, segmentEnd);
	if (l2 == 0.0) {
		return b2Distance(p, segmentStart);  // v == w case
	}

	// Consider the line extending the segment, parameterized as v + t (w - v)
	// We find projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	const double t = b2Dot(p - segmentStart, segmentEnd - segmentStart) / l2;
	//const double t = ((p - segmentStart).DotProduct(segmentEnd - segmentStart)) / l2;
	if (t < 0.0) {
		return b2Distance(p, segmentStart); // Beyond the 'v' end of the segment
	}
	else if (t > 1.0) {
		return b2Distance(p, segmentEnd);   // Beyond the 'w' end of the segment
	}

	// Projection falls on the segment
	b2Vec2 projection = segmentStart + (segmentEnd - segmentStart) * t;
	return b2Distance(p, projection);
}

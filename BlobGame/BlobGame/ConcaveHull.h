#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cassert>
#include <unordered_map>
#include <cstdint>


#pragma warning(push, 0)
#include <flann\flann.hpp>
#pragma warning(pop)

#define USE_OPENMP

#if defined USE_OPENMP
#if !defined _OPENMP
#pragma message("You've chosen to want OpenMP usage but have not made it a compilation option. Compile with /openmp")
#endif
#endif

using std::uint64_t;
struct Point2
{
	double x = 0.0;
	double y = 0.0;
	uint64_t id = 0;

	Point2() = default;

	Point2(double x, double y)
		: x(x)
		, y(y)
	{}
};
using PointVector = std::vector<Point2>;

class ConcaveHull {

public:
	

	auto static ConcaveHullAlg(PointVector &dataset, size_t k, bool iterate)->PointVector;

private:
	struct PointValue
	{
		Point2 Point;
		double distance = 0.0;
		double angle = 0.0;
	};

	static const size_t stride = 24; // size in bytes of x, y, id

	
	using PointValueVector = std::vector<PointValue>;
	using LineSegment = std::pair<Point2, Point2>;

	// Floating Point comparisons
	auto static Equal(double a, double b) -> bool;
	auto static Zero(double a) -> bool;
	auto static LessThan(double a, double b) -> bool;
	auto static LessThanOrEqual(double a, double b) -> bool;
	auto static GreaterThan(double a, double b) -> bool;

	// Algorithm-specific
	auto static NearestNeighboursFlann(flann::Index<flann::L2<double>> &index, const Point2 &p, size_t k)->PointValueVector;
	
	auto static ConcaveHullAlg(PointVector &dataset, size_t k, PointVector &hull) -> bool;
	auto static SortByAngle(PointValueVector &values, const Point2 &p, double prevAngle)->PointVector;
	auto static AddPoint(PointVector &Points, const Point2 &p) -> void;

	// General maths
	auto static PointsEqual(const Point2 &a, const Point2 &b) -> bool;
	auto static Angle(const Point2 &a, const Point2 &b) -> double;
	auto static NormaliseAngle(double radians) -> double;
	auto static PointInPolygon(const Point2 &p, const PointVector &list) -> bool;
	auto static Intersects(const LineSegment &a, const LineSegment &b) -> bool;

	// Point list utilities
	auto static FindMinYPoint(const PointVector &Points)->Point2;
	auto static RemoveDuplicates(PointVector &Points) -> void;
	auto static IdentifyPoints(PointVector &Points) -> void;
	auto static RemoveHull(PointVector &Points, const PointVector &hull)->PointVector::iterator;
	auto static MultiplePointInPolygon(PointVector::iterator begin, PointVector::iterator end, const PointVector &hull) -> bool;
	auto static RemovePoint(PointVector &list, const Point2 &p) -> void;

	// Unit tests
	auto static TestAngle() -> void;
	auto static TestIntersects() -> void;
	auto static TestSplit() -> void;

	// I/O
	auto static Split(const std::string &value, const char *delims)->std::vector<std::string>;

	template <class InIt, class Predicate>
	bool static omp_parallel_any_of(InIt first, InIt last, const Predicate &pr);
};
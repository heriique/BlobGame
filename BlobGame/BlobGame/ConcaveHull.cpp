#include "ConcaveHull.h"

// Compare a and b for equality
auto ConcaveHull::Equal(double a, double b) -> bool
{
	return fabs(a - b) <= DBL_EPSILON;
}

// Compare value to zero
auto ConcaveHull::Zero(double a) -> bool
{
	return fabs(a) <= DBL_EPSILON;
}

// Compare for a < b
auto ConcaveHull::LessThan(double a, double b) -> bool
{
	return a < (b - DBL_EPSILON);
}

// Compare for a <= b
auto ConcaveHull::LessThanOrEqual(double a, double b) -> bool
{
	return a <= (b + DBL_EPSILON);
}

// Compare for a > b
auto ConcaveHull::GreaterThan(double a, double b) -> bool
{
	return a > (b + DBL_EPSILON);
}

// Compare whether two Points have the same x and y
auto ConcaveHull::PointsEqual(const Point2 &a, const Point2 &b) -> bool
{
	return Equal(a.x, b.x) && Equal(a.y, b.y);
}

// Remove duplicates in a list of Points
auto ConcaveHull::RemoveDuplicates(PointVector &Points) -> void
{
	sort(begin(Points), end(Points), [](const Point2 & a, const Point2 & b)
	{
		if (Equal(a.x, b.x))
			return LessThan(a.y, b.y);
		else
			return LessThan(a.x, b.x);
	});

	auto newEnd = unique(begin(Points), end(Points), [](const Point2 & a, const Point2 & b)
	{
		return PointsEqual(a, b);
	});

	Points.erase(newEnd, end(Points));
}

// Uniquely id the Points for binary searching
auto ConcaveHull::IdentifyPoints(PointVector &Points) -> void
{
	uint64_t id = 0;

	for (auto itr = begin(Points); itr != end(Points); ++itr, ++id)
	{
		itr->id = id;
	}
}

// Find the Point having the smallest y-value
auto ConcaveHull::FindMinYPoint(const PointVector &Points) -> Point2
{
	assert(!Points.empty());

	auto itr = min_element(begin(Points), end(Points), [](const Point2 & a, const Point2 & b)
	{
		if (Equal(a.y, b.y))
			return GreaterThan(a.x, b.x);
		else
			return LessThan(a.y, b.y);
	});

	return *itr;
}

// Lookup by ID and remove a Point from a list of Points
auto ConcaveHull::RemovePoint(PointVector &list, const Point2 &p) -> void
{
	auto itr = std::lower_bound(begin(list), end(list), p, [](const Point2 & a, const Point2 & b)
	{
		return a.id < b.id;
	});

	assert(itr != end(list) && itr->id == p.id);

	if (itr != end(list))
		list.erase(itr);
}

// Add a Point to a list of Points
auto ConcaveHull::AddPoint(PointVector &Points, const Point2 &p) -> void
{
	Points.push_back(p);
}

// Return the k-nearest Points in a list of Points from the given Point p (uses Flann library).
auto ConcaveHull::NearestNeighboursFlann(flann::Index<flann::L2<double>> &index, const Point2 &p, size_t k) -> PointValueVector
{
	std::vector<int> vIndices(k);
	std::vector<double> vDists(k);
	double test[] = { p.x, p.y };

	flann::Matrix<double> query(test, 1, 2);
	flann::Matrix<int> mIndices(vIndices.data(), 1, static_cast<int>(vIndices.size()));
	flann::Matrix<double> mDists(vDists.data(), 1, static_cast<int>(vDists.size()));

	int count_ = index.knnSearch(query, mIndices, mDists, k, flann::SearchParams(128));
	size_t count = static_cast<size_t>(count_);

	PointValueVector result(count);

	for (size_t i = 0; i < count; ++i)
	{
		int id = vIndices[i];
		const double *Point = index.getPoint(id);
		result[i].Point.x = Point[0];
		result[i].Point.y = Point[1];
		result[i].Point.id = id;
		result[i].distance = vDists[i];
	}

	return result;
}

// Returns a list of Points sorted in descending order of clockwise angle
auto ConcaveHull::SortByAngle(PointValueVector &values, const Point2 &from, double prevAngle) -> PointVector
{
	for_each(begin(values), end(values), [from, prevAngle](PointValue & to)
	{
		to.angle = NormaliseAngle(Angle(from, to.Point) - prevAngle);
	});

	sort(begin(values), end(values), [](const PointValue & a, const PointValue & b)
	{
		return GreaterThan(a.angle, b.angle);
	});

	PointVector angled(values.size());

	transform(begin(values), end(values), begin(angled), [](const PointValue & pv)
	{
		return pv.Point;
	});

	return angled;
}

// Get the angle in radians measured clockwise from +'ve x-axis
auto ConcaveHull::Angle(const Point2 &a, const Point2 &b) -> double
{
	double angle = -atan2(b.y - a.y, b.x - a.x);

	return NormaliseAngle(angle);
}

// Return angle in range: 0 <= angle < 2PI
auto ConcaveHull::NormaliseAngle(double radians) -> double
{
	if (radians < 0.0)
		return radians + M_PI + M_PI;
	else
		return radians;
}

// Return the new logical end after removing Points from dataset having ids belonging to hull
auto ConcaveHull::RemoveHull(PointVector &Points, const PointVector &hull) -> PointVector::iterator
{
	std::vector<uint64_t> ids(hull.size());

	transform(begin(hull), end(hull), begin(ids), [](const Point2 & p)
	{
		return p.id;
	});

	sort(begin(ids), end(ids));

	return remove_if(begin(Points), end(Points), [&ids](const Point2 & p)
	{
		return binary_search(begin(ids), end(ids), p.id);
	});
}

// Uses OpenMP to determine whether a condition exists in the specified range of elements. https://msdn.microsoft.com/en-us/library/ff521445.aspx
template <class InIt, class Predicate>
bool ConcaveHull::omp_parallel_any_of(InIt first, InIt last, const Predicate &pr)
{
	typedef typename std::iterator_traits<InIt>::value_type item_type;

	// A flag that indicates that the condition exists.
	bool found = false;

#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(last - first); ++i)
	{
		if (!found)
		{
			item_type &cur = *(first + i);

			// If the element satisfies the condition, set the flag to cancel the operation.
			if (pr(cur))
			{
				found = true;
			}
		}
	}

	return found;
}

// Check whether all Points in a begin/end range are inside hull.
auto ConcaveHull::MultiplePointInPolygon(PointVector::iterator begin, PointVector::iterator end, const PointVector &hull) -> bool
{
	auto test = [&hull](const Point2 & p)
	{
		return !PointInPolygon(p, hull);
	};

	bool anyOutside = true;

#if defined USE_OPENMP

	anyOutside = omp_parallel_any_of(begin, end, test); // multi-threaded

#else

	anyOutside = std::any_of(begin, end, test); // single-threaded

#endif

	return !anyOutside;
}

// Point-in-polygon test
auto ConcaveHull::PointInPolygon(const Point2 &p, const PointVector &list) -> bool
{
	if (list.size() <= 2)
		return false;

	const double &x = p.x;
	const double &y = p.y;

	int inout = 0;
	auto v0 = list.begin();
	auto v1 = v0 + 1;

	while (v1 != list.end())
	{
		if ((LessThanOrEqual(v0->y, y) && LessThan(y, v1->y)) || (LessThanOrEqual(v1->y, y) && LessThan(y, v0->y)))
		{
			if (!Zero(v1->y - v0->y))
			{
				double tdbl1 = (y - v0->y) / (v1->y - v0->y);
				double tdbl2 = v1->x - v0->x;

				if (LessThan(x, v0->x + (tdbl2 * tdbl1)))
					inout++;
			}
		}

		v0 = v1;
		v1++;
	}

	if (inout == 0)
		return false;
	else if (inout % 2 == 0)
		return false;
	else
		return true;
}

// Test whether two line segments intersect each other
auto ConcaveHull::Intersects(const LineSegment &a, const LineSegment &b) -> bool
{
	// https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/

	const double &ax1 = a.first.x;
	const double &ay1 = a.first.y;
	const double &ax2 = a.second.x;
	const double &ay2 = a.second.y;
	const double &bx1 = b.first.x;
	const double &by1 = b.first.y;
	const double &bx2 = b.second.x;
	const double &by2 = b.second.y;

	double a1 = ay2 - ay1;
	double b1 = ax1 - ax2;
	double c1 = a1 * ax1 + b1 * ay1;
	double a2 = by2 - by1;
	double b2 = bx1 - bx2;
	double c2 = a2 * bx1 + b2 * by1;
	double det = a1 * b2 - a2 * b1;

	if (Zero(det))
	{
		return false;
	}
	else
	{
		double x = (b2 * c1 - b1 * c2) / det;
		double y = (a1 * c2 - a2 * c1) / det;

		bool on_both = true;
		on_both = on_both && LessThanOrEqual(std::min(ax1, ax2), x) && LessThanOrEqual(x, std::max(ax1, ax2));
		on_both = on_both && LessThanOrEqual(std::min(ay1, ay2), y) && LessThanOrEqual(y, std::max(ay1, ay2));
		on_both = on_both && LessThanOrEqual(std::min(bx1, bx2), x) && LessThanOrEqual(x, std::max(bx1, bx2));
		on_both = on_both && LessThanOrEqual(std::min(by1, by2), y) && LessThanOrEqual(y, std::max(by1, by2));
		return on_both;
	}
}

// Iteratively call the main algorithm with an increasing k until success
auto ConcaveHull::ConcaveHullAlg(PointVector &dataset, size_t k, bool iterate) -> PointVector
{
	while (k < dataset.size())
	{
		PointVector hull;
		if (ConcaveHullAlg(dataset, k, hull) || !iterate)
		{
			return hull;
		}
		k++;
	}

	return{};
}

// The main algorithm from the Moreira-Santos paper.
auto ConcaveHull::ConcaveHullAlg(PointVector &PointList, size_t k, PointVector &hull) -> bool
{
	hull.clear();

	if (PointList.size() < 3)
	{
		return true;
	}
	if (PointList.size() == 3)
	{
		hull = PointList;
		return true;
	}

	// construct a randomized kd-tree index using 4 kd-trees
	// 2 columns, but stride = 24 bytes in width (x, y, ignoring id)
	flann::Matrix<double> matrix(&(PointList.front().x), PointList.size(), 2, stride);
	flann::Index<flann::L2<double>> flannIndex(matrix, flann::KDTreeIndexParams(4));
	flannIndex.buildIndex();

	std::cout << "\rFinal 'k'        : " << k;

	// Initialise hull with the min-y Point
	Point2 firstPoint = FindMinYPoint(PointList);
	AddPoint(hull, firstPoint);

	// Until the hull is of size > 3 we want to ignore the first Point from nearest neighbour searches
	Point2 currentPoint = firstPoint;
	flannIndex.removePoint(firstPoint.id);

	double prevAngle = 0.0;
	int step = 1;

	// Iterate until we reach the start, or until there's no Points left to process
	while ((!PointsEqual(currentPoint, firstPoint) || step == 1) && hull.size() != PointList.size())
	{
		if (step == 4)
		{
			// Put back the first Point into the dataset and into the flann index
			firstPoint.id = PointList.size();
			flann::Matrix<double> firstPointMatrix(&firstPoint.x, 1, 2, stride);
			flannIndex.addPoints(firstPointMatrix);
		}

		PointValueVector kNearestNeighbours = NearestNeighboursFlann(flannIndex, currentPoint, k);
		PointVector cPoints = SortByAngle(kNearestNeighbours, currentPoint, prevAngle);

		bool its = true;
		size_t i = 0;

		while (its && i < cPoints.size())
		{
			size_t lastPoint = 0;
			if (PointsEqual(cPoints[i], firstPoint))
				lastPoint = 1;

			size_t j = 2;
			its = false;

			while (!its && j < hull.size() - lastPoint)
			{
				auto line1 = std::make_pair(hull[step - 1], cPoints[i]);
				auto line2 = std::make_pair(hull[step - j - 1], hull[step - j]);
				its = Intersects(line1, line2);
				j++;
			}

			if (its)
				i++;
		}

		if (its)
			return false;

		currentPoint = cPoints[i];

		AddPoint(hull, currentPoint);

		prevAngle = Angle(hull[step], hull[step - 1]);

		flannIndex.removePoint(currentPoint.id);

		step++;
	}

	// The original Points less the Points belonging to the hull need to be fully enclosed by the hull in order to return true.
	PointVector dataset = PointList;

	auto newEnd = RemoveHull(dataset, hull);
	bool allEnclosed = MultiplePointInPolygon(begin(dataset), newEnd, hull);

	return allEnclosed;
}

// Unit test of Angle() function
auto ConcaveHull::TestAngle() -> void
{
	auto ToDegrees = [](double radians)
	{
		return radians * 180.0 / M_PI;
	};

	auto Test = [&](const Point2 &p, double expected)
	{
		double actual = ToDegrees(Angle({ 0.0, 0.0 }, p));
		assert(Equal(actual, expected));
	};

	double value = ToDegrees(atan(3.0 / 4.0));

	Test({ 5.0,  0.0 }, 0.0);
	Test({ 4.0,  3.0 }, 360.0 - value);
	Test({ 3.0,  4.0 }, 270.0 + value);
	Test({ 0.0,  5.0 }, 270.0);
	Test({ -3.0,  4.0 }, 270.0 - value);
	Test({ -4.0,  3.0 }, 180.0 + value);
	Test({ -5.0,  0.0 }, 180.0);
	Test({ -4.0, -3.0 }, 180.0 - value);
	Test({ -3.0, -4.0 }, 90.0 + value);
	Test({ 0.0, -5.0 }, 90.0);
	Test({ 3.0, -4.0 }, 90.0 - value);
	Test({ 4.0, -3.0 }, 0.0 + value);
}

// Unit test the Intersects() function
auto ConcaveHull::TestIntersects() -> void
{
	using std::make_pair;

	std::unordered_map<char, Point2> values;
	values['A'] = { 0.0,  0.0 };
	values['B'] = { -1.5,  3.0 };
	values['C'] = { 2.0,  2.0 };
	values['D'] = { -2.0,  1.0 };
	values['E'] = { -2.5,  5.0 };
	values['F'] = { -1.5,  7.0 };
	values['G'] = { 1.0,  9.0 };
	values['H'] = { -4.0,  7.0 };
	values['I'] = { 3.0, 10.0 };
	values['J'] = { 2.0, 11.0 };
	values['K'] = { -1.0, 11.0 };
	values['L'] = { -3.0, 11.0 };
	values['M'] = { -5.0,  9.5 };
	values['N'] = { -6.0,  7.5 };
	values['O'] = { -6.0,  4.0 };
	values['P'] = { -5.0,  2.0 };

	auto Test = [&values](const char a1, const char a2, const char b1, const char b2, bool expected)
	{
		assert(Intersects(make_pair(values[a1], values[a2]), make_pair(values[b1], values[b2])) == expected);
		assert(Intersects(make_pair(values[a2], values[a1]), make_pair(values[b1], values[b2])) == expected);
		assert(Intersects(make_pair(values[a1], values[a2]), make_pair(values[b2], values[b1])) == expected);
		assert(Intersects(make_pair(values[a2], values[a1]), make_pair(values[b2], values[b1])) == expected);
	};

	Test('B', 'D', 'A', 'C', false);
	Test('A', 'B', 'C', 'D', true);
	Test('L', 'K', 'H', 'F', false);
	Test('E', 'C', 'F', 'B', true);
	Test('P', 'C', 'E', 'B', false);
	Test('P', 'C', 'A', 'B', true);
	Test('O', 'E', 'C', 'F', false);
	Test('L', 'C', 'M', 'N', false);
	Test('L', 'C', 'N', 'B', false);
	Test('L', 'C', 'M', 'K', true);
	Test('L', 'C', 'G', 'I', false);
	Test('L', 'C', 'I', 'E', true);
	Test('M', 'O', 'N', 'F', true);
}

auto ConcaveHull::TestSplit() -> void
{
	std::vector<double> expected = { -123.456, -987.654 };

	auto Test = [&expected](const std::string &input)
	{
		auto actual = Split(input, " ,\t");
		assert(actual.size() >= 2);
		assert(Equal(atof(actual[0].c_str()), expected[0]));
		assert(Equal(atof(actual[1].c_str()), expected[1]));
	};

	Test("-123.456 -987.654");
	Test("-123.4560 -987.6540");
	Test("-123.45600 -987.65400");
	Test("-123.456 -987.654 ");
	Test("-123.456 -987.654 100.5");
	Test("-123.456 -987.654 hello");
	Test("-123.456 -987.654 hello world");

	Test("-123.456,-987.654");
	Test("-123.4560,-987.6540");
	Test("-123.45600,-987.65400");
	Test("-123.456,-987.654,");
	Test("-123.456,-987.654,100.5");
	Test("-123.456,-987.654,hello");
	Test("-123.456,-987.654,hello,world");

	Test("-123.456\t-987.654");
	Test("-123.4560\t-987.6540");
	Test("-123.45600\t-987.65400");
	Test("-123.456\t-987.654\t");
	Test("-123.456\t-987.654\t100.5");
	Test("-123.456\t-987.654\thello");
	Test("-123.456\t-987.654\thello\tworld");

	Test(" -123.456   -987.654   ");
	Test(" -123.4560  -987.6540  ");
	Test(" -123.45600 -987.65400 ");
	Test(" -123.456   -987.654  ");
	Test(" -123.456   -987.654  100.5");
	Test(" -123.456   -987.654  hello");
	Test(" -123.456   -987.654  hello   world");
}

// String tokenise using any one of delimiters, adjacent spaces are treated as one
auto ConcaveHull::Split(const std::string &value, const char *delims) -> std::vector<std::string>
{
	std::vector<std::string> ret;

	size_t start = value.find_first_not_of(' ', 0);
	while (start != std::string::npos)
	{
		size_t pos = value.find_first_of(delims, start);
		if (pos == std::string::npos)
		{
			ret.push_back(value.substr(start));
			break;
		}
		else
		{
			ret.push_back(value.substr(start, pos - start));

			if (value[pos] == ' ' && strchr(delims, ' '))
				start = value.find_first_not_of(' ', pos);
			else
				start = pos + 1;
		}
	}

	return ret;
}
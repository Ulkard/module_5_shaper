#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <ranges>
#include <stack>
#include <vector>

namespace geometry::convex_hull {

GeometryResult<std::vector<Point2D>> GrahamScan(const Shapes &points);

}  // namespace geometry::convex_hull
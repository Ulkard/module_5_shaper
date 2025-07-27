#include "convex_hull.hpp"
#include "geometry.hpp"
#include <algorithm>
#include <expected>
#include <functional>

#include "visualization.hpp"

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.Cross(new_p2);
}

struct ShapeToPointsVisitor {
    template <typename T>
    std::vector<Point2D> operator()(const T &shape) {
        std::vector<Point2D> result;
        for (const auto &point : shape.Vertices()) {
            result.push_back(point);
        }
        return result;
    }
};

enum class Orientation { COLLINEAR = 0, CW, CCW };

Orientation getOrientation(const Point2D &p0, const Point2D &p1, const Point2D &p2) {
    double val = (p1.y - p0.y) * (p2.x - p1.x) - (p1.x - p0.x) * (p2.y - p1.y);

    if (fabs(val) < EPSILON)
        return Orientation::COLLINEAR;
    return (val > 0) ? Orientation::CW : Orientation::CCW;
}

GeometryResult<std::vector<Point2D>> GrahamScan(const Shapes &shapes) {
    namespace rv = std::ranges::views;
    ShapeToPointsVisitor visitor;
    StackForGrahamScan hull_stack;
    std::vector<Point2D> points = shapes |
                                  rv::transform([&visitor](const Shape &shape) { return std::visit(visitor, shape); }) |
                                  rv::join | std::ranges::to<std::vector>();
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InsufficientPoints};
    }
    auto p0_it = std::ranges::min_element(points, &Point2D::operator<);
    std::swap(points[0], *p0_it);
    Point2D p0 = points[0];

    std::ranges::sort(points, [p0](const Point2D &p1, const Point2D &p2) {
        Orientation o = getOrientation(p0, p1, p2);

        if (o == Orientation::COLLINEAR) {
            return p0.DistanceTo(p2) >= p0.DistanceTo(p1);
        }
        return o == Orientation::CCW;
    });

    for (const Point2D &point : points) {
        while ((hull_stack.Size() > 1) &&
               getOrientation(hull_stack.NextToTop(), hull_stack.Top(), point) == Orientation::CW) {
            hull_stack.Pop();
        }
        hull_stack.Push(point);
    }

    return hull_stack.Extract();
}

}  // namespace geometry::convex_hull
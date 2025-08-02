#include "convex_hull.hpp"
#include "geometry.hpp"
#include <algorithm>
#include <expected>
#include <functional>

#include "visualization.hpp"

namespace geometry::convex_hull {

class StackForGrahamScan {
public:
    void Push(const Point2D &p) { s.push_back(p); }
    void Pop() { s.pop_back(); }

    [[nodiscard]] size_t Size() { return s.size(); }
    [[nodiscard]] Point2D Top() { return s.back(); }
    [[nodiscard]] Point2D NextToTop() { return *std::prev(s.end(), 2); }

    [[nodiscard]] std::vector<Point2D> &&Extract() { return std::move(s); }

private:
    std::vector<Point2D> s;
};

struct ShapeToPointsVisitor {
    template <typename T>
    [[nodiscard]] std::vector<Point2D> operator()(const T &shape) {
        std::vector<Point2D> result;
        for (const auto &point : shape.Vertices()) {
            result.push_back(point);
        }
        return result;
    }
};

enum class Orientation { COLLINEAR = 0, CW, CCW };

[[nodiscard]] Orientation getOrientation(const Point2D &p0, const Point2D &p1, const Point2D &p2) {
    double val = CrossProduct(p2, p1, p0);
    if (fabs(val) < EPSILON)
        return Orientation::COLLINEAR;
    return (val > 0) ? Orientation::CW : Orientation::CCW;
}

[[nodiscard]] GeometryResult<std::vector<Point2D>> GrahamScan(const Shapes &shapes) {
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
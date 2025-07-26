#pragma once
#include "geometry.hpp"
#include <cmath>
#include <optional>
#include <print>
#include <stdexcept>
#include <type_traits>

namespace geometry::intersections {

class IntersectionVisitor {
public:
    static constexpr double EPSILON = 1e-10;

    std::optional<Point2D> operator()(const Line &lhs, const Line &rhs) {
        if (!lhs.BoundBox().Overlaps(rhs.BoundBox())) {
            return std::nullopt;
        }

        const auto &[p1, p2] = lhs;
        const auto &[p3, p4] = rhs;
        Point2D dir_1 = lhs.Direction();
        Point2D dir_2 = rhs.Direction();

        double cross_dir = dir_1.Cross(dir_2);
        Point2D qp = p3 - p1;
        if (std::abs(cross_dir) < EPSILON) {
            return std::nullopt;
        }

        double pos_along_1 = qp.Cross(dir_2) / cross_dir;
        double pos_along_2 = qp.Cross(dir_1) / cross_dir;

        // Check if intersection point lies within both lines
        if (pos_along_1 >= 0 && pos_along_1 <= 1 && pos_along_2 >= 0 && pos_along_2 <= 1) {
            // Calculate intersection point
            return Point2D{p1.x + pos_along_1 * dir_1.x, p1.y + pos_along_1 * dir_1.y};
        }

        return std::nullopt;
    }

    std::optional<Point2D> operator()(const Line &line, const Circle &circle) {
        if (!line.BoundBox().Overlaps(circle.BoundBox())) {
            return std::nullopt;
        }

        auto [dx, dy] = line.Direction();
        // Vector from line start to circle center
        auto [fx, fy] = Line{line.start, circle.Center()}.Direction();

        // Coefficients of the quadratic equation
        double a = dx * dx + dy * dy;
        double b = 2 * (fx * dx + fy * dy);
        double c = fx * fx + fy * fy - circle.radius * circle.radius;

        double discriminant = b * b - 4 * a * c;
        discriminant = std::sqrt(discriminant);
        if (discriminant < 0) {
            return std::nullopt;
        }

        double x1 = (-b - discriminant) / (2 * a);
        double x2 = (-b + discriminant) / (2 * a);

        // тут и в кругах может быть два пересечения, но прекод подразумевает возврат только одного
        if (x1 >= 0 && x1 <= 1) {
            return Point2D{line.start.x + x1 * dx, line.start.y + x1 * dy};
        }
        if (x2 >= 0 && x2 <= 1) {
            return Point2D{line.start.x + x2 * dx, line.start.y + x2 * dy};
        }

        return std::nullopt;
    }
    std::optional<Point2D> operator()(const Circle &circle, const Line &line) { return operator()(line, circle); }

    std::optional<Point2D> operator()(const Circle &lhs, const Circle &rhs) {
        // Calculate distance between centers
        auto [dx, dy] = rhs.Center() - lhs.Center();
        double distance = std::hypot(dx, dy);

        if (distance > lhs.radius + rhs.radius) {
            return std::nullopt;
        }

        // Check for coincident circles
        if (distance == 0 && lhs.radius == rhs.radius) {
            return std::nullopt;
        }

        double a = (lhs.radius * lhs.radius - rhs.radius * rhs.radius + distance * distance) / (2 * distance);
        double h = std::sqrt(lhs.radius * lhs.radius - a * a);

        Point2D p2{lhs.Center().x + (dx * a) / distance, lhs.Center().y + (dy * a) / distance};

        if (h == 0) {  // One intersection
            return p2;
        } else {  // Two intersections
            return Point2D{p2.x + (-dy * h) / distance, p2.y + (dx * h) / distance};
            // second point {p2.x - (-dy * h) / distance, p2.y - (dx * h) / distance};
        }

        return std::nullopt;
    }

    std::optional<Point2D> operator()(auto &, auto &) {
        throw std::logic_error("IntersectionVisitor: unsupported types");
    }
};

inline std::optional<Point2D> GetIntersectPoint(const Shape &shape1, const Shape &shape2) {
    return std::visit(IntersectionVisitor{}, shape1, shape2);
}

}  // namespace geometry::intersections
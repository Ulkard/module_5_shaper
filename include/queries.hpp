#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <functional>
#include <optional>
#include <variant>

namespace geometry::queries {

template <class... Ts>
struct Multilambda : Ts... {
    using Ts::operator()...;
};

/*
 * Класс для поиска расстояния от точки до фигуры
 *
 * Требуется организовать возможность нахождения расстояния для всех возможных фигур типа-суммы Shape
 */
struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    template <typename T>
    double operator()(T &&shape) {
        return point.DistanceTo(shape.Center());
    }
};

/*
 * Класс для поиска расстояния между двумя фигурами
 *
 * Требуется организовать возможность нахождения расстояния только для следующих комбинаций фигур:
 *    - Any    & Point
 *    - Line   & Line
 *    - Circle & Circle
 *
 * Для всех остальных требуется вернуть пустое значение
 */
struct ShapeToShapeDistanceVisitor {
    std::optional<double> operator()(const Line &lhs, const Line &rhs) {
        const auto [a, b] = lhs;
        const auto [c, d] = rhs;
        auto ccw = [](const Point2D &A, const Point2D &B, const Point2D &C) {
            return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
        };

        bool intersect = ccw(a, c, d) != ccw(b, c, d) && ccw(a, b, c) != ccw(a, b, d);
        if (intersect)
            return 0.0;

        return std::ranges::min({pointToLineDistance(a, rhs), pointToLineDistance(b, rhs), pointToLineDistance(c, lhs),
                                 pointToLineDistance(d, lhs)});
    }

    std::optional<double> operator()(const Circle &lhs, const Circle &rhs) {
        return lhs.Center().DistanceTo(rhs.Center()) - lhs.radius - rhs.radius;
    }

    template <typename T1, typename T2>
    std::optional<double> operator()(T1 &&, T2 &&) {
        return std::nullopt;
    }

private:
    double pointToLineDistance(const Point2D &p, const Line &line) {
        const auto [a, b] = line;
        if (a == b) {
            return p.DistanceTo(a);
        }
        Point2D line_dir = line.Direction();
        Point2D ap = p - a;

        double t = ap.Dot(line_dir) / line_dir.Dot(line_dir);
        t = std::clamp(t, 0.0, 1.0);

        Point2D projection = a + line_dir * t;
        return p.DistanceTo(projection);
    }
};

/*
 * Функции-помощники
 */
inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    PointToShapeDistanceVisitor visitor(point);
    return std::visit(visitor, shape);
}

inline BoundingBox GetBoundBox(const Shape &shape) {
    return std::visit(Multilambda{[](const Line &shape) { return shape.BoundBox(); },
                                  [](const Triangle &shape) { return shape.BoundBox(); },
                                  [](const Rectangle &shape) { return shape.BoundBox(); },
                                  [](const RegularPolygon &shape) { return shape.BoundBox(); },
                                  [](const Circle &shape) { return shape.BoundBox(); },
                                  [](const Polygon &shape) { return shape.BoundBox(); }},
                      shape);
}

inline double GetHeight(const Shape &shape) {
    return std::visit(Multilambda{[](const Line &shape) { return shape.Height(); },
                                  [](const Triangle &shape) { return shape.Height(); },
                                  [](const Rectangle &shape) { return shape.Height(); },
                                  [](const RegularPolygon &shape) { return shape.Height(); },
                                  [](const Circle &shape) { return shape.Height(); },
                                  [](const Polygon &shape) { return shape.Height(); }},
                      shape);
}

inline bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2) {
    return GetBoundBox(shape1).Overlaps(GetBoundBox(shape2));
}

std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    ShapeToShapeDistanceVisitor visitor;
    return std::visit(visitor, shape1, shape2);
}

}  // namespace geometry::queries
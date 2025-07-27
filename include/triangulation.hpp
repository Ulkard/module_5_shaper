#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <print>
#include <set>
#include <vector>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    bool ContainsPoint(const Point2D &p) const {
        Point2D center = Circumcenter();
        double radius = Circumradius();
        return center.DistanceTo(p) <= radius + EPSILON;
    }

    Point2D Circumcenter() const {
        double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (std::abs(d) < EPSILON) {
            return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
        }

        double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                     (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                    d;

        double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                     (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                    d;

        return {ux, uy};
    }

    double Circumradius() const {
        Point2D center = Circumcenter();
        return center.DistanceTo(a);
    }

    bool SharesEdge(const DelaunayTriangle &other) const {
        std::vector<Point2D> this_points = {a, b, c};
        std::vector<Point2D> other_points = {other.a, other.b, other.c};

        int shared_count = 0;
        for (const Point2D &p1 : this_points) {
            for (const Point2D &p2 : other_points) {
                if (std::abs(p1.x - p2.x) < EPSILON && std::abs(p1.y - p2.y) < EPSILON) {
                    shared_count++;
                    break;
                }
            }
        }

        return shared_count == 2;
    }

    bool operator==(const DelaunayTriangle &other) const {
        return std::ranges::contains(vertices(), other.a) && std::ranges::contains(vertices(), other.b) &&
               std::ranges::contains(vertices(), other.c);
    }

    std::vector<Point2D> vertices() const { return {a, b, c}; }
};

struct Edge {
    Point2D p1, p2;

    Edge(Point2D p1, Point2D p2) : p1(p1), p2(p2) {
        if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y)) {
            std::swap(this->p1, this->p2);
        }
    }

    bool operator<(const Edge &other) const {
        if (std::abs(p1.x - other.p1.x) > EPSILON)
            return p1.x < other.p1.x;
        if (std::abs(p1.y - other.p1.y) > EPSILON)
            return p1.y < other.p1.y;
        if (std::abs(p2.x - other.p2.x) > EPSILON)
            return p2.x < other.p2.x;
        return p2.y < other.p2.y;
    }

    bool operator==(const Edge &other) const {
        return std::abs(p1.x - other.p1.x) < EPSILON && std::abs(p1.y - other.p1.y) < EPSILON &&
               std::abs(p2.x - other.p2.x) < EPSILON && std::abs(p2.y - other.p2.y) < EPSILON;
    }
};

inline DelaunayTriangle makeSuperTriangle(std::span<const Point2D> points) {
    double min_x = points[0].x;
    double max_x = points[0].x;
    double min_y = points[0].y;
    double max_y = points[0].y;
    for (const auto &point : points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double delta_max = std::max(dx, dy) * 10;

    Point2D p1(min_x - delta_max, min_y - delta_max);
    Point2D p2(max_x + delta_max, min_y - delta_max);
    Point2D p3(min_x + dx / 2, max_y + delta_max);

    return DelaunayTriangle(p1, p2, p3);
}

inline std::vector<Edge> GetEdges(const DelaunayTriangle &triangle) {
    return {{triangle.a, triangle.b}, {triangle.b, triangle.c}, {triangle.c, triangle.a}};
}

inline bool SharesEdge(const Edge &edge, const DelaunayTriangle &triangle) {
    std::vector<Edge> triangle_edges = GetEdges(triangle);
    for (const auto &e : triangle_edges) {
        if (e == edge) {
            return true;
        }
    }
    return false;
}

inline GeometryResult<std::vector<DelaunayTriangle>> DelaunayTriangulation(std::span<const Point2D> points) {
    if (points.size() < 3) {
        return std::unexpected{GeometryError::InsufficientPoints};
    }
    const auto super = makeSuperTriangle(points);
    std::vector<DelaunayTriangle> triangulation{super};

    for (const auto &point : points) {
        // находим все "плохие" треугольники (из текущей триангуляции), в чьи описанные окружности входит эта точка
        std::vector<DelaunayTriangle> bad_triangles;
        for (const auto &triangle : triangulation) {
            if (triangle.ContainsPoint(point)) {
                bad_triangles.push_back(triangle);
            }
        }

        // ищем границы "полигональной дырки"
        std::vector<Edge> polygon;
        for (const auto &triangle : bad_triangles) {

            for (const auto &edge : GetEdges(triangle)) {
                bool shared = false;
                for (const auto &other_triangle : bad_triangles) {
                    if (&triangle == &other_triangle)
                        continue;
                    if (SharesEdge(edge, other_triangle)) {
                        shared = true;
                        break;
                    }
                }
                if (!shared) {
                    polygon.push_back(edge);
                }
            }
        }

        // удалить из текущей триангуляции все плохие треугольники: cur_triangulation.erase(bad_triangles.contains(*it))
        std::erase_if(triangulation,
                      [&bad_triangles](const DelaunayTriangle &t) { return std::ranges::contains(bad_triangles, t); });

        // Для каждой границы "дырки" (polygonal hole) создаются новые треугольники с новой точкой: { ТочкаРебра1,
        // ТочкаРебра2, НоваяТочка }.
        for (const auto &edge : polygon) {
            triangulation.emplace_back(edge.p1, edge.p2, point);
        }
    }

    // Удаляем все треугольники, включающие вершины супер-треугольника
    std::erase_if(triangulation, [&super](const DelaunayTriangle &t) {
        for (const auto &point : t.vertices()) {
            if (std::ranges::contains(super.vertices(), point)) {
                return true;
            }
        }
        return false;
    });

    return triangulation;
}

}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};

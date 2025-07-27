#pragma once
#include <algorithm>
#include <array>
#include <bits/ranges_algo.h>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <expected>
#include <format>
#include <functional>
#include <iterator>
#include <numbers>
#include <optional>
#include <print>
#include <ranges>
#include <string_view>
#include <variant>
#include <vector>

namespace geometry {

static constexpr double EPSILON = 1e-10;

struct Point2D {
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    // Comparison
    bool operator<(const Point2D &other) const { return x < other.x && y < other.y; }
    bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }

    // Binary math operators
    Point2D operator+(const Point2D &other) const { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D &other) const { return {x - other.x, y - other.y}; }
    Point2D operator*(double value) const { return {x * value, y * value}; }
    Point2D operator/(double value) const { return {x / value, y / value}; }

    // Binary geometry operations
    double Dot(const Point2D &other) const { return x * other.x + y * other.y; }
    double Cross(const Point2D &other) const { return x * other.y - y * other.x; }
    double Length() const { return std::sqrt(x * x + y * y); }
    double DistanceTo(const Point2D &other) const { return (*this - other).Length(); }

    Point2D Normalize() const {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }
};

template <size_t N>
struct Lines2D {
    std::array<double, N> x;
    std::array<double, N> y;
};

struct Lines2DDyn {
    std::vector<double> x;
    std::vector<double> y;

    void Reserve(size_t n) {
        x.reserve(n);
        y.reserve(n);
    }
    void PushBack(Point2D p) {
        x.push_back(p.x);
        y.push_back(p.y);
    }
    void PushBack(double px, double py) {
        x.push_back(px);
        y.push_back(py);
    }
    Point2D Front() const { return {x.front(), y.front()}; }
};

struct BoundingBox {
    double min_x, min_y, max_x, max_y;

    bool Overlaps(const BoundingBox &other) const {
        return !((max_x < other.min_x) || (min_x > other.max_x) || (max_y < other.min_y) || (min_y > other.max_y));
    }
    double Width() const { return max_x - min_x; }
    double Height() const { return max_y - min_y; }
    Point2D Center() const { return {(min_x + max_x) / 2, (min_y + max_y) / 2}; }
};

struct Line {
    Point2D start, end;

    double Length() const { return start.DistanceTo(end); }
    Point2D Direction() const { return end - start; }
    BoundingBox BoundBox() const {
        return {std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x), std::max(start.y, end.y)};
    }
    double Height() const { return std::abs(end.y - start.y); }
    Point2D Center() const { return (start + end) / 2; }

    Lines2D<2> Lines() const { return {{start.x, end.x}, {start.y, end.y}}; }
    std::array<Point2D, 2> Vertices() const { return {start, end}; }
};

struct Triangle {
    Point2D a, b, c;

    double Area() const {
        double ab = a.DistanceTo(b);
        double bc = b.DistanceTo(c);
        double ca = c.DistanceTo(a);
        double sp = (ab + bc + ca) / 2;
        return std::sqrt((sp - ab) * (sp - bc) * (sp - ca));
    }
    double Height() const { return std::ranges::max(/* {a,b,c} */ Vertices(), std::greater<>(), &Point2D::y).y; }
    Point2D Center() const { return (a + b + c) / 3; }
    BoundingBox BoundBox() const {
        return {std::ranges::min(/* {a,b,c} */ Vertices(), std::less<>(), &Point2D::x).x,
                std::ranges::min(/* {a,b,c} */ Vertices(), std::less<>(), &Point2D::y).y,
                std::ranges::max(/* {a,b,c} */ Vertices(), std::greater<>(), &Point2D::x).x,
                std::ranges::max(/* {a,b,c} */ Vertices(), std::greater<>(), &Point2D::y).y};
    }
    std::array<Point2D, 3> Vertices() const { return {a, b, c}; }

    Lines2D<4> Lines() const { return {{a.x, b.x, c.x, a.x}, {a.y, b.y, c.y, a.y}}; }
};

struct Rectangle {
    Point2D bottom_left;
    double width, height;

    double Area() const { return width * height; }
    double Height() const { return height; }
    Point2D Center() const { return {bottom_left.x + width / 2, bottom_left.y + height / 2}; }
    BoundingBox BoundBox() const {
        return {bottom_left.x, bottom_left.y, bottom_left.x + width, bottom_left.y + height};
    }
    std::array<Point2D, 4> Vertices() const {
        return {bottom_left, bottom_left + Point2D(width, 0), bottom_left + Point2D(width, height),
                bottom_left + Point2D(0, height)};
    }

    Lines2D<5> Lines() const {
        return {{bottom_left.x, bottom_left.x, bottom_left.x + width, bottom_left.x + width, bottom_left.x},
                {bottom_left.y, bottom_left.y + height, bottom_left.y + height, bottom_left.y, bottom_left.y}};
    }
};

struct RegularPolygon {
    Point2D center_p;
    double radius;
    int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    BoundingBox BoundBox() const {
        return {center_p.x - radius, center_p.y - radius, center_p.x + radius, center_p.y + radius};
    }
    double Height() const { return center_p.y + radius; }
    Point2D Center() const { return center_p; }

    std::vector<Point2D> Vertices() const {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }
    Lines2DDyn Lines(size_t N = 100) const {
        Lines2DDyn result;
        size_t size = std::min<size_t>(N, sides + 1);
        result.Reserve(size);

        for (size_t i = 0; i < size; ++i) {
            const double angle = 2 * std::numbers::pi * i / size;
            result.PushBack(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        result.PushBack(result.Front());
        return result;
    }
};

struct Circle {
    Point2D center_p;
    double radius;

    constexpr Circle(Point2D center, double radius) : center_p(center), radius(radius) {}

    BoundingBox BoundBox() const {
        return {center_p.x - radius, center_p.y - radius, center_p.x + radius, center_p.y + radius};
    }
    double Height() const { return center_p.y + radius; }
    Point2D Center() const { return center_p; }

    std::vector<Point2D> Vertices(size_t N = 30) const {
        std::vector<Point2D> points;
        points.reserve(N);

        for (size_t i = 0; i < N; ++i) {
            const double angle = 2 * std::numbers::pi * i / N;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }

    Lines2DDyn Lines(size_t N = 100) const {
        Lines2DDyn result;
        result.Reserve(N);

        for (size_t i = 0; i < N; ++i) {
            const double angle = 2 * std::numbers::pi * i / N;
            result.PushBack(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        result.PushBack(result.Front());
        return result;
    }
};

class Polygon {
public:
    constexpr Polygon(const std::vector<Point2D> &points) : points_(points) {
        bounding_box_ = {std::ranges::min(/* {a,b,c} */ points_, std::less<>(), &Point2D::x).x,
                         std::ranges::min(/* {a,b,c} */ points_, std::less<>(), &Point2D::y).y,
                         std::ranges::max(/* {a,b,c} */ points_, std::greater<>(), &Point2D::x).x,
                         std::ranges::max(/* {a,b,c} */ points_, std::greater<>(), &Point2D::y).y};
    }

    BoundingBox BoundBox() const { return bounding_box_; }
    double Height() const { return bounding_box_.max_y - bounding_box_.min_y; }
    Point2D Center() const { return std::ranges::fold_left(points_, Point2D(0, 0), std::plus<>()) / points_.size(); }

    const std::vector<Point2D> &Vertices() const { return points_; }
    Lines2DDyn Lines(size_t N = 100) const {
        Lines2DDyn result;
        size_t size = std::min(N, points_.size());
        result.Reserve(points_.size());

        for (size_t i = 0; i < size; ++i) {
            result.PushBack(points_[i]);
        }
        result.PushBack(result.Front());
        return result;
    }

private:
    std::vector<Point2D> points_;
    BoundingBox bounding_box_;
};

using Shape = std::variant<Line, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenrateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Point2D &p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};
template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) {
        auto it = ctx.begin();
        if (std::string_view(ctx).starts_with("new_line")) {
            use_new_line = true;
            return it + "new_line"sv.size();
        }

        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {
        for (const geometry::Point2D &p : v) {
            std::format_to(ctx.out(), "{}{}", use_new_line ? "\n\t" : " ", p);
        }

        return ctx.out();
    }
};

template <>
struct std::formatter<geometry::Line> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Line &l, FormatContext &ctx) {
        return std::format_to(ctx.out(), "Line({}, {})", l.start, l.end);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center_p, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Rectangle &r, FormatContext &ctx) {
        return std::format_to(ctx.out(), "Rectangle(bottom_left={}, w={:.2f}, h={:.2f})", r.bottom_left, r.width,
                              r.height);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};
template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.a, t.b, t.c);
    }
};
template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};

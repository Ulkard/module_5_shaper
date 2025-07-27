#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <iterator>
#include <optional>
#include <print>
#include <ranges>
#include <variant>
#include <vector>

using namespace geometry;

namespace rng = std::ranges;
namespace rv = std::ranges::views;
using namespace geometry::queries;

void PrintShapesWithidxAndHeights(const Shapes &shapes) {
    for (size_t i = 0; i < shapes.size(); ++i) {
        std::println("{}. Высота {}. - {}", i, GetHeight(shapes[i]), shapes[i]);
    }
}

void PrintAllIntersections(const Shape &shape, Shapes others) {
    std::println("\n=== Intersections ===");
    std::vector<std::string> text =
        others | rv::filter([](const Shape &other) {
            return std::holds_alternative<Line>(other) || std::holds_alternative<Circle>(other);
        }) |
        rv::transform([&shape](const Shape &other) {
            return std::visit(intersections::IntersectionVisitor{}, shape, other)
                .transform([&](const Point2D &p) {
                    return std::format("Пересечение найдено в точке {} между фигурами {} и {}", p, shape, other);
                })
                .or_else([&]() -> std::optional<std::string> {
                    return std::format("Фигуры {} и {} не пересекаются", shape, other);
                })
                .value();
        }) |
        std::ranges::to<std::vector>();

    for (const auto &line : text) {
        std::println("{}", line);
    }
}

void PrintDistancesFromPointToShapes(Point2D p, Shapes shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("Testing point: {} ", p);

    Shapes random_shapes;
    std::ranges::sample(shapes, std::back_inserter(random_shapes), 5, std::mt19937{std::random_device{}()});

    for (const Shape &shape : random_shapes) {
        std::println("Расстояние от точки {} до фигуры {} равно {}", p, shape, queries::DistanceToPoint(shape, p));
    }
}

void PerformShapeAnalysis(Shapes shapes) {
    std::println("\n=== Shape Analysis ===");
    std::println("AABB collisions:");
    for (const auto &[shape_1, shape_2] : utils::FindAllCollisions(shapes)) {
        std::println("{} x {}", shape_1, shape_2);
    }
    auto highest_result = utils::FindHighestShape(shapes);
    if (highest_result) {
        std::println("\nHighestShape: {}", shapes[*highest_result]);
    }

    // distance
    auto it_1 = std::ranges::find_if(shapes, [](const Shape &shape) { return std::holds_alternative<Circle>(shape); });
    if (it_1 == shapes.end()) {
        return;
    }
    auto it_2 = std::ranges::find_if(std::next(it_1), shapes.end(),
                                     [](const Shape &shape) { return std::holds_alternative<Circle>(shape); });
    if (it_2 == shapes.end()) {
        return;
    }
    std::println("\nРасстояние между {} и {} равно {}", *it_1, *it_2, *DistanceBetweenShapes(*it_1, *it_2));
}

void PerformExtraShapeAnalysis(std::span<const Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");
    Shapes random_shapes;
    std::ranges::sample(shapes | rv::filter([](const Shape &shape) { return GetHeight(shape) > 50; }),
                        std::back_inserter(random_shapes), 3, std::mt19937{std::random_device{}()});

    // в тестовой выборке нет фигур 'выше' 50, max_y - min_y = height ?
    std::println("Фигуры 'выше' 50:");
    for (const auto &shape : random_shapes) {
        std::println("{}", shape);
    }

    if (shapes.empty()) {
        return;
    }

    std::println("\nНаибольшая высота - {}", std::ranges::max(shapes, {}, queries::GetHeight));
    std::println("Наименьшая высота - {}", std::ranges::min(shapes, {}, queries::GetHeight));
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    Shapes shapes = generator.GenerateShapes(15);

    std::println("Generated {} random shapes", shapes.size());

    PrintShapesWithidxAndHeights(shapes);

    PrintAllIntersections(shapes[0], shapes);

    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);

    PerformShapeAnalysis(shapes);

    PerformExtraShapeAnalysis(shapes);

    geometry::visualization::Draw(shapes);

    // Graham
    std::vector<Point2D> graham_result = convex_hull::GrahamScan(shapes).value();
    shapes.push_back(Polygon{graham_result});
    geometry::visualization::Draw(shapes);

    // Bowyer-Watson
    {
        std::vector<Point2D> points = {{0, 0}, {10, 0}, {5, 8}, {15, 5}, {2, 12}};
        const auto delaunay_result = triangulation::DelaunayTriangulation(points);
        geometry::visualization::Draw(delaunay_result.value());
    }
    return 0;
}
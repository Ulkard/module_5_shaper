#pragma once
#include "geometry.hpp"
#include "queries.hpp"
#include <algorithm>
#include <functional>
#include <print>
#include <random>
#include <ranges>
#include <utility>
#include <vector>

namespace geometry::utils {

class ShapeGenerator {
public:
    ShapeGenerator(double min_coord = -100.0, double max_coord = 100.0, double min_size = 1.0, double max_size = 20.0)
        : gen(20), coord_dist(min_coord, max_coord), size_dist(min_size, max_size), sides_dist(3, 12), type_dist(0, 4) {
    }

    Shape GenerateRandomShape() {
        Point2D center{coord_dist(gen), coord_dist(gen)};
        double size = size_dist(gen);

        switch (type_dist(gen)) {
        case 0: {
            Point2D end{center.x + size, center.y + size};
            return Line{center, end};
        }
        case 1: {
            Point2D a{center.x, center.y};
            Point2D b{center.x + size, center.y};
            Point2D c{center.x + size / 2, center.y + size};
            return Triangle{a, b, c};
        }
        case 2: {
            return Rectangle{center, size, size * 0.8};
        }
        case 3: {
            int sides = sides_dist(gen);
            return RegularPolygon{center, size, sides};
        }
        case 4: {
            return Circle{center, size};
        }
        }
        return Circle{center, size};
    }

    Shapes GenerateShapes(size_t count) {
        Shapes shapes;
        shapes.reserve(count);

        for (auto _ : std::views::iota(0u, count)) {
            shapes.push_back(GenerateRandomShape());
        }

        return shapes;
    }

private:
    std::mt19937 gen;
    std::uniform_real_distribution<double> coord_dist;
    std::uniform_real_distribution<double> size_dist;
    std::uniform_int_distribution<int> sides_dist;
    std::uniform_int_distribution<int> type_dist;
};

[[nodiscard]] std::vector<std::pair<Shape, Shape>> FindAllCollisions(const Shapes &shapes) {
    namespace rv = std::ranges::views;
    auto unique_pairs = rv::iota(0u, shapes.size()) | rv::transform([&shapes](size_t i) {
                            return rv::iota(i + 1, shapes.size()) |
                                   rv::transform([i, &shapes](size_t j) { return std::pair{shapes[i], shapes[j]}; });
                        }) |
                        rv::join;
    return unique_pairs |
           rv::filter([](auto &&elem) { return queries::BoundingBoxesOverlap(elem.first, elem.second); }) |
           std::ranges::to<std::vector>();
}

[[nodiscard]] std::optional<size_t> FindHighestShape(const Shapes &shapes) noexcept {
    if (shapes.empty()) {
        return std::nullopt;
    }
    auto it = std::ranges::max_element(shapes, {}, queries::GetHeight);
    return std::distance(shapes.begin(), it);
}

}  // namespace geometry::utils
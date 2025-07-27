#include "geometry.hpp"
#include "intersections.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <optional>
#include <variant>

using geometry::intersections::IntersectionVisitor;
using namespace geometry;

TEST(intersections, line_x_line) {
    const Shape line_1(Line{{0, 0}, {1, 1}});
    const Shape line_2(Line{{1, 0}, {0, 1}});
    const Shape distant_line(Line{{3, 3}, {4, 4}});

    IntersectionVisitor visitor;
    Point2D p{0.5, 0.5};
    EXPECT_DOUBLE_EQ(std::visit(visitor, line_1, line_2)->DistanceTo(p), 0);
    EXPECT_EQ(std::visit(visitor, distant_line, line_2), std::nullopt);
    EXPECT_EQ(std::visit(visitor, line_1, distant_line), std::nullopt);
}

TEST(intersections, line_x_circle) {
    const Shape line(Line{{0, 0}, {2, 2}});
    const Shape circle(Circle{{0, 0}, std::hypot(1, 1)});
    const Shape distant_line(Line{{3, 3}, {4, 4}});
    const Shape distant_circle(Circle{{3, 3}, 1});

    IntersectionVisitor visitor;
    Point2D p{1, 1};
    EXPECT_DOUBLE_EQ(std::visit(visitor, line, circle)->DistanceTo(p), 0);
    EXPECT_DOUBLE_EQ(std::visit(visitor, circle, line)->DistanceTo(p), 0);
    EXPECT_EQ(std::visit(visitor, line, distant_circle), std::nullopt);
    EXPECT_EQ(std::visit(visitor, circle, distant_line), std::nullopt);
}

TEST(intersections, circle_x_circle) {
    const Shape circle_1(Circle{{0, 0}, std::hypot(1, 1)});
    const Shape circle_2(Circle{{2, 0}, std::hypot(1, 1)});
    const Shape distant_circle(Circle{{10, 10}, 1});

    IntersectionVisitor visitor;
    Point2D p{1, 1};
    auto result = *std::visit(visitor, circle_1, circle_2);
    EXPECT_DOUBLE_EQ(result.x, p.x);
    EXPECT_DOUBLE_EQ(result.y, p.y);
    EXPECT_EQ(std::visit(visitor, circle_1, distant_circle), std::nullopt);
    EXPECT_EQ(std::visit(visitor, distant_circle, circle_2), std::nullopt);
}

TEST(intersections, unexpected_type) {
    IntersectionVisitor visitor;
    const Shape valid_type(Line{});
    const Shape unexpected_shape_1(Rectangle{});
    const Shape unexpected_shape_2(Rectangle{});

    EXPECT_ANY_THROW(std::visit(visitor, unexpected_shape_1, unexpected_shape_2));
    EXPECT_ANY_THROW(std::visit(visitor, valid_type, unexpected_shape_2));
    EXPECT_ANY_THROW(std::visit(visitor, unexpected_shape_1, valid_type));
}
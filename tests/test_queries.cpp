#include "geometry.hpp"
#include "queries.hpp"
#include <cstddef>
#include <gtest/gtest.h>

using namespace geometry;
using namespace geometry::queries;

TEST(PointToShapeDistanceVisitor, simple_case) {
    Point2D p(0, 0);
    PointToShapeDistanceVisitor visitor(p);

    const Shape line(Line({1, 0}, {2, 0}));
    const Shape circle(Circle({2, 0}, 1));
    const Shape rect(Rectangle({2, -1}, 2, 2));
    EXPECT_DOUBLE_EQ(std::visit(visitor, line), 1.5);
    EXPECT_DOUBLE_EQ(std::visit(visitor, circle), 2);
    EXPECT_DOUBLE_EQ(std::visit(visitor, rect), 3);
}

TEST(ShapeToShapeDistanceVisitor, line_x_line) {
    ShapeToShapeDistanceVisitor visitor;

    const Shape line_1(Line({0, 0}, {0, 2}));
    const Shape line_2(Line({1, 0}, {2, 10}));
    const Shape crossed_line(Line({-1, 1}, {1, 1}));
    EXPECT_DOUBLE_EQ(*std::visit(visitor, line_1, line_2), 1);
    EXPECT_DOUBLE_EQ(*std::visit(visitor, line_1, crossed_line), 0);
}

TEST(ShapeToShapeDistanceVisitor, circle_x_circle) {
    ShapeToShapeDistanceVisitor visitor;

    const Shape circle_1(Circle({0, 0}, 2));
    const Shape circle_2(Circle({4, 0}, 1));
    const Shape crossed_circle(Circle({3, 0}, 2));
    EXPECT_DOUBLE_EQ(*std::visit(visitor, circle_1, circle_2), 1);
    EXPECT_DOUBLE_EQ(*std::visit(visitor, circle_1, circle_1), 0);
    EXPECT_DOUBLE_EQ(*std::visit(visitor, circle_1, crossed_circle), 0);

    const Shape concentric_circle_1(Circle({0, 0}, 1));
    const Shape concentric_circle_2(Circle({0, 0}, 3));
    EXPECT_DOUBLE_EQ(*std::visit(visitor, circle_1, concentric_circle_1), 0);
    EXPECT_DOUBLE_EQ(*std::visit(visitor, circle_1, concentric_circle_2), 0);
}

TEST(ShapeToShapeDistanceVisitor, unexpected_types) {
    ShapeToShapeDistanceVisitor visitor;

    const Shape valid_type(Line{});
    const Shape unexpected_shape_1(Rectangle{});
    const Shape unexpected_shape_2(Triangle{});

    EXPECT_EQ(std::visit(visitor, unexpected_shape_1, unexpected_shape_2), std::nullopt);
    EXPECT_EQ(std::visit(visitor, valid_type, unexpected_shape_2), std::nullopt);
    EXPECT_EQ(std::visit(visitor, unexpected_shape_1, valid_type), std::nullopt);
}

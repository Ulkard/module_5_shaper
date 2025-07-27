#include "shape_utils.hpp"
#include <gtest/gtest.h>

using namespace geometry;
using namespace geometry::utils;

TEST(ShapeUtils, FindAllCollisions) {
    std::vector<Shape> shapes{Line{{0, 0}, {2, 2}}, Circle{{0, 0}, std::hypot(1, 1)}, Line{{3, 3}, {4, 4}},
                              Circle{{3, 3}, 1}};
    EXPECT_EQ(FindAllCollisions(shapes).size(), 3);

    std::vector<Shape> empty_shapes;
    EXPECT_EQ(FindAllCollisions(empty_shapes).size(), 0);
}

TEST(ShapeUtils, FindHighestShape) {
    std::vector<Shape> shapes{Line{{0, 0}, {2, 2}}, Circle{{0, 0}, 10}, Line{{3, 3}, {4, 4}}, Circle{{3, 3}, 1}};
    EXPECT_EQ(*FindHighestShape(shapes), 1);

    std::vector<Shape> empty_shapes;
    EXPECT_EQ(FindHighestShape(empty_shapes), std::nullopt);
}
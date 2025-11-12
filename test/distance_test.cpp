#include <gtest/gtest.h>

#include <adore_math/distance.h>
#include <adore_math/point.h>
#include <adore_math/polygon.h>

using adore::math::Point2d;

TEST( Distance2D, ZeroDistanceSamePoint )
{
  Point2d p{ 0.0, 0.0 };
  EXPECT_DOUBLE_EQ( adore::math::distance_2d( p, p ), 0.0 );
}

TEST( Distance2D, SimplePythagorean )
{
  Point2d a{ 0.0, 0.0 };
  Point2d b{ 3.0, 4.0 };
  EXPECT_DOUBLE_EQ( adore::math::distance_2d( a, b ), 5.0 );
}

TEST( DistancePointToSegment, PointOnSegment )
{
  Point2d p{ 1.0, 0.0 };
  Point2d s0{ 0.0, 0.0 };
  Point2d s1{ 2.0, 0.0 };

  double d = adore::math::distance_point_to_segment( p, s0, s1 );
  EXPECT_DOUBLE_EQ( d, 0.0 );
}

TEST( DistancePointToSegment, PointAboveSegment )
{
  Point2d p{ 0.0, 1.0 };
  Point2d s0{ 0.0, 0.0 };
  Point2d s1{ 2.0, 0.0 };

  double d = adore::math::distance_point_to_segment( p, s0, s1 );
  EXPECT_DOUBLE_EQ( d, 1.0 );
}

TEST( PolygonDistance, DisjointUnitSquares )
{
  adore::math::Polygon2d a;
  a.points = {
    { 0.0, 0.0 },
    { 1.0, 0.0 },
    { 1.0, 1.0 },
    { 0.0, 1.0 },
  };

  adore::math::Polygon2d b;
  b.points = {
    { 2.0, 0.0 },
    { 3.0, 0.0 },
    { 3.0, 1.0 },
    { 2.0, 1.0 },
  };

  double d = adore::math::polygon_distance( a, b );
  EXPECT_DOUBLE_EQ( d, 1.0 );
}

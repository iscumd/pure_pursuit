#include "pure_pursuit.h"
#include "catch.hh"

bool approximately_equals( Point3D expected, Point3D actual )
{
    return ( actual.x == Approx( expected.x ).margin( 1e-3 ) )
        && ( actual.y == Approx( expected.y ).margin( 1e-3 ) )
        && ( actual.z == Approx( expected.z ).margin( 1e-3 ) );
}

class PurePursuitTest : private PurePursuit
{
public:
    PurePursuitTest( const Path& robot_path, const double& lookahead_distance )
        : PurePursuit( robot_path, lookahead_distance )
    {
    }

    Point2D get_target_state_test( const Point3D& state )
    {
        return get_target_state( state );
    }

    void reset_path_test( const Path& robot_path ) { reset_path( robot_path ); }

    void reset_lookahead_distance_test( const double& lookahead_distance )
    {
        reset_lookahead_distance( lookahead_distance );
    }

    Point3D get_lookahead_point_test( const Point3D& state )
    {
        return get_lookahead_point( state );
    }

    std::pair<Point2D, Point2D>
    get_ongoing_path_segment_test( const int& current_segment )
    {
        return get_ongoing_path_segment( current_segment );
    }

    Point3D get_point_on_path_test( const double& position )
    {
        return get_point_on_path( position );
    }

    std::pair<Point2D, double> get_location_on_path_test( const Point2D& state )
    {
        return get_location_on_path( state );
    }
};

TEST_CASE( "Test get location on path", "[path_location]" )
{
    SECTION( "Simple Test" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );

        auto location = test_class.get_location_on_path_test( { 5, 5 } );
        CHECK( location.first == Point2D( 5, 0 ) );

        location = test_class.get_location_on_path_test( { 3, 3 } );
        CHECK( location.first == Point2D( 3, 0 ) );
    }

    SECTION( "Edge Case" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto location = test_class.get_location_on_path_test( { 11, 0 } );
        CHECK( location.first == Point2D( 10, 0 ) );
    }
}

TEST_CASE( "Test get point on path", "[point_path]" )
{
    SECTION( "Simple Test" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 }, { 20, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 15 );
        CHECK( point == Point3D( 15, 0, 10 ) );
    }

    SECTION( "Complex Path" )
    {
        Point3D exp = { 3.528, 5.3704, 10.7864 };
        Path p = { { 0, 3, 0 }, { 2, 1, 3 }, { 4, 6, 10 }, { 1, 2, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 9 );
        CHECK( approximately_equals( exp, point ) );
    }

    SECTION( "Parallel to y axis" )
    {
        Point3D exp = { 3, 5.694449, 11.04167 };
        Path p      = { { 1, 2, 0 }, { 3, 5, 10 }, { 3, 7, 13 }, { 4, 9, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 4.3 );
        CHECK( approximately_equals( exp, point ) );
    }

    SECTION( "Edge Case: Position too high" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 }, { 20, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 43 );
        CHECK( point == Point3D( 20, 0, 10 ) );
    }

    SECTION( "Edge Case: Position is Negative" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 }, { 20, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( -15 );
        CHECK( point == Point3D( 0, 0, 0 ) );
    }
}



TEST_CASE( "Test get distance to point", "[distance_point]" )
{
    SECTION( "Simple Test" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 }, { 20, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 15 );
        CHECK( point == Point3D( 15, 0, 10 ) );
    }

    SECTION( "Complex Path" )
    {
        Point3D exp = { 3.528, 5.3704, 10.7864 };
        Path p = { { 0, 3, 0 }, { 2, 1, 3 }, { 4, 6, 10 }, { 1, 2, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 9 );
        CHECK( approximately_equals( exp, point ) );
    }

    SECTION( "Parallel to y axis" )
    {
        Point3D exp = { 3, 5.694449, 11.04167 };
        Path p      = { { 1, 2, 0 }, { 3, 5, 10 }, { 3, 7, 13 }, { 4, 9, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 4.3 );
        CHECK( approximately_equals( exp, point ) );
    }

    SECTION( "Edge Case: Position too high" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 }, { 20, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 43 );
        CHECK( point == Point3D( 20, 0, 10 ) );
    }

    SECTION( "Edge Case: Position is Negative" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 }, { 20, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( -15 );
        CHECK( point == Point3D( 0, 0, 0 ) );
    }
}
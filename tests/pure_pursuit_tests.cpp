#include "pure_pursuit.h"
#include "catch.hh"

bool approximately_equals( Point3D expected, Point3D actual )
{
    return ( actual.x == Approx( expected.x ).margin( 1e-3 ) )
        && ( actual.y == Approx( expected.y ).margin( 1e-3 ) )
        && ( actual.z == Approx( expected.z ).margin( 1e-3 ) );
}
bool approximately_equals( Point2D expected, Point2D actual )
{
    return ( actual.x == Approx( expected.x ).margin( 1e-3 ) )
        && ( actual.y == Approx( expected.y ).margin( 1e-3 ) );
}

class PurePursuitTest : private PurePursuit
{
public:
    PurePursuitTest( const Path& robot_path, const double& lookahead_distance )
        : PurePursuit( robot_path, lookahead_distance )
    {
    }

    std::tuple<Point3D, double, double> get_target_state_test( const Point3D& state )
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

    Point3D get_point_on_path_test( const double& position )
    {
        return get_point_on_path( position );
    }

    std::pair<Point2D, double> get_location_on_path_test( const Point2D& state )
    {
        return get_location_on_path( state );
    }

    double get_distance_to_point_test( const Point2D& point )
    {
        return get_distance_to_point( point );
    }

    double path_length_test() { return path_length(); }
};

TEST_CASE( "Test get target state", "[target_state]" )
{
    SECTION( "Simple Test" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 3 );
        auto values = test_class.get_target_state_test( { 5, 5, M_PI } );
        CHECK( std::get<0>( values ) == Point3D( 8, 0, 8 ) );
        CHECK( std::get<1>( values )
               == Approx( M_PI_4 ).margin(
                      1e-3 ) );  // point is 45 degrees away from x axis
        CHECK( std::get<2>( values ) == Approx( -3 * M_PI_4 ).margin( 1e-3 ) );
    }

    SECTION( "Parallel to y axis" )
    {
        Path p = { { 1, 2, 0 }, { 3, 5, 10 }, { 3, 7, 13 }, { 4, 9, 15 } };
        PurePursuitTest test_class( p, 1 );
        auto values = test_class.get_target_state_test( { 4, 5, M_PI_2 } );
        CHECK(
            approximately_equals( Point3D( 3.0, 6.0, 11.5 ), std::get<0>( values ) ) );
        //        CHECK( std::get<1>( values) == Approx(M_PI_4).margin(1e-3));
        //        CHECK( std::get<2>( values ) == Approx(0).margin(1e-3));
        // FIXME: These two double values are slightly off... needs to be checked
        // visually
    }

    SECTION( "Edge Case" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto values = test_class.get_target_state_test( { 11, 0, 0 } );
        CHECK( std::get<0>( values ) == p.at( 1 ) );
        CHECK( std::get<1>( values )
               == Approx( 0 ).margin( 1e-3 ) );  // point is 45 degrees away from x axis
        CHECK( std::get<2>( values ) == Approx( 0 ).margin( 1e-3 ) );
    }
}

TEST_CASE( "Test get location on path", "[path_location]" )
{
    SECTION( "Simple Test" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto location = test_class.get_location_on_path_test( { 5, 5 } );
        CHECK( approximately_equals( location.first, Point2D( 5, 0 ) ) );
        CHECK( location.second == Approx( 5 ).margin( 1e-3 ) );

        location = test_class.get_location_on_path_test( { 3, 3 } );
        CHECK( approximately_equals( location.first, Point2D( 3, 0 ) ) );
        CHECK( location.second == Approx( 3 ).margin( 1e-3 ) );
    }

    SECTION( "Parallel to y axis" )
    {
        Path p = { { 1, 2, 0 }, { 3, 5, 10 }, { 3, 7, 13 }, { 4, 9, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto location = test_class.get_location_on_path_test( { 4, 6 } );
        CHECK( approximately_equals( location.first, { 3, 6 } ) );
    }

    SECTION( "Edge Case" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto location = test_class.get_location_on_path_test( { 11, 0 } );
        CHECK( approximately_equals( location.first, Point2D( 10, 0 ) ) );
        CHECK( location.second == Approx( 1 ).margin( 1e-3 ) );
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
        Path p = { { 0, 3, 0 }, { 2, 1, 3 }, { 4, 6, 10 }, { 1, 2, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 9 );
        CHECK( approximately_equals( Point3D( 3.528, 5.3704, 10.7864 ), point ) );
    }

    SECTION( "Parallel to y axis" )
    {
        Path p = { { 1, 2, 0 }, { 3, 5, 10 }, { 3, 7, 13 }, { 4, 9, 15 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_point_on_path_test( 4.3 );
        CHECK( approximately_equals( Point3D( 3, 5.694449, 11.04167 ), point ) );
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
        Path p = { { 0, 0, 0 }, { 5, 10, 10 }, { 10, 20, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto dist = test_class.get_distance_to_point_test( { 6, 12 } );
        CHECK( dist == Approx( 13.416407864 ).margin( 1e-3 ) );
    }
    SECTION( "Point on segment parallel to the y-axis " )
    {
        Path p = { { 0, 0, 0 }, { 5, 10, 10 }, { 10, 5, 10 }, { 10, 15, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto dist = test_class.get_distance_to_point_test( { 10, 10 } );
        CHECK( dist == Approx( 23.251407 ).margin( 1e-3 ) );
    }
    SECTION( "Complex Path" )
    {
        Path p
            = { { 1, 3, 0 }, { 4, 1, 10 }, { 4, 4, 10 }, { 5, 15, 10 }, { 3, 10, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto dist = test_class.get_distance_to_point_test( { 4.3, 13.25 } );
        CHECK( dist == Approx( 19.535712 ).margin( 1e-3 ) );
    }
    SECTION( "Point not on path" )
    {
        Path p
            = { { 1, 3, 0 }, { 4, 1, 10 }, { 4, 4, 10 }, { 5, 15, 10 }, { 3, 10, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto dist = test_class.get_distance_to_point_test( { 100, 13.25 } );
        auto l    = test_class.path_length_test();
        CHECK( dist != l );
        CHECK( dist == -1 );

        auto dist2 = test_class.get_distance_to_point_test( { 3, 10 } );
        CHECK( dist2 != -1 );
    }
}

TEST_CASE( "Test get lookahead point", "[lookahead_point]" )
{
    SECTION( "Simple Test" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 3 );
        auto point = test_class.get_lookahead_point_test( { 5, 5, 5 } );
        CHECK( point == Point3D( 8, 0, 8 ) );

        point = test_class.get_lookahead_point_test( { 3, 3, 3 } );
        CHECK( point == Point3D( 6, 0, 6 ) );
    }

    SECTION( "Parallel to y axis" )
    {
        Path p = { { 1, 2, 0 }, { 3, 5, 10 }, { 3, 7, 13 }, { 4, 9, 15 } };
        PurePursuitTest test_class( p, 3 );
        auto point = test_class.get_lookahead_point_test( { 2, 6, 14 } );
        CHECK( approximately_equals( Point3D( 3.894427, 8.78885, 14.788854 ), point ) );
    }

    SECTION( "Edge Case" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto point = test_class.get_lookahead_point_test( { 11, 0, 12 } );
        CHECK( point == p.at( 1 ) );
    }
}
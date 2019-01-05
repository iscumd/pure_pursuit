#include "pure_pursuit.h"
#include "catch.hh"

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
        REQUIRE( location.first == Point2D( 5, 0 ) );

        location = test_class.get_location_on_path_test( { 3, 3 } );
        REQUIRE( location.first == Point2D(  3, 0  ) );
    }

    SECTION( "Edge Case" )
    {
        Path p = { { 0, 0, 0 }, { 10, 0, 10 } };
        PurePursuitTest test_class( p, 5 );
        auto location = test_class.get_location_on_path_test( { 11, 0 } );
        REQUIRE( location.first == Point2D( 10, 0 ) );
    }
}

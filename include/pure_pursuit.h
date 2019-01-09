/**
 * @file pure_pursuit.h
 * @brief Path Tracking Program for a robot
 * @author Aaron Cofield, JessRose Narsinghia, Andrew R. Davis
 * @bug TO BE MARKED
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <tuple>
#include <vector>

#define EPSILON 1e-3

/**
 * @brief data structure to represent x and y coordinates for the robot
 */
struct Point2D
{
    double x, y;
    Point2D( const double& ix, const double& iy )
        : x( ix )
        , y( iy )
    {
    }
    Point2D() {}
};

bool operator==( const Point2D& lhs, const Point2D& rhs )
{
    return ( lhs.x == rhs.x && lhs.y == rhs.y );
}

/**
 * @brief: data structure to present x, y, and z (velocity) of the robot
 */
struct Point3D
{
    double x, y, z;
    Point3D( const double& ix, const double& iy, const double& iz )
        : x( ix )
        , y( iy )
        , z( iz )
    {
    }
    Point3D() {}

    Point2D to2D() { return Point2D( x, y ); }
};

bool operator==( const Point3D& lhs, const Point3D& rhs )
{
    return ( lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z );
}

typedef std::vector<Point3D> Path;
typedef std::pair<Point3D, Point3D> Segment3D;
typedef std::pair<Point2D, Point2D> Segment2D;

class PurePursuit
{
public:
    PurePursuit( const Path& robot_path, const double& lookahead_distance );

    /**
    * @brief Find the targer state (point and velocity) of the robot
    * @param state a Point3D where x and y are the position of the bot and z is heading
    * @return return Point3D that is lookahead point, first double is heading to point,
    * second double is the heading error
    */
    std::tuple<Point3D, double, double> get_target_state( const Point3D& state );

    /**
    * @brief will reset the path the robot must follow to a new robot path
    * @param new path for the robot to follow as robot_path
    */
    void reset_path( const Path& robot_path );

    /**
    * @brief will reset lookahead distance
     * @param the new lookahead distance
    */
    void reset_lookahead_distance( const double& lookahead_distance );

protected:
    /**
    * @brief Will get the coordinates and target velocity of the lookahead point
    * @param the current location of the robot
     * @return Point3D variable for a point lookahead distance ahead of the state
    */
    Point3D get_lookahead_point( const Point3D& state );

    /**
    * @brief Will get the point on the path that corresponds to the distance form
    * beginning of path
     * @param position double value to represent point position
     * @return the point3D that is on the path
    */
    Point3D get_point_on_path( const double& position );

    /**
    * @brief will get the robots location on a path segment relevant to the location of
    * the robot
     * @param point of the robot (its current location_
     * @return  first value is the x,y location coordinate of robot corresponding with
    * the path. and the second value is a double distance between robot and the path
    */
    std::pair<Point2D, double> get_location_on_path( const Point2D& state );

    /**
     * @brief will find total length of robot path
     * @return total distance of path from first point to last point
     */
    double path_length();

    /**
     * @brief gets total distance from first point on path to point on path
     * @param point2D that exists on the path
     * @return distance double from first point on Path to the parameter point
     */
    double get_distance_to_point( const Point2D& currPoint );

    double m_lookahead_distance;
    Path m_robot_path;
};

#endif
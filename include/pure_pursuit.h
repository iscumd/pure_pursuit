/**
 * @file pure_pursuit.h
 * @brief Path Tracking Program for a robot
 * @author Aaron Cofield, JessRose Narsinghia, Andrew R. Davis
 * @bug TO BE MARKED
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <vector>

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
};

bool operator==( const Point3D& lhs, const Point3D& rhs )
{
    return ( lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z );
}

typedef std::vector<Point3D> Path;
typedef std::pair<Point3D, Point3D> segment3D;
typedef std::pair<Point2D, Point2D> segment2D;

class PurePursuit
{
public:
    PurePursuit( const Path& robot_path, const double& lookahead_distance );

    /**
    * @brief
    * @param state a Point3D where x and y are the position of the bot and z is the orientation
    * @return a target linear and angular velocity as a Point2D where x is the linear velocity and y is the angula velocity.
    */
    Point2D get_target_state( const Point3D& state );

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
    * @brief Will get the coordinates to the ongoing path segment where the first value is the point that has the smallest index in the path of the two values
    */
    std::pair<Point2D, Point2D> get_ongoing_path_segment( const int& current_segment );

    /**
    * @brief Will get point3D on path according to path location
    */
    Point3D get_point_on_path( const double& position );

    /**
    * @brief will get the robots location on a path segment where the first value is the
    * x,y coordinate of the
     * Search for the shortest distance from the robot to each path segment
    */
    Point2D get_location_on_path( const Point2D& state );

    /**
     * @brief will find length of robot path
     * uses distance formula
     *
     */
    double path_length();

    /**
     *
     * @pbrief parameter is a point that exists on the path, function will return the
     * distance from first point on Path
     * to the parameter point
     *
     */
    double get_distance_to_point( const Point2D& currPoint );

    double m_lookahead_distance;
    Path m_robot_path;
};

#endif
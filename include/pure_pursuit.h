#include <vector>

struct Point2D{
	double x,y;
};

struct Point3D{
	double x,y,x;
};

typedef Path3D std::vector<Point3D>;

class PurePursuit{
public:
	PurePursuit(const Path& robot_path, const double& lookahead_distance);

	/**
	* @brief Will return a target linear and angular velocity as a Point2D 
	* where x is the linear velocity and y is the angular velocity
	* 
	* @input state a Point3D where x and y are the position of the bot and z is the orientation
	* 
	* Implement waypoint following code here
	*/
	Point2D get_target_state(const Point3D& state);

	/**
	* @brief will reset the path the robot must follow to a new robot path
	*/
	void reset_path(const Path& robot_path);

	/**
	* @brief will reset lookahead distance
	*/
	void reset_lookahead_distance(const double& lookahead_distance);

private:
	/**
	* @brief Will get the coordinates and target velocity of the lookahead point
	*/
	Point3D get_lookahead_point(const Point3D& state);

	/**
	* @brief Will get the coordinates to the ongoing path segment where the first value is
	* the point that has the smallest index in the path of the two values
	*/
	std::pair<Point2D,Point2D> get_ongoing_path_segment(const int& current_segment);

	/**
	* @brief Will get point3D on path according to path location
	*/
	Point3D get_point_on_path(const double& position);

	/**
	* @brief will get the robots location on a path segment where the first value is the x,y coordinate of the
	* path location. and the second value is a double that represents its location on a path
	*/
	std::pair<Point2D,double> get_location_on_path(const Point2D& state){
	//Seach for the shortest distance from the robot to each path segment
	}

	double m_lookahead_distance;
	Path m_robot_path;
};

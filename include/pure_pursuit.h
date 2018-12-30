#include <vector>
#include <cmath>

struct Point2D{
	double x,y;
};

struct Point3D{
	double x, y, z;
};

typedef std::vector<Point3D> Path;

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
		double counter = 0;
		double slope, distanceTraveled, distanceToRobot, shortestDistance = INT16_MAX;
		Point2D closestLocation, currentLoc;
		closestLocation.x = m_robot_path[0].x;
		closestLocation.y = m_robot_path[0].y;
		currentLoc.x = m_robot_path[0].x;
		currentLoc.y = m_robot_path[0].y;

		for (int i = 0; i < (m_robot_path.size() - 1); i++) {

			slope = ((m_robot_path[i + 1].y - m_robot_path[i].y) / (m_robot_path[i + 1].x - m_robot_path[i].x));
			counter = 0;

			while ((currentLoc.x != m_robot_path[i + 1].x) || (currentLoc.y != m_robot_path[i + 1].y)) {
				//segment is going up and right
				if ((m_robot_path[i].y < m_robot_path[i + 1].y) && (m_robot_path[i].x < m_robot_path[i + 1].x)) {
					counter++;

					currentLoc.x = (m_robot_path[i].x + counter);
					currentLoc.y = (currentLoc.y + slope);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.x >= m_robot_path[i + 1].x) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}
					if (currentLoc.y >= m_robot_path[i + 1].y) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//segment is going up and to the left
				else if ((m_robot_path[i].y < m_robot_path[i + 1].y) && (m_robot_path[i].x > m_robot_path[i + 1].x)) {
					counter--;

					currentLoc.x = (m_robot_path[i].x + counter);
					currentLoc.y = (currentLoc.y + -slope);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.x <= m_robot_path[i + 1].x) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}
					if (currentLoc.y >= m_robot_path[i + 1].y) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//segment is going down and to the left
				else if ((m_robot_path[i].y > m_robot_path[i + 1].y) && (m_robot_path[i].x > m_robot_path[i + 1].x)) {
					counter--;

					currentLoc.x = (m_robot_path[i].x + counter);
					currentLoc.y = (currentLoc.y + -slope);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.x <= m_robot_path[i + 1].x) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}
					if (currentLoc.y <= m_robot_path[i + 1].y) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//segment is going down and to the right
				else if ((m_robot_path[i].y > m_robot_path[i + 1].y) && (m_robot_path[i].x < m_robot_path[i + 1].x)) {
					counter++;

					currentLoc.x = (m_robot_path[i].x + counter);
					currentLoc.y = (currentLoc.y + slope);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.x >= m_robot_path[i + 1].x) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}
					if (currentLoc.y <= m_robot_path[i + 1].y) {
						currentLoc.x = m_robot_path[i + 1].x;
						currentLoc.y = m_robot_path[i + 1].y;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//segment is going up
				else if ((m_robot_path[i].y < m_robot_path[i + 1].y) && (m_robot_path[i].x == m_robot_path[i + 1].x)) {
					counter++;

					currentLoc.y = (currentLoc.y + counter);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.y >= m_robot_path[i + 1].y) {
						currentLoc.y = m_robot_path[i + 1].y;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//segment is going right
				else if ((m_robot_path[i].y == m_robot_path[i + 1].y) && (m_robot_path[i].x < m_robot_path[i + 1].x)) {
					counter++;

					currentLoc.x = (currentLoc.x + counter);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.x >= m_robot_path[i + 1].x) {
						currentLoc.x = m_robot_path[i + 1].x;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//segment is going down
				else if ((m_robot_path[i].y > m_robot_path[i + 1].y) && (m_robot_path[i].x == m_robot_path[i + 1].x)) {
					counter--;

					currentLoc.y = (currentLoc.y + counter);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.y <= m_robot_path[i + 1].y) {
						currentLoc.y = m_robot_path[i + 1].y;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}

					std::cout << "Current Location: " << currentLoc.x << "," << currentLoc.y << " DistanceToRobot: " << distanceToRobot << std::endl;//do not keep, for testing  DELETE
				}
				//segment is going left
				else if ((m_robot_path[i].y == m_robot_path[i + 1].y) && (m_robot_path[i].x > m_robot_path[i + 1].x)) {
					counter--;

					currentLoc.x = (currentLoc.x + counter);

					//if currentLocation goes beyond next point, it is set to the next point
					if (currentLoc.x <= m_robot_path[i + 1].x) {
						currentLoc.x = m_robot_path[i + 1].x;
					}

					//calculates distance to robot
					distanceToRobot = sqrt(pow(abs(-currentLoc.x + state.x), 2) + pow(abs(-currentLoc.y + state.y), 2));

					//checks to see if currentLocation is closer to the current position
					if (distanceToRobot < shortestDistance) {
						shortestDistance = distanceToRobot;
						closestLocation = currentLoc;
					}
				}
				//Previous point = next point. Should not happen, assumed mistake.
				else {
					break;
				}
			}
		}

		std::pair<Point2D, double> returnVal;
		returnVal.first = closestLocation;
		returnVal.second = 0;//distanceTraveled;

		return returnVal;
	}

	double m_lookahead_distance;
	Path m_robot_path;
};

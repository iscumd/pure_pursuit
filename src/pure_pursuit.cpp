// implement class here

std::pair<Point2D, double> get_location_on_path(const Point2D& state) {
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
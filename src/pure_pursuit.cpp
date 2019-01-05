// implement class here
#include "pure_pursuit.h"

PurePursuit::PurePursuit(const Path &robot_path, const double &lookahead_distance): m_robot_path(robot_path), m_lookahead_distance(lookahead_distance) {}

Point2D PurePursuit::get_target_state(const Point3D &state) {}

void PurePursuit::reset_path(const Path &robot_path) {}

void PurePursuit::reset_lookahead_distance(const double &lookahead_distance) {}

Point3D PurePursuit::get_lookahead_point(const Point3D &state) {}

std::pair<Point2D, Point2D> PurePursuit::get_ongoing_path_segment(const int &current_segment) {}

Point3D PurePursuit::get_point_on_path(const double &position) {}

std::pair<Point2D, double> PurePursuit::get_location_on_path(const Point2D &state) {}
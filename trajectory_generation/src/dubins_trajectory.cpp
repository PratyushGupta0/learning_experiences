#include <trajectory_generation/dubins_trajectory.hpp>

namespace ariitk::trajectory_generation {

DubinsTrajectory::DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    publish_trajectory_server_ = nh.advertiseService("command", &DubinsTrajectory::commandServiceCallback, this);
    publish_trajectory_client_ = nh.serviceClient<std_srvs::Trigger>("command");

    DubinsTrajectory::loadParams(nh_private);
    DubinsTrajectory::computePoints();
    DubinsTrajectory::generateTrajectory();
}
float DubinsTrajectory::dist(float x, float y, float x1, float y1) {
    float den = sqrt(x * x + y * y);
    float num = fabs(x * y1 - x1 * y);
    return num / den;
}
void DubinsTrajectory::loadParams(ros::NodeHandle& nh_private) {
    nh_private.param("x_way", end_pos_.x(), 0.0);
    nh_private.param("y_way", end_pos_.y(), 0.0);
    nh_private.param("z_way", end_pos_.z(), 0.0);
    nh_private.param("radius", radius);
    nh_private.param("centre_x", centre_x);
    nh_private.param("centre_y", centre_y);
    nh_private.param("distance_", distance_);
    nh_private.param("launch_position_x", launch_pos_.x(), 0.0);
    nh_private.param("launch_position_y", launch_pos_.y(), 0.0);
    nh_private.param("launch_position_z", launch_pos_.z(), 0.0);
    nh_private.param("v_max", v_max);
    nh_private.param("a_max", a_max);
}
void DubinsTrajectory::computePoints() {
    ROS_INFO("Calculating waypoints for the victorious forward journey");
    mav_trajectory_generation::Vertex start(3), end(3);
    derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;
    start.makeStartOrEnd(launch_pos_, derivative_to_optimize_);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, 0.0);
    end.makeStartOrEnd(end_pos_, derivative_to_optimize_);

    vertices_.push_back(start);
    using Line2 = Eigen::Hyperplane<float, 2>;
    using Vec2 = Eigen::Vector2f;
    Vec2 a(launch_pos_.x(), launch_pos_.y());
    Vec2 b(end_pos_.x(), end_pos_.y());
    Vec2 c(centre_x, centre_y);
    Vec2 d(centre_x + launch_pos_.y(), centre_y - launch_pos_.x());

    Line2 ac = Line2::Through(a, c);
    Line2 bd = Line2::Through(b, d);

    float x1 = (ac.intersection(bd))[0];
    float y1 = (ac.intersection(bd))[1];
    float distance = dist(end_pos_.x(), end_pos_.y(), centre_x, centre_y);
    float up = sqrt((radius + 1) * (radius + 1) - distance * distance);
    float angle = atan(end_pos_.y() / end_pos_.x());

    float cos = end_pos_.x() / sqrt((end_pos_.x() * end_pos_.x() + end_pos_.y() * end_pos_.y()));
    float sin = end_pos_.y() / sqrt((end_pos_.x() * end_pos_.x() + end_pos_.y() * end_pos_.y()));

    Point first(x1 + up * cos, y1 + up * sin, launch_pos_.z(), angle);
    Point third(x1 - up * cos, y1 - up * sin, launch_pos_.z(), angle);

    cos = -end_pos_.y() / sqrt((end_pos_.x() * end_pos_.x() + end_pos_.y() * end_pos_.y()));
    sin = end_pos_.x() / sqrt((end_pos_.x() * end_pos_.x() + end_pos_.y() * end_pos_.y()));
    Point second(centre_x - (radius + 1) * cos, centre_y - (radius + 1) * sin, launch_pos_.z(), angle);
    mav_trajectory_generation::Vertex p1(3), p2(3), p3(3);
    p1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, first.position);
    p2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, second.position);
    p3.addConstraint(mav_trajectory_generation::derivative_order::POSITION, third.position);
    vertices_.push_back(p1);
    vertices_.push_back(p2);
    vertices_.push_back(p3);
    vertices_.push_back(end);
}
void DubinsTrajectory::generateTrajectory() {
    segment_times_ = mav_trajectory_generation::estimateSegmentTimes(vertices_, v_max, a_max);

    mav_trajectory_generation::PolynomialOptimization<10> opt(3);
    opt.setupFromVertices(vertices_, segment_times_, derivative_to_optimize_);
    opt.solveLinear();
    opt.getTrajectory(&trajectory_);

    std::string frame_id = "world";  // LOOK HERE
    mav_trajectory_generation::drawMavTrajectory(trajectory_, distance_, frame_id, &markers_);
}

bool DubinsTrajectory::commandServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    trajectory_msgs::MultiDOFJointTrajectory generated_trajectory;

    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, 0.1, &trajectory_points);
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &generated_trajectory);

    if (command_) {
        trajectory_pub_.publish(generated_trajectory);
    }

    resp.success = true;
    resp.message = "Trajectory given as command";
    ROS_INFO("%s\n", resp.message.c_str());
    return true;
}

void DubinsTrajectory::run() {
    if (visualize_) {
        marker_pub_.publish(markers_);
    }
}
}  // namespace ariitk::trajectory_generation

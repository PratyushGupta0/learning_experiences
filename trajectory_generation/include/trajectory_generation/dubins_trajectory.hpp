#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

namespace ariitk::trajectory_generation {

struct Point {
    Point(const Eigen::Vector3d& pos, const double& yaw = 0.0)
        : position(pos)
        , yaw(yaw) {
    }

    Point(const double& x, const double& y, const double& z, const double& yaw = 0.0)
        : Point(Eigen::Vector3d(x, y, z), yaw) {
    }

    Point()
        : position(Eigen::Vector3d(0, 0, 0))
        , yaw(0.0) {
    }

    Eigen::Vector3d position;
    double yaw = 0.0;
};

class DubinsTrajectory {
  public:
    DubinsTrajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void generateTrajectory();
    void run();

  private:
    void loadParams(ros::NodeHandle& nh_private);
    void computePoints();
    float dist(float x, float y, float x1, float y1);
    bool commandServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
    float radius;
    float centre_x;
    float centre_y;
    float v_max;
    float a_max;
    double distance_;
    int derivative_to_optimize_;

    bool visualize_;
    bool command_;

    mav_trajectory_generation::Vertex::Vector vertices_;
    mav_trajectory_generation::Trajectory trajectory_;
    std::vector<double> segment_times_;
    Eigen::Vector3d launch_pos_;
    Eigen::Vector3d end_pos_;

    visualization_msgs::MarkerArray markers_;
    ros::Publisher marker_pub_;
    ros::Publisher trajectory_pub_;
    ros::ServiceServer publish_trajectory_server_;
    ros::ServiceClient publish_trajectory_client_;
};

}  // namespace ariitk::trajectory_generation
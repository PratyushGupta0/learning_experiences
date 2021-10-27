# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;mav_msgs;std_srvs;visualization_msgs;mav_trajectory_generation_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltrajectory_generation".split(';') if "-ltrajectory_generation" != "" else []
PROJECT_NAME = "trajectory_generation"
PROJECT_SPACE_DIR = "/home/pratyush/suas_ws/install"
PROJECT_VERSION = "0.0.0"

# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;nav_msgs;four_wheel_steering_msgs;realtime_tools;tf;urdf_geometry_parser;pluginlib".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfour_wheel_steering_controller".split(';') if "-lfour_wheel_steering_controller" != "" else []
PROJECT_NAME = "four_wheel_steering_controller"
PROJECT_SPACE_DIR = "/home/hugoc/scout_ws/install"
PROJECT_VERSION = "0.17.0"

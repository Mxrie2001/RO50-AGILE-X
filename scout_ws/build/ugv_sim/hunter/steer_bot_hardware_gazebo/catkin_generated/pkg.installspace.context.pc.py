# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;angles;control_toolbox;gazebo_ros_control;hardware_interface;joint_limits_interface".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsteer_bot_hardware_gazebo".split(';') if "-lsteer_bot_hardware_gazebo" != "" else []
PROJECT_NAME = "steer_bot_hardware_gazebo"
PROJECT_SPACE_DIR = "/home/hugoc/scout_ws/install"
PROJECT_VERSION = "0.1.0"

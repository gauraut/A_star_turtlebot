cmake_minimum_required(VERSION 2.8.3)

project(project_3)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation tf rostest geometry_msgs sensor_msgs message_generation tf2 tf2_msgs tf2_ros tf2_geometry_msgs gazebo_msgs cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

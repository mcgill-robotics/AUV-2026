cmake_minimum_required(VERSION 3.8)
project(auv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(MSG_DEP_SET 
	geometry_msgs 
	std_msgs
	actionlib_msgs
)

set(msg_files	
	"msg/DeadReckonReport.msg"
	"msg/PingerBearing.msg"
	"msg/PingerTimeDifference.msg"
	"msg/ThrusterForces.msg"
	"msg/ThrusterMicroseconds.msg"
	"msg/UnityState.msg"
	"msg/VelocityReport.msg"
	"msg/VisionObject.msg"
	"msg/VisionObjectArray.msg"
)

set(srv_files
	"srv/SetInt32.srv"
	"srv/AutomaticCapture.srv"
)

set(action_files
	"action/Effort.action"
	"action/StateQuaternion.action"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(dep IN LISTS MSG_DEP_SET)
	find_package(${dep} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	${action_files}
	DEPENDENCIES ${MSG_DEP_SET}
)

ament_export_dependencies(${MSG_DEP_SET})

ament_package()

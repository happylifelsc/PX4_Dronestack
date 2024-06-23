# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dronestack: 1 messages, 1 services")

set(MSG_I_FLAGS "-Idronestack:/home/cpsl/catkin_ws/src/dronestack/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dronestack_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" NAME_WE)
add_custom_target(_dronestack_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dronestack" "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" NAME_WE)
add_custom_target(_dronestack_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dronestack" "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dronestack
)

### Generating Services
_generate_srv_cpp(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dronestack
)

### Generating Module File
_generate_module_cpp(dronestack
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dronestack
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dronestack_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dronestack_generate_messages dronestack_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(dronestack_generate_messages_cpp _dronestack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" NAME_WE)
add_dependencies(dronestack_generate_messages_cpp _dronestack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dronestack_gencpp)
add_dependencies(dronestack_gencpp dronestack_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dronestack_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dronestack
)

### Generating Services
_generate_srv_eus(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dronestack
)

### Generating Module File
_generate_module_eus(dronestack
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dronestack
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dronestack_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dronestack_generate_messages dronestack_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(dronestack_generate_messages_eus _dronestack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" NAME_WE)
add_dependencies(dronestack_generate_messages_eus _dronestack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dronestack_geneus)
add_dependencies(dronestack_geneus dronestack_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dronestack_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dronestack
)

### Generating Services
_generate_srv_lisp(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dronestack
)

### Generating Module File
_generate_module_lisp(dronestack
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dronestack
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dronestack_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dronestack_generate_messages dronestack_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(dronestack_generate_messages_lisp _dronestack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" NAME_WE)
add_dependencies(dronestack_generate_messages_lisp _dronestack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dronestack_genlisp)
add_dependencies(dronestack_genlisp dronestack_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dronestack_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dronestack
)

### Generating Services
_generate_srv_nodejs(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dronestack
)

### Generating Module File
_generate_module_nodejs(dronestack
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dronestack
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dronestack_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dronestack_generate_messages dronestack_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(dronestack_generate_messages_nodejs _dronestack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" NAME_WE)
add_dependencies(dronestack_generate_messages_nodejs _dronestack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dronestack_gennodejs)
add_dependencies(dronestack_gennodejs dronestack_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dronestack_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dronestack
)

### Generating Services
_generate_srv_py(dronestack
  "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dronestack
)

### Generating Module File
_generate_module_py(dronestack
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dronestack
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dronestack_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dronestack_generate_messages dronestack_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(dronestack_generate_messages_py _dronestack_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cpsl/catkin_ws/src/dronestack/srv/waypoint_nav.srv" NAME_WE)
add_dependencies(dronestack_generate_messages_py _dronestack_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dronestack_genpy)
add_dependencies(dronestack_genpy dronestack_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dronestack_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dronestack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dronestack
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dronestack_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dronestack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dronestack
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dronestack_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dronestack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dronestack
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dronestack_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dronestack)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dronestack
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dronestack_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dronestack)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dronestack\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dronestack
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dronestack_generate_messages_py std_msgs_generate_messages_py)
endif()

# Install script for directory: /home/cpsl/catkin_ws/src/mavros/mavros_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cpsl/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/msg" TYPE FILE FILES
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ADSBVehicle.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ActuatorControl.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Altitude.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/AttitudeTarget.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/BatteryStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/CamIMUStamp.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/CellularStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/CameraImageCaptured.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/CommandCode.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/CompanionProcessStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/OnboardComputerStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/DebugValue.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ESCInfo.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ESCInfoItem.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ESCStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ESCStatusItem.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ESCTelemetry.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ESCTelemetryItem.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/EstimatorStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ExtendedState.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/FileEntry.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/GlobalPositionTarget.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/GPSINPUT.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/GPSRAW.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/GPSRTK.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/HilActuatorControls.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/HilControls.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/HilGPS.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/HilSensor.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/HilStateQuaternion.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/HomePosition.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/LandingTarget.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/LogData.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/LogEntry.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/MagnetometerReporter.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ManualControl.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Mavlink.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/MountControl.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/NavControllerOutput.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/OpticalFlowRad.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/OverrideRCIn.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Param.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/ParamValue.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/PlayTuneV2.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/PositionTarget.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/RCIn.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/RCOut.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/RTCM.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/RadioStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/RTKBaseline.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/State.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/StatusText.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/SysStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/TerrainReport.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Thrust.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/TimesyncStatus.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Trajectory.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Tunnel.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/VFR_HUD.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/VehicleInfo.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Vibration.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/Waypoint.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/WaypointList.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/WaypointReached.msg"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/msg/WheelOdomStamped.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/srv" TYPE FILE FILES
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandAck.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandBool.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandHome.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandInt.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandLong.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandTOL.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandTriggerControl.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandTriggerInterval.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/CommandVtolTransition.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileChecksum.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileClose.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileList.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileMakeDir.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileOpen.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileRead.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileRemove.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileRemoveDir.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileRename.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileTruncate.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/FileWrite.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestData.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestEnd.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/LogRequestList.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/MountConfigure.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/MessageInterval.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/ParamGet.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/ParamPull.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/ParamPush.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/ParamSet.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/SetMavFrame.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/SetMode.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/StreamRate.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/VehicleInfoGet.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/WaypointClear.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/WaypointPull.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/WaypointPush.srv"
    "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/srv/WaypointSetCurrent.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/home/cpsl/catkin_ws/build/mavros/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cpsl/catkin_ws/devel/include/mavros_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cpsl/catkin_ws/devel/share/roseus/ros/mavros_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cpsl/catkin_ws/devel/share/common-lisp/ros/mavros_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cpsl/catkin_ws/devel/share/gennodejs/ros/mavros_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/cpsl/catkin_ws/devel/lib/python3/dist-packages/mavros_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/cpsl/catkin_ws/devel/lib/python3/dist-packages/mavros_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cpsl/catkin_ws/build/mavros/mavros_msgs/catkin_generated/installspace/mavros_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES "/home/cpsl/catkin_ws/build/mavros/mavros_msgs/catkin_generated/installspace/mavros_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs/cmake" TYPE FILE FILES
    "/home/cpsl/catkin_ws/build/mavros/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig.cmake"
    "/home/cpsl/catkin_ws/build/mavros/mavros_msgs/catkin_generated/installspace/mavros_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mavros_msgs" TYPE FILE FILES "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mavros_msgs" TYPE DIRECTORY FILES "/home/cpsl/catkin_ws/src/mavros/mavros_msgs/include/mavros_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()


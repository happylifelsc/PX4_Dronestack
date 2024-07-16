# Install script for directory: /home/cpsl/PX4-Autopilot/platforms/nuttx/src/px4/stm/stm32h7

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/adc/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/board_critmon/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/board_hw_info/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/board_reset/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/dshot/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/hrt/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/led_pwm/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/io_pins/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/spi/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/tone_alarm/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/version/cmake_install.cmake")
  include("/home/cpsl/PX4-Autopilot/build/holybro_kakuteh7_default/platforms/nuttx/src/px4/stm/stm32h7/px4io_serial/cmake_install.cmake")

endif()


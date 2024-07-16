set(CMAKE_HOST_SYSTEM "Linux-5.15.0-113-generic")
set(CMAKE_HOST_SYSTEM_NAME "Linux")
set(CMAKE_HOST_SYSTEM_VERSION "5.15.0-113-generic")
set(CMAKE_HOST_SYSTEM_PROCESSOR "x86_64")

include("/home/cpsl/PX4-Autopilot/platforms/nuttx/cmake/Toolchain-arm-none-eabi.cmake")

set(CMAKE_SYSTEM "Generic-1")
set(CMAKE_SYSTEM_NAME "Generic")
set(CMAKE_SYSTEM_VERSION "1")
set(CMAKE_SYSTEM_PROCESSOR "cortex-m7")

set(CMAKE_CROSSCOMPILING "TRUE")

set(CMAKE_SYSTEM_LOADED 1)

cmake_minimum_required(VERSION 3.5)
project(fast_decompress)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# Eigen3
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

include(FindTurboJPEG.cmake)
if(NOT TurboJPEG_FOUND)
    message(FATAL_ERROR "libjpeg-turbo not found")
endif()

# ===================================================
# Executable
set(TARGET_NAME fast_decompress_node)
ament_auto_add_executable(${TARGET_NAME} src/fast_decompress.cpp)
target_include_directories(${TARGET_NAME} PUBLIC include lib ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS} ${TurboJPEG_LIBRARIES})

# ===================================================
ament_auto_package()

cmake_minimum_required(VERSION 3.5)
project(turbo_decoder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
# OpenCV
find_package(OpenCV REQUIRED)

include(FindTurboJPEG.cmake)
if(NOT TurboJPEG_FOUND)
  message(FATAL_ERROR "libjpeg-turbo not found")
endif()

# ===================================================
# Executable
ament_auto_add_library(${PROJECT_NAME} lib/turbo_decoder.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${TurboJPEG_LIBRARIES})

# ===================================================
# Executable
set(TARGET_NAME decoder_node)
ament_auto_add_executable(${TARGET_NAME} src/decoder_node.cpp)
target_include_directories(${TARGET_NAME} PUBLIC include)
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})

set(TARGET_NAME demo_node)
ament_auto_add_executable(${TARGET_NAME} src/demo_node.cpp)
target_include_directories(${TARGET_NAME} PUBLIC include)
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})

# ===================================================
ament_auto_package(INSTALL_TO_SHARE launch)

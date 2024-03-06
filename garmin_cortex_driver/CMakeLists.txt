cmake_minimum_required(VERSION 3.5)

project (garmin_cortex_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  garmin_cortex_msgs
  roscpp
)

find_package(GarminCortex REQUIRED)
find_package(Threads)

catkin_package(
  CATKIN_DEPENDS garmin_cortex_msgs
)

add_executable(garmin_cortex src/cortex.cpp src/garmin_cortex.cpp)

target_include_directories(garmin_cortex PUBLIC include ${GarminCortex_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
target_link_libraries(garmin_cortex ${GarminCortex_LIBRARY} Threads::Threads ${catkin_LIBRARIES} flite_cmu_us_kal flite_usenglish flite_cmulex flite)
add_dependencies(garmin_cortex ${catkin_EXPORTED_TARGETS})



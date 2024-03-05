cmake_minimum_required(VERSION 3.5)

project (garmin_cortex)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  message_generation
  roscpp
)

find_package(GarminCortex REQUIRED)
find_package(Threads)

add_service_files(
  FILES
  SetVHFCallingChannelID.srv
  TransmitText.srv
)

generate_messages(
)

catkin_package(
  CATKIN_DEPENDS audio_common_msgs message_runtime
)

add_executable(garmin_cortex src/garmin_cortex.cpp)

target_include_directories(garmin_cortex PUBLIC ${GarminCortex_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
target_link_libraries(garmin_cortex ${GarminCortex_LIBRARY} Threads::Threads ${catkin_LIBRARIES} flite_cmu_us_kal flite_usenglish flite_cmulex flite)
add_dependencies(garmin_cortex ${catkin_EXPORTED_TARGETS})



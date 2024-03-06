cmake_minimum_required(VERSION 3.5)

project (garmin_cortex_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ChannelPreset.msg
  VHFChannel.msg
  VHFChannelSet.msg
  VHFReceiverControl.msg
  VHFRegion.msg
  VHFSettingsControl.msg
)

add_service_files(
  FILES
  SetVHFCallingChannelID.srv
)

generate_messages(
  DEPENDENCIES
  audio_common_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS audio_common_msgs message_runtime std_msgs
)


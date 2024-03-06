#ifndef GARMIN_CORTEX_GARMIN_CORTEX_H
#define GARMIN_CORTEX_GARMIN_CORTEX_H

#include <ros/ros.h>
#include <VHFoIP.h>
#include "garmin_cortex_msgs/SetVHFCallingChannelID.h"
#include "marine_radio_msgs/TransmitAudio.h"

namespace garmin_cortex
{

class Cortex
{
public:
  Cortex();
  ~Cortex();

private:

  void payload_received_callback(const std::string & subject, const std::string& payload);
  void transmit_audio_done_callback(bool success);
  void channel_catalog_changed_callback(bool success);
  void channel_list_callback(const vesper::vader::IVaderObject & object);
  void audio_received_callback(int itu_channel, const vesper::voice::audio_payload_container_t & audio_payload);
  bool setVHFCallingChannelIDService(garmin_cortex_msgs::SetVHFCallingChannelID::Request& req, garmin_cortex_msgs::SetVHFCallingChannelID::Response& resp);
  bool transmitAudioService(marine_radio_msgs::TransmitAudio::Request& req, marine_radio_msgs::TransmitAudio::Response& resp);


  std::shared_ptr<asio::io_context> io_context_;
  std::thread io_context_thread_;

  std::shared_ptr<vesper::VHFoIP> cortex_;

  ros::ServiceServer set_vhf_calling_channel_id_service_;
  ros::ServiceServer transmit_audio_service_;

  ros::Publisher audio_data_publisher_;
  ros::Publisher audio_info_publisher_;
  ros::Time last_audio_info_publish_time_;

  ros::Publisher channels_publisher_;
  ros::Publisher receiver_control_publisher_;
  ros::Publisher settings_control_publisher_;

  std::string frame_id_ = "cortex";

};

} // namespace garmin_cortex

#endif
#include "garmin_cortex/cortex.h"

#include "marine_radio_msgs/AudioDataStamped.h"
#include "audio_common_msgs/AudioInfo.h"
#include <Vader/objects/VHFRegionInfo.h>
#include "garmin_cortex_msgs/VHFChannelSet.h"
#include "garmin_cortex_msgs/VHFReceiverControl.h"
#include "garmin_cortex_msgs/VHFSettingsControl.h"
#include "common/json.h"


namespace garmin_cortex
{

Cortex::Cortex()
{
  ros::NodeHandle nh("~");

  std::string hub_ip("10.1.1.1");
  hub_ip = nh.param("ip_address", hub_ip);
  
  int port = nh.param("port", 8000);
  frame_id_ = nh.param("frame_id", frame_id_);

  audio_info_publisher_ = nh.advertise<audio_common_msgs::AudioInfo>("audio_info", 1, true);
  audio_data_publisher_ = nh.advertise<marine_radio_msgs::AudioDataStamped>("audio", 10);
  channels_publisher_ = nh.advertise<garmin_cortex_msgs::VHFChannelSet>("channels", 1, true);
  receiver_control_publisher_ = nh.advertise<garmin_cortex_msgs::VHFReceiverControl>("receiver_controls", 1, true);
  settings_control_publisher_ = nh.advertise<garmin_cortex_msgs::VHFSettingsControl>("settings_controls", 1, true);

  io_context_ = std::make_shared<asio::io_context>();

  ROS_INFO_STREAM("Connecting to hub on " << hub_ip);
  cortex_ = std::make_shared<vesper::VHFoIP>(io_context_, hub_ip, port);
  cortex_->start();

  io_context_thread_ = std::thread([this](){
    this->io_context_->run();
  });

  std::string country_code = "USA";
  country_code = nh.param("region", country_code);

  vesper::vader::VHFAvailableRegions country_id = vesper::vader::VHFAvailableRegions::INTERNATIONAL;
  if(country_code.size() == 3)
  {
    if(country_code == "USA")
      country_id = vesper::vader::VHFAvailableRegions::US;
    if(country_code == "CAN")
      country_id = vesper::vader::VHFAvailableRegions::CANADA;
  }

  std::string atis_requirement = "none";
  atis_requirement = nh.param("atis", atis_requirement);
  
  vesper::vader::VHFATISRequirement atis = vesper::vader::VHFATISRequirement::NONE;
  if(atis_requirement == "coastal")
    atis = vesper::vader::VHFATISRequirement::COASTAL;
  if(atis_requirement == "inland")
    atis = vesper::vader::VHFATISRequirement::INLAND;

  cortex_->vhf_change_catalog_region(country_id, country_code, atis, std::bind(&Cortex::channel_catalog_changed_callback, this, std::placeholders::_1));

  cortex_->add_vader_payload_received_callback(std::bind( &Cortex::payload_received_callback, this, std::placeholders::_1, std::placeholders::_2));

  cortex_->add_receive_audio_payload_callback(std::bind(&Cortex::audio_received_callback, this, std::placeholders::_1, std::placeholders::_2));

  set_vhf_calling_channel_id_service_ = nh.advertiseService("set_vhf_calling_channel_id", &Cortex::setVHFCallingChannelIDService, this);

  transmit_audio_service_ = nh.advertiseService("transmit_audio", &Cortex::transmitAudioService, this);

  cortex_->add_transmit_audio_payload_callback(std::bind(&Cortex::transmit_audio_done_callback, this, std::placeholders::_1));


}

Cortex::~Cortex()
{
  io_context_->stop();

while (!io_context_thread_.joinable())
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
io_context_thread_.join();

}


void Cortex::payload_received_callback(const std::string & subject, const std::string& payload)
{
  auto pj =  vesper::common::JSON::parse(payload);
  if(subject == "ChannelIdentifier")
  {

  }
  else if(subject == "VHFReceiverControl")
  {
    garmin_cortex_msgs::VHFReceiverControl vrc;
    if (pj.contains("receiverState"))
      vrc.receiver_state = pj["receiverState"];
    if (pj.contains("callingChannelId"))
      vrc.calling_channel_id = pj["callingChannelId"];
    if (pj.contains("callingChannelHighPower"))
      vrc.calling_channel_high_power = pj["callingChannelHighPower"];
    if (pj.contains("priorityChannelId"))
      vrc.priority_channel_id = pj["priorityChannelId"];
    if (pj.contains("squelchOn"))
      vrc.squelch_on = pj["squelchOn"];
    if (pj.contains("dualWatchDisabled"))
      vrc.dual_watch_disabled = pj["dualWatchDisabled"];
    if (pj.contains("dualWatchOffReason"))
      vrc.dual_watch_off_reason = pj["dualWatchOffReason"];
    receiver_control_publisher_.publish(vrc);
  }
  else if(subject == "VHFSettingsControl")
  {
    garmin_cortex_msgs::VHFSettingsControl vsc;
    if (pj.contains("region"))
    {
      vsc.region.region = pj["region"]["region"];
      vsc.region.atis = pj["region"]["atis"];
    }
    if (pj.contains("squelchBias"))
      vsc.squelch_bias = pj["squelchBias"];
    if (pj.contains("wxWatch"))
      vsc.wx_watch = pj["wxWatch"];
    if (pj.contains("scanning"))
      vsc.scanning = pj["scanning"];
    if (pj.contains("highPower"))
      vsc.high_power = pj["highPower"];
    if (pj.contains("presets"))
      for(const auto &ps: pj["presets"])
      {
        garmin_cortex_msgs::ChannelPreset cp;
        if (ps.contains("presetId"))
          cp.preset_id = ps["presetId"];
        if (ps.contains("channelId"))
          cp.channel_id = ps["channelId"];
        if (ps.contains("highPower"))
          cp.high_power = ps["highPower"];
        vsc.presets.push_back(cp);
      }
    settings_control_publisher_.publish(vsc);
  }
  else
    ROS_INFO_STREAM("Subject: " << subject << "\nPayload: " << payload);
}


void Cortex::transmit_audio_done_callback(bool success)
{
  if(!success)
    ROS_WARN("Audio transmit unsuccesful");
}

void Cortex::channel_catalog_changed_callback(bool success)
{
  if(!success)
    ROS_WARN("Setting channel region unsuccesful");
  cortex_->vhf_channels_in_current_region(std::bind(&Cortex::channel_list_callback, this, std::placeholders::_1));
}

void Cortex::channel_list_callback(const vesper::vader::IVaderObject & object)
{
  const vesper::vader::VHFRegionInfo* region_info = dynamic_cast<const vesper::vader::VHFRegionInfo*>(&object);
  if(region_info)
  {
    garmin_cortex_msgs::VHFChannelSet channels;
    for(const auto &channel: region_info->communication_channels)
    {
      garmin_cortex_msgs::VHFChannel channel_msg;
      channel_msg.id = channel.channel_id;
      channel_msg.number = channel.number;
      channel_msg.variant = channel.variant;
      channel_msg.description = channel.description;
      channel_msg.display = channel.display;
      channel_msg.channel_type = garmin_cortex_msgs::VHFChannel::CHANNEL_TYPE_COMMUNICATION;
      channels.channels.push_back(channel_msg);
    }
    for(const auto &channel: region_info->weather_channels)
    {
      garmin_cortex_msgs::VHFChannel channel_msg;
      channel_msg.id = channel.channel_id;
      channel_msg.number = channel.number;
      channel_msg.variant = channel.variant;
      channel_msg.description = channel.description;
      channel_msg.display = channel.display;
      channel_msg.channel_type = garmin_cortex_msgs::VHFChannel::CHANNEL_TYPE_WEATHER;
      channels.channels.push_back(channel_msg);
    }
    for(const auto &channel: region_info->safety_broadcast_channels)
    {
      garmin_cortex_msgs::VHFChannel channel_msg;
      channel_msg.id = channel.channel_id;
      channel_msg.number = channel.number;
      channel_msg.variant = channel.variant;
      channel_msg.description = channel.description;
      channel_msg.display = channel.display;
      channel_msg.channel_type = garmin_cortex_msgs::VHFChannel::CHANNEL_TYPE_BROADCAST;
      channels.channels.push_back(channel_msg);
    }
    channels_publisher_.publish(channels);
  }
  else
    ROS_WARN_STREAM("Failed to dynamic cast to VHFRegionInfo");

}

void Cortex::audio_received_callback(int itu_channel, const vesper::voice::audio_payload_container_t & audio_payload)
{
  auto now = ros::Time::now();
  if(last_audio_info_publish_time_.is_zero() || now - last_audio_info_publish_time_ > ros::Duration(1.0))
  {
    audio_common_msgs::AudioInfo info;
    info.channels = 1;
    info.sample_rate = vesper::voice::audio_sampling_hz;
    info.sample_format = "S16LE";
    info.coding_format = "PCM";
    info.bitrate = vesper::voice::audio_sampling_hz*sizeof(vesper::voice::audio_payload_t)*8;
    audio_info_publisher_.publish(info);
    last_audio_info_publish_time_ = now;
  }

  marine_radio_msgs::AudioDataStamped data;
  data.header.stamp = now;
  data.header.frame_id = frame_id_;
  data.audio.channel_id = itu_channel;
  data.audio.audio.data.resize(audio_payload.size() * sizeof(vesper::voice::audio_payload_t));
  memcpy(data.audio.audio.data.data(), audio_payload.data(), audio_payload.size()*sizeof(vesper::voice::audio_payload_t));
  audio_data_publisher_.publish(data);
}


bool Cortex::setVHFCallingChannelIDService(garmin_cortex_msgs::SetVHFCallingChannelID::Request& req, garmin_cortex_msgs::SetVHFCallingChannelID::Response& resp)
{
  cortex_->vhf_calling_channel_id(req.channel_id);
  return true;
}

bool Cortex::transmitAudioService(marine_radio_msgs::TransmitAudio::Request& req, marine_radio_msgs::TransmitAudio::Response& resp)
{
  vesper::voice::audio_payload_container_t data;
  for(size_t i = 0; i < req.message.audio_data.audio.data.size()/2; i++)
  {
    auto sample = reinterpret_cast<vesper::voice::audio_payload_t*>(req.message.audio_data.audio.data.data())[i];
    data.push_back(sample);
    data.push_back(sample);
  }

  cortex_->vhf_transmit(std::move(data));

  resp.success = true;

  return true;
}

} // namespace garmin_cortex


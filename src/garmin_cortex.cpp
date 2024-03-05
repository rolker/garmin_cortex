#include <ros/ros.h>
#include <VHFoIP.h>
#include <Vader/objects/VHFRegionInfo.h>
#include "garmin_cortex/SetVHFCallingChannelID.h"
#include "garmin_cortex/TransmitText.h"
#include "audio_common_msgs/AudioData.h"
#include "audio_common_msgs/AudioInfo.h"

#include "flite/flite.h"
extern "C"
{
	cst_voice* register_cmu_us_kal();
}

class Cortex
{
public:
  Cortex()
  {
    std::string hub_ip("10.1.1.1");
    ros::NodeHandle nh("~");
    hub_ip = nh.param("ip_address", hub_ip);
    
    int port = nh.param("port", 8000);

    audio_info_publisher_ = nh.advertise<audio_common_msgs::AudioInfo>("audio_info", 1, true);
    audio_data_publisher_ = nh.advertise<audio_common_msgs::AudioData>("audio", 10);

    io_context_ = std::make_shared<asio::io_context>();

    ROS_INFO_STREAM("Connecting to hub on " << hub_ip);
    cortex_ = std::make_shared<vesper::VHFoIP>(io_context_, hub_ip, port);
    cortex_->start();

    io_context_thread_ = std::thread([this](){
      this->io_context_->run();
    });

    cortex_->add_vader_payload_received_callback(std::bind( &Cortex::payload_received_callback, this, std::placeholders::_1, std::placeholders::_2));

    cortex_->add_receive_audio_payload_callback(std::bind(&Cortex::audio_received_callback, this, std::placeholders::_1, std::placeholders::_2));

    set_vhf_calling_channel_id_service_ = nh.advertiseService("set_vhf_calling_channel_id", &Cortex::setVHFCallingChannelIDService, this);

    cortex_->add_transmit_audio_payload_callback(std::bind(&Cortex::transmit_audio_done_callback, this, std::placeholders::_1));

    flite_init();
    voice_ = register_cmu_us_kal();
    transmit_text_service_ = nh.advertiseService("transmit_text", &Cortex::transmitTextService, this);

  }


  ~Cortex()
  {
    io_context_->stop();

  while (!io_context_thread_.joinable())
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  io_context_thread_.join();

  }


private:
  void payload_received_callback(const std::string & subject, const std::string& payload)
  {
    ROS_INFO_STREAM("Subject: " << subject << "\nPayload: " << payload);
  }

  void transmit_audio_done_callback(bool success)
  {
    if(!success)
      ROS_WARN("Audio transmit unsuccesful");
  }

  void audio_received_callback(int itu_channel, const vesper::voice::audio_payload_container_t & audio_payload)
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

    audio_common_msgs::AudioData data;
    data.data.resize(audio_payload.size() * sizeof(vesper::voice::audio_payload_t));
    memcpy(data.data.data(), audio_payload.data(), audio_payload.size()*sizeof(vesper::voice::audio_payload_t));
    audio_data_publisher_.publish(data);
  }

  bool setVHFCallingChannelIDService(garmin_cortex::SetVHFCallingChannelID::Request& req, garmin_cortex::SetVHFCallingChannelID::Response& resp)
  {
    cortex_->vhf_calling_channel_id(req.channel_id);
    return true;
  }


  bool transmitTextService(garmin_cortex::TransmitText::Request& req, garmin_cortex::TransmitText::Response& resp)
  {
    auto wave = flite_text_to_wave(req.message.c_str(), voice_);
    resp.transmit_duration = wave->num_samples/float(wave->sample_rate);
    
    vesper::voice::audio_payload_container_t data;
    for(int i = 0; i < wave->num_samples; i++)
    {
      data.push_back(wave->samples[i]);
      data.push_back(wave->samples[i]);
    }

    cortex_->vhf_transmit(std::move(data));

    delete_wave(wave);

    return true;
  }

  std::shared_ptr<asio::io_context> io_context_;
  std::thread io_context_thread_;

  std::shared_ptr<vesper::VHFoIP> cortex_;

  ros::ServiceServer set_vhf_calling_channel_id_service_;
  ros::ServiceServer transmit_text_service_;

  ros::Publisher audio_data_publisher_;
  ros::Publisher audio_info_publisher_;
  ros::Time last_audio_info_publish_time_;

  cst_voice* voice_ = nullptr;
};





int main(int argc, char** argv)
{
  ros::init(argc, argv, "garmin_cortex");

  Cortex cortex;

  ros::spin();


  return 0;
}


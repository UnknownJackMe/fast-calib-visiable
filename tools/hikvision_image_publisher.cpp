#include "MvCameraControl.h"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void CheckMvRet(const std::string &call, int ret) {
  if (ret != MV_OK) {
    throw std::runtime_error(call + " failed with code " + std::to_string(ret));
  }
}

std::string DeviceSerial(const MV_CC_DEVICE_INFO *info) {
  if (info == nullptr) {
    return "";
  }
  if (info->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  return "";
}

std::string DeviceModel(const MV_CC_DEVICE_INFO *info) {
  if (info == nullptr) {
    return "";
  }
  if (info->nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chModelName);
  }
  if (info->nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chModelName);
  }
  return "";
}

class HikvisionImagePublisher : public rclcpp::Node {
 public:
  HikvisionImagePublisher() : Node("hikvision_image_publisher") {
    const auto serial = declare_parameter<std::string>("serial", "DA3217436");
    const auto exposure_us = declare_parameter<double>("exposure_us", 30000.0);
    const auto gain = declare_parameter<double>("gain", 8.0);
    const auto frame_id = declare_parameter<std::string>("frame_id", "hikvision_camera");
    const auto topic = declare_parameter<std::string>("topic", "/camera/image_raw");
    const auto publish_rate = declare_parameter<double>("publish_rate", 1.0);

    if (publish_rate <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
    }

    frame_id_ = frame_id;
    publisher_ = create_publisher<sensor_msgs::msg::Image>(topic, rclcpp::SensorDataQoS());
    OpenCamera(serial, static_cast<float>(exposure_us), static_cast<float>(gain));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&HikvisionImagePublisher::PublishFrame, this));

    RCLCPP_INFO(get_logger(),
                "Publishing Hikvision %s serial=%s on %s at %.2f Hz, exposure=%.0f us, gain=%.1f",
                model_.c_str(), serial.c_str(), topic.c_str(), publish_rate, exposure_us, gain);
  }

  ~HikvisionImagePublisher() override { CloseCamera(); }

 private:
  void OpenCamera(const std::string &expected_serial, float exposure_us, float gain) {
    MV_CC_DEVICE_INFO_LIST devices;
    std::memset(&devices, 0, sizeof(devices));
    CheckMvRet("MV_CC_EnumDevices", MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices));

    const MV_CC_DEVICE_INFO *selected = nullptr;
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
      const auto *info = devices.pDeviceInfo[i];
      RCLCPP_INFO(get_logger(), "Camera[%u] model=%s serial=%s", i,
                  DeviceModel(info).c_str(), DeviceSerial(info).c_str());
      if (DeviceSerial(info) == expected_serial) {
        selected = info;
        model_ = DeviceModel(info);
      }
    }
    if (selected == nullptr) {
      throw std::runtime_error("Hikvision camera serial " + expected_serial + " not found");
    }

    CheckMvRet("MV_CC_CreateHandle", MV_CC_CreateHandle(&handle_, selected));
    try {
      CheckMvRet("MV_CC_OpenDevice", MV_CC_OpenDevice(handle_));
      CheckMvRet("AcquisitionMode", MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous"));
      CheckMvRet("TriggerMode", MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off"));
      CheckMvRet("PixelFormat", MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8));
      CheckMvRet("ExposureAuto", MV_CC_SetExposureAutoMode(handle_, 0));
      CheckMvRet("ExposureTime", MV_CC_SetExposureTime(handle_, exposure_us));
      CheckMvRet("GainAuto", MV_CC_SetEnumValue(handle_, "GainAuto", 0));
      CheckMvRet("Gain", MV_CC_SetGain(handle_, gain));
      CheckMvRet("MV_CC_StartGrabbing", MV_CC_StartGrabbing(handle_));
      grabbing_ = true;

      MVCC_INTVALUE_EX payload;
      std::memset(&payload, 0, sizeof(payload));
      CheckMvRet("PayloadSize", MV_CC_GetIntValueEx(handle_, "PayloadSize", &payload));
      buffer_.resize(static_cast<size_t>(payload.nCurValue));
    } catch (...) {
      CloseCamera();
      throw;
    }
  }

  void CloseCamera() {
    if (handle_ == nullptr) {
      return;
    }
    if (grabbing_) {
      MV_CC_StopGrabbing(handle_);
      grabbing_ = false;
    }
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }

  void PublishFrame() {
    MV_FRAME_OUT_INFO_EX info;
    std::memset(&info, 0, sizeof(info));
    const int ret = MV_CC_GetOneFrameTimeout(
        handle_, buffer_.data(), static_cast<unsigned int>(buffer_.size()), &info, 1000);
    if (ret != MV_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "MV_CC_GetOneFrameTimeout failed: %d", ret);
      return;
    }

    cv::Mat bayer(static_cast<int>(info.nHeight), static_cast<int>(info.nWidth),
                  CV_8UC1, buffer_.data());
    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);

    sensor_msgs::msg::Image message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.height = info.nHeight;
    message.width = info.nWidth;
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = info.nWidth * 3;
    message.data.assign(bgr.data, bgr.data + bgr.total() * bgr.elemSize());
    publisher_->publish(std::move(message));
  }

  void *handle_ = nullptr;
  bool grabbing_ = false;
  std::string model_;
  std::string frame_id_;
  std::vector<unsigned char> buffer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<HikvisionImagePublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

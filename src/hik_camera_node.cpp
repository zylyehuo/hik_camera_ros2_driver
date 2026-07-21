#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

namespace hik_camera_ros2_driver
{
class HikCameraRos2DriverNode : public rclcpp::Node
{
public:
  explicit HikCameraRos2DriverNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera_ros2_driver", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraRos2DriverNode");

    declareParameters();
    initializeRosInterfaces();

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(
        &HikCameraRos2DriverNode::dynamicParametersCallback, this,
        std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraRos2DriverNode::captureLoop, this);
  }

  ~HikCameraRos2DriverNode() override
  {
    running_.store(false);

    // Stop grabbing before join so a blocking GetImageBuffer call can return.
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_handle_ != nullptr) {
        const int ret = MV_CC_StopGrabbing(camera_handle_);
        if (ret != MV_OK && !isIgnorableStopError(ret)) {
          RCLCPP_WARN(
            this->get_logger(),
            "Stop grabbing during shutdown failed: %s",
            sdkErrorText(ret).c_str());
        }
      }
    }

    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }

    closeCamera();
    RCLCPP_INFO(this->get_logger(), "HikCameraRos2DriverNode destroyed");
  }

private:
  struct GigENetworkStats
  {
    int64_t received_bytes = 0;
    int64_t lost_packets = 0;
    unsigned int lost_frames = 0;
    int64_t requested_resend_packets = 0;
    int64_t resent_packets = 0;
  };

  static bool errorEquals(const int ret, const unsigned int expected)
  {
    return static_cast<unsigned int>(ret) == expected;
  }

  static bool isIgnorableStopError(const int ret)
  {
    return errorEquals(ret, static_cast<unsigned int>(MV_E_CALLORDER)) ||
           errorEquals(ret, static_cast<unsigned int>(MV_E_HANDLE));
  }

  static bool isGigETransportError(const int ret)
  {
    return errorEquals(ret, static_cast<unsigned int>(MV_E_NETER)) ||
           errorEquals(ret, static_cast<unsigned int>(MV_E_PACKET)) ||
           errorEquals(ret, static_cast<unsigned int>(MV_E_BUSY)) ||
           errorEquals(ret, static_cast<unsigned int>(MV_E_GC_TIMEOUT)) ||
           errorEquals(ret, static_cast<unsigned int>(MV_E_IP_CONFLICT));
  }

  static const char * sdkErrorName(const int ret)
  {
    switch (static_cast<unsigned int>(ret)) {
      case MV_OK:
        return "MV_OK";
      case MV_E_HANDLE:
        return "MV_E_HANDLE";
      case MV_E_SUPPORT:
        return "MV_E_SUPPORT";
      case MV_E_BUFOVER:
        return "MV_E_BUFOVER";
      case MV_E_CALLORDER:
        return "MV_E_CALLORDER";
      case MV_E_PARAMETER:
        return "MV_E_PARAMETER";
      case MV_E_RESOURCE:
        return "MV_E_RESOURCE";
      case MV_E_NODATA:
        return "MV_E_NODATA";
      case MV_E_PRECONDITION:
        return "MV_E_PRECONDITION";
      case MV_E_NOENOUGH_BUF:
        return "MV_E_NOENOUGH_BUF";
      case MV_E_GC_ACCESS:
        return "MV_E_GC_ACCESS";
      case MV_E_GC_TIMEOUT:
        return "MV_E_GC_TIMEOUT";
      case MV_E_NOT_IMPLEMENTED:
        return "MV_E_NOT_IMPLEMENTED";
      case MV_E_INVALID_ADDRESS:
        return "MV_E_INVALID_ADDRESS";
      case MV_E_WRITE_PROTECT:
        return "MV_E_WRITE_PROTECT";
      case MV_E_ACCESS_DENIED:
        return "MV_E_ACCESS_DENIED";
      case MV_E_BUSY:
        return "MV_E_BUSY";
      case MV_E_PACKET:
        return "MV_E_PACKET";
      case MV_E_NETER:
        return "MV_E_NETER";
      case MV_E_IP_CONFLICT:
        return "MV_E_IP_CONFLICT";
      default:
        return "MV_E_UNKNOWN";
    }
  }

  static std::string sdkErrorText(const int ret)
  {
    std::ostringstream stream;
    stream << sdkErrorName(ret) << " (0x";
    stream.setf(std::ios::hex, std::ios::basefield);
    stream.setf(std::ios::uppercase);
    stream.width(8);
    stream.fill('0');
    stream << static_cast<unsigned int>(ret) << ")";
    return stream.str();
  }

  void declareParameters()
  {
    camera_sn_ = this->declare_parameter<std::string>("camera_sn", "DA4714681");
    pixel_format_ = this->declare_parameter<std::string>("pixel_format", "BayerRG8");

    acquisition_frame_rate_ =
      this->declare_parameter<double>("acquisition_frame_rate", 10.0);
    exposure_time_ = this->declare_parameter<int>("exposure_time", 120);
    gain_ = this->declare_parameter<double>("gain", 16.9);

    image_width_ = this->declare_parameter<int>("image_width", 1280);
    image_height_ = this->declare_parameter<int>("image_height", 720);
    image_node_num_ = this->declare_parameter<int>("image_node_num", 8);

    frame_timeout_ms_ = this->declare_parameter<int>("frame_timeout_ms", 1000);
    startup_frame_timeout_ms_ =
      this->declare_parameter<int>("startup_frame_timeout_ms", 2000);
    timeout_recovery_threshold_ =
      this->declare_parameter<int>("timeout_recovery_threshold", 3);
    reconnect_interval_ms_ =
      this->declare_parameter<int>("reconnect_interval_ms", 1000);
    open_settle_ms_ = this->declare_parameter<int>("open_settle_ms", 300);
    enable_auto_reconnect_ =
      this->declare_parameter<bool>("enable_auto_reconnect", true);

    reuse_camera_profile_on_reconnect_ =
      this->declare_parameter<bool>("reuse_camera_profile_on_reconnect", true);
    set_gige_packet_size_on_connect_ =
      this->declare_parameter<bool>("set_gige_packet_size_on_connect", true);
    gige_packet_size_ =
      this->declare_parameter<int>("gige_packet_size", 1500);

    gige_packet_delay_ =
      this->declare_parameter<int>("gige_packet_delay", 0);
    gige_enable_resend_ =
      this->declare_parameter<bool>("gige_enable_resend", true);
    gige_max_resend_percent_ =
      this->declare_parameter<int>("gige_max_resend_percent", 20);
    gige_resend_timeout_ms_ =
      this->declare_parameter<int>("gige_resend_timeout_ms", 50);
    gige_resend_retry_times_ =
      this->declare_parameter<int>("gige_resend_retry_times", 5);
    gige_resend_interval_ms_ =
      this->declare_parameter<int>("gige_resend_interval_ms", 10);
    packet_loss_summary_interval_sec_ =
      this->declare_parameter<int>("packet_loss_summary_interval_sec", 5);

    use_sensor_data_qos_ =
      this->declare_parameter<bool>("use_sensor_data_qos", true);
    camera_name_ = this->declare_parameter<std::string>("camera_name", "camera");
    frame_id_ = this->declare_parameter<std::string>(
      "frame_id", camera_name_ + "_optical_frame");
    camera_topic_ = this->declare_parameter<std::string>(
      "camera_topic", camera_name_ + "/image");
    camera_info_url_ = this->declare_parameter<std::string>(
      "camera_info_url", "package://hik_camera_ros2_driver/config/camera_info.yaml");

    image_width_ = std::max(image_width_, 0);
    image_height_ = std::max(image_height_, 0);
    image_node_num_ = std::max(1, std::min(image_node_num_, 30));
    frame_timeout_ms_ = std::max(frame_timeout_ms_, 1);
    startup_frame_timeout_ms_ = std::max(startup_frame_timeout_ms_, 100);
    timeout_recovery_threshold_ = std::max(timeout_recovery_threshold_, 1);
    reconnect_interval_ms_ = std::max(reconnect_interval_ms_, 100);
    open_settle_ms_ = std::max(open_settle_ms_, 0);
    gige_packet_size_ = std::max(gige_packet_size_, 576);
    gige_packet_delay_ = std::max(gige_packet_delay_, 0);
    gige_max_resend_percent_ =
      std::max(1, std::min(gige_max_resend_percent_, 100));
    gige_resend_timeout_ms_ = std::max(gige_resend_timeout_ms_, 1);
    gige_resend_retry_times_ = std::max(gige_resend_retry_times_, 1);
    gige_resend_interval_ms_ = std::max(gige_resend_interval_ms_, 1);
    packet_loss_summary_interval_sec_ =
      std::max(1, std::min(packet_loss_summary_interval_sec_, 60));
  }

  void initializeRosInterfaces()
  {
    const auto qos =
      use_sensor_data_qos_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, camera_topic_, qos);

    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    if (camera_info_manager_->validateURL(camera_info_url_)) {
      if (!camera_info_manager_->loadCameraInfo(camera_info_url_)) {
        RCLCPP_WARN(
          this->get_logger(), "Failed to load camera info from: %s",
          camera_info_url_.c_str());
      }
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Invalid camera info URL: %s", camera_info_url_.c_str());
    }

    image_msg_.header.frame_id = frame_id_;
    image_msg_.encoding = "rgb8";
    image_msg_.is_bigendian = false;
  }

  std::string serialNumberOf(const MV_CC_DEVICE_INFO * device_info) const
  {
    if (device_info == nullptr) {
      return "";
    }

    if (device_info->nTLayerType == MV_GIGE_DEVICE) {
      return reinterpret_cast<const char *>(
        device_info->SpecialInfo.stGigEInfo.chSerialNumber);
    }

    if (device_info->nTLayerType == MV_USB_DEVICE) {
      return reinterpret_cast<const char *>(
        device_info->SpecialInfo.stUsb3VInfo.chSerialNumber);
    }

    return "";
  }

  const char * transportName(const unsigned int transport_type) const
  {
    if (transport_type == MV_GIGE_DEVICE) {
      return "GigE";
    }
    if (transport_type == MV_USB_DEVICE) {
      return "USB3";
    }
    return "Unknown";
  }

  bool checkSdkResult(
    const int ret, const std::string & operation, const bool required)
  {
    if (ret == MV_OK) {
      return true;
    }

    if (isGigETransportError(ret)) {
      RCLCPP_WARN(
        this->get_logger(),
        "GigE control channel failed during '%s': %s; aborting this setup attempt",
        operation.c_str(), sdkErrorText(ret).c_str());
      return false;
    }

    if (required) {
      RCLCPP_ERROR(
        this->get_logger(), "Required camera operation '%s' failed: %s",
        operation.c_str(), sdkErrorText(ret).c_str());
      return false;
    }

    RCLCPP_WARN(
      this->get_logger(), "Optional camera operation '%s' was skipped: %s",
      operation.c_str(), sdkErrorText(ret).c_str());
    return true;
  }

  bool setEnumString(
    void * handle, const char * key, const std::string & value,
    const bool required)
  {
    const int ret = MV_CC_SetEnumValueByString(handle, key, value.c_str());
    return checkSdkResult(
      ret, std::string("set ") + key + "=" + value, required);
  }

  bool setIntValue(
    void * handle, const char * key, const int64_t value, const bool required)
  {
    const int ret = MV_CC_SetIntValue(
      handle, key, static_cast<unsigned int>(value));
    return checkSdkResult(
      ret, std::string("set ") + key + "=" + std::to_string(value), required);
  }

  bool setFloatValue(
    void * handle, const char * key, const double value, const bool required)
  {
    const int ret = MV_CC_SetFloatValue(handle, key, static_cast<float>(value));
    std::ostringstream operation;
    operation << "set " << key << "=" << value;
    return checkSdkResult(ret, operation.str(), required);
  }

  bool setBoolValue(
    void * handle, const char * key, const bool value, const bool required)
  {
    const int ret = MV_CC_SetBoolValue(handle, key, value);
    return checkSdkResult(
      ret, std::string("set ") + key + "=" + (value ? "true" : "false"),
      required);
  }

  static unsigned int clampToIntegerNode(
    const int requested, const MVCC_INTVALUE & value)
  {
    const unsigned int lower = value.nMin;
    const unsigned int upper = std::max(value.nMin, value.nMax);
    unsigned int result = static_cast<unsigned int>(std::max(requested, 0));
    result = std::max(lower, std::min(result, upper));

    if (value.nInc > 1U && result > lower) {
      result = lower + ((result - lower) / value.nInc) * value.nInc;
    }
    return result;
  }

  bool configureGigEPacketSize(void * handle)
  {
    if (!set_gige_packet_size_on_connect_) {
      RCLCPP_WARN(
        this->get_logger(),
        "GigE packet-size enforcement is disabled; current network MTU must "
        "already match the camera packet size");
      return true;
    }

    MVCC_INTVALUE packet_info{};
    int ret = MV_GIGE_GetGevSCPSPacketSize(handle, &packet_info);
    if (!checkSdkResult(ret, "query GigE packet size", true)) {
      return false;
    }

    const unsigned int desired_packet_size =
      clampToIntegerNode(gige_packet_size_, packet_info);

    if (packet_info.nCurValue != desired_packet_size) {
      RCLCPP_INFO(
        this->get_logger(),
        "GigE packet size adjustment: camera=%u, setting=%u to match the "
        "standard 1500-byte Ethernet MTU",
        packet_info.nCurValue, desired_packet_size);

      ret = MV_GIGE_SetGevSCPSPacketSize(handle, desired_packet_size);
      if (!checkSdkResult(ret, "set GigE packet size", true)) {
        return false;
      }
    }

    MVCC_INTVALUE verified_info{};
    ret = MV_GIGE_GetGevSCPSPacketSize(handle, &verified_info);
    if (!checkSdkResult(ret, "verify GigE packet size", true)) {
      return false;
    }

    if (verified_info.nCurValue != desired_packet_size) {
      RCLCPP_ERROR(
        this->get_logger(),
        "GigE packet-size verification failed: requested=%u, actual=%u",
        desired_packet_size, verified_info.nCurValue);
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "GigE packet size configured: current=%u, requested=%d, range=%u..%u, "
      "increment=%u",
      verified_info.nCurValue, gige_packet_size_, verified_info.nMin,
      verified_info.nMax, verified_info.nInc);
    return true;
  }

  void configureGigEResend(void * handle)
  {
    const unsigned int enable = gige_enable_resend_ ? 1U : 0U;
    int ret = MV_GIGE_SetResend(
      handle, enable, static_cast<unsigned int>(gige_max_resend_percent_),
      static_cast<unsigned int>(gige_resend_timeout_ms_));
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to configure GigE packet resend: %s; continuing",
        sdkErrorText(ret).c_str());
      return;
    }

    if (!gige_enable_resend_) {
      RCLCPP_INFO(this->get_logger(), "GigE packet resend disabled");
      return;
    }

    ret = MV_GIGE_SetResendMaxRetryTimes(
      handle, static_cast<unsigned int>(gige_resend_retry_times_));
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set GigE resend retry count: %s; continuing",
        sdkErrorText(ret).c_str());
    }

    ret = MV_GIGE_SetResendTimeInterval(
      handle, static_cast<unsigned int>(gige_resend_interval_ms_));
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set GigE resend interval: %s; continuing",
        sdkErrorText(ret).c_str());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "GigE resend enabled: max_percent=%d, timeout=%d, retry_times=%d, "
      "retry_interval=%d",
      gige_max_resend_percent_, gige_resend_timeout_ms_,
      gige_resend_retry_times_, gige_resend_interval_ms_);
  }

  void configureGigEPacketDelay(void * handle)
  {
    MVCC_INTVALUE delay_info{};
    const int get_delay_ret = MV_GIGE_GetGevSCPD(handle, &delay_info);
    if (get_delay_ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to query GigE packet delay: %s; continuing",
        sdkErrorText(get_delay_ret).c_str());
      return;
    }

    const unsigned int desired_delay =
      clampToIntegerNode(gige_packet_delay_, delay_info);
    if (delay_info.nCurValue != desired_delay) {
      const int set_delay_ret = MV_GIGE_SetGevSCPD(handle, desired_delay);
      if (set_delay_ret != MV_OK) {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to set GigE packet delay=%u: %s; keeping current=%u",
          desired_delay, sdkErrorText(set_delay_ret).c_str(),
          delay_info.nCurValue);
        return;
      }
    }

    MVCC_INTVALUE packet_info{};
    const int packet_ret = MV_GIGE_GetGevSCPSPacketSize(handle, &packet_info);
    if (packet_ret == MV_OK) {
      RCLCPP_INFO(
        this->get_logger(),
        "GigE stream pacing configured: packet_size=%u, packet_delay=%u "
        "(requested=%d, range=%u..%u, increment=%u)",
        packet_info.nCurValue, desired_delay, gige_packet_delay_,
        delay_info.nMin, delay_info.nMax, delay_info.nInc);
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "GigE stream pacing configured: packet_delay=%u (packet-size query: %s)",
        desired_delay, sdkErrorText(packet_ret).c_str());
    }
  }

  int readGigENetworkStats(void * handle, GigENetworkStats & stats)
  {
    MV_MATCH_INFO_NET_DETECT net_info{};
    MV_ALL_MATCH_INFO all_info{};
    all_info.nType = MV_MATCH_TYPE_NET_DETECT;
    all_info.pInfo = &net_info;
    all_info.nInfoSize = sizeof(net_info);

    const int ret = MV_CC_GetAllMatchInfo(handle, &all_info);
    if (ret == MV_OK) {
      stats.received_bytes = net_info.nReviceDataSize;
      stats.lost_packets = net_info.nLostPacketCount;
      stats.lost_frames = net_info.nLostFrameCount;
      stats.requested_resend_packets = net_info.nRequestResendPacketCount;
      stats.resent_packets = net_info.nResendPacketCount;
    }
    return ret;
  }

  bool configureCamera(void * handle, const unsigned int transport_type)
  {
    if (!MV_CC_IsDeviceConnected(handle)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Camera disconnected immediately after opening; aborting this setup attempt");
      return false;
    }

    const bool apply_full_profile =
      !reuse_camera_profile_on_reconnect_ || !camera_profile_applied_;

    // This is an SDK-side cache setting and must be made before StartGrabbing.
    // Failure is non-fatal because the SDK default can still acquire frames.
    const int node_ret = MV_CC_SetImageNodeNum(
      handle, static_cast<unsigned int>(image_node_num_));
    if (node_ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to set SDK image-node count: %s; continuing",
        sdkErrorText(node_ret).c_str());
    }

    if (transport_type == MV_GIGE_DEVICE) {
      // GevSCPSPacketSize is a camera-side persistent node. The previous v6
      // run showed 5092 bytes while the host path accepts standard 1500-byte
      // Ethernet frames, causing every image payload packet to be discarded.
      // Enforce and verify 1500 bytes before starting the stream.
      if (!configureGigEPacketSize(handle)) {
        return false;
      }

      // Host/SDK stream settings must be applied to every newly created handle.
      configureGigEResend(handle);
      configureGigEPacketDelay(handle);
    }

    if (!apply_full_profile) {
      RCLCPP_INFO(
        this->get_logger(),
        "Fast reconnect: reusing the camera profile already applied in this process");
      return true;
    }

    // Apply the camera profile only once per process. Rewriting the full GenICam
    // profile on every reconnect creates many GVCP transactions and makes a
    // momentary GigE fault much harder to recover from.
    if (!setEnumString(handle, "TriggerMode", "Off", true)) {
      return false;
    }

    if (image_width_ > 0 || image_height_ > 0) {
      if (!setIntValue(handle, "OffsetX", 0, false)) {
        return false;
      }
      if (!setIntValue(handle, "OffsetY", 0, false)) {
        return false;
      }
    }
    if (image_width_ > 0 && !setIntValue(handle, "Width", image_width_, true)) {
      return false;
    }
    if (image_height_ > 0 && !setIntValue(handle, "Height", image_height_, true)) {
      return false;
    }

    // DA4714681 already produces BayerRG8. No ADCBitDepth operation is issued.
    if (!setEnumString(handle, "PixelFormat", pixel_format_, true)) {
      return false;
    }

    // Frame rate is retained because it directly limits GigE bandwidth.
    // Exposure and gain remain optional, but a network failure still aborts the
    // current attempt instead of producing a cascade of misleading warnings.
    if (!setBoolValue(handle, "AcquisitionFrameRateEnable", true, false)) {
      return false;
    }
    if (!setFloatValue(
        handle, "AcquisitionFrameRate", acquisition_frame_rate_, false))
    {
      return false;
    }
    if (!setFloatValue(handle, "ExposureTime", exposure_time_, false)) {
      return false;
    }
    if (!setFloatValue(handle, "Gain", gain_, false)) {
      return false;
    }

    camera_profile_applied_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Camera profile applied once: requested=%dx%d, pixel=%s, fps=%.3f, "
      "exposure=%.0f us, gain=%.3f, SDK_buffers=%d, GigE_packet_size=%d, "
      "GigE_packet_delay=%d",
      image_width_, image_height_, pixel_format_.c_str(), acquisition_frame_rate_,
      static_cast<double>(exposure_time_), gain_, image_node_num_,
      gige_packet_size_, gige_packet_delay_);
    return true;
  }

  void destroyTemporaryHandle(
    void * handle, const bool grabbing, const bool opened)
  {
    if (handle == nullptr) {
      return;
    }

    if (grabbing) {
      MV_CC_StopGrabbing(handle);
    }
    if (opened) {
      MV_CC_CloseDevice(handle);
    }
    MV_CC_DestroyHandle(handle);
  }

  bool connectCameraOnce()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);

    if (camera_handle_ != nullptr) {
      return true;
    }

    MV_CC_DEVICE_INFO_LIST device_list{};
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to enumerate cameras: %s",
        sdkErrorText(ret).c_str());
      return false;
    }

    if (device_list.nDeviceNum == 0) {
      RCLCPP_WARN(this->get_logger(), "No Hikrobot camera detected");
      return false;
    }

    if (camera_sn_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameter 'camera_sn' must be set; refusing to select an arbitrary camera");
      return false;
    }

    MV_CC_DEVICE_INFO * selected_device = nullptr;
    unsigned int selected_index = 0;

    // MV_CC_EnumDevices returns a list for the transport layer. The driver only
    // compares serial-number metadata until the configured serial is found.
    // It does not create a handle, query accessibility, read network settings,
    // log, open, configure, or otherwise operate any non-matching camera.
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      MV_CC_DEVICE_INFO * device = device_list.pDeviceInfo[i];
      if (device == nullptr) {
        continue;
      }

      if (serialNumberOf(device) == camera_sn_) {
        selected_device = device;
        selected_index = i;
        break;
      }
    }

    if (selected_device == nullptr) {
      RCLCPP_WARN(
        this->get_logger(), "Target camera SN [%s] was not detected",
        camera_sn_.c_str());
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(), "Matched target camera: SN=%s, transport=%s",
      camera_sn_.c_str(), transportName(selected_device->nTLayerType));

    if (!MV_CC_IsDeviceAccessible(selected_device, MV_ACCESS_Exclusive)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Target camera SN [%s] is not exclusively accessible",
        camera_sn_.c_str());
      return false;
    }

    const std::string selected_serial = serialNumberOf(selected_device);
    const unsigned int selected_transport = selected_device->nTLayerType;

    void * new_handle = nullptr;
    bool opened = false;

    ret = MV_CC_CreateHandle(&new_handle, selected_device);
    if (ret != MV_OK || new_handle == nullptr) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to create camera handle: %s",
        sdkErrorText(ret).c_str());
      return false;
    }

    ret = MV_CC_OpenDevice(new_handle, MV_ACCESS_Exclusive, 0);
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to open camera SN [%s]: %s",
        selected_serial.c_str(), sdkErrorText(ret).c_str());
      destroyTemporaryHandle(new_handle, false, false);
      return false;
    }
    opened = true;

    if (open_settle_ms_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(open_settle_ms_));
    }

    if (!configureCamera(new_handle, selected_transport)) {
      destroyTemporaryHandle(new_handle, false, opened);
      return false;
    }

    ret = MV_CC_StartGrabbing(new_handle);
    if (ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to start camera grabbing: %s",
        sdkErrorText(ret).c_str());
      destroyTemporaryHandle(new_handle, false, opened);
      return false;
    }
    // Expose the successfully started handle immediately. The normal capture
    // loop performs timeout counting and recovery; a separate validation read
    // caused unnecessary open/close churn on an already unstable GigE link.
    camera_handle_ = new_handle;
    transport_type_ = selected_transport;

    RCLCPP_INFO(
      this->get_logger(),
      "Camera connected and grabbing started: index=%u, transport=%s, SN=%s",
      selected_index, transportName(transport_type_), selected_serial.c_str());

    return true;
  }

  void closeCameraUnlocked()
  {
    if (camera_handle_ == nullptr) {
      return;
    }

    const int stop_ret = MV_CC_StopGrabbing(camera_handle_);
    if (stop_ret != MV_OK && !isIgnorableStopError(stop_ret)) {
      RCLCPP_WARN(
        this->get_logger(), "Stop grabbing failed during camera close: %s",
        sdkErrorText(stop_ret).c_str());
    }

    const int close_ret = MV_CC_CloseDevice(camera_handle_);
    if (close_ret != MV_OK &&
        !errorEquals(close_ret, static_cast<unsigned int>(MV_E_HANDLE)))
    {
      RCLCPP_WARN(
        this->get_logger(), "Close camera device failed: %s",
        sdkErrorText(close_ret).c_str());
    }

    const int destroy_ret = MV_CC_DestroyHandle(camera_handle_);
    if (destroy_ret != MV_OK) {
      RCLCPP_WARN(
        this->get_logger(), "Destroy camera handle failed: %s",
        sdkErrorText(destroy_ret).c_str());
    }

    camera_handle_ = nullptr;
    transport_type_ = 0;
  }

  void closeCamera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    closeCameraUnlocked();
  }

  void recoverCamera(const std::string & reason, const int error_code)
  {
    if (!enable_auto_reconnect_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Camera recovery is disabled; reason=%s, code=%s", reason.c_str(),
        sdkErrorText(error_code).c_str());
      return;
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Camera stream lost; reconnecting. reason=%s, code=%s", reason.c_str(),
      sdkErrorText(error_code).c_str());
    closeCamera();
    sleepInterruptibly(reconnect_interval_ms_);
  }

  void sleepInterruptibly(const int milliseconds)
  {
    int remaining = milliseconds;
    while (remaining > 0 && running_.load() && rclcpp::ok()) {
      const int sleep_ms = std::min(remaining, 100);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      remaining -= sleep_ms;
    }
  }

  void captureLoop()
  {
    RCLCPP_INFO(this->get_logger(), "Camera capture thread started");

    int consecutive_timeouts = 0;
    int consecutive_capture_errors = 0;
    bool recovering_from_stream_loss = false;
    bool first_frame_logged = false;
    auto last_timeout_log =
      std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto last_rate_log = std::chrono::steady_clock::now();
    auto last_packet_loss_summary = std::chrono::steady_clock::now();

    uint64_t packet_window_frames = 0U;
    uint64_t packet_window_affected_frames = 0U;
    uint64_t packet_window_reported_lost = 0U;
    unsigned int packet_window_max_per_frame = 0U;
    GigENetworkStats previous_net_stats{};
    bool previous_net_stats_valid = false;

    while (running_.load() && rclcpp::ok()) {
      bool camera_available = false;
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_available = camera_handle_ != nullptr;
      }

      if (!camera_available) {
        if (connectCameraOnce()) {
          if (recovering_from_stream_loss) {
            RCLCPP_INFO(this->get_logger(), "Camera stream recovered successfully");
          }
          recovering_from_stream_loss = false;
          consecutive_timeouts = 0;
          consecutive_capture_errors = 0;
          first_frame_logged = false;
          packet_window_frames = 0U;
          packet_window_affected_frames = 0U;
          packet_window_reported_lost = 0U;
          packet_window_max_per_frame = 0U;
          previous_net_stats_valid = false;
          last_packet_loss_summary = std::chrono::steady_clock::now();
          continue;
        }

        recovering_from_stream_loss = true;
        sleepInterruptibly(reconnect_interval_ms_);
        continue;
      }

      MV_FRAME_OUT out_frame{};
      int get_ret = MV_E_HANDLE;
      int convert_ret = MV_E_HANDLE;
      int free_ret = MV_OK;
      bool frame_ready = false;
      unsigned int frame_number = 0;
      unsigned int lost_packets = 0;
      unsigned int frame_width = 0;
      unsigned int frame_height = 0;

      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (camera_handle_ == nullptr) {
          continue;
        }

        get_ret = MV_CC_GetImageBuffer(
          camera_handle_, &out_frame,
          static_cast<unsigned int>(frame_timeout_ms_));

        if (get_ret == MV_OK) {
          frame_width = out_frame.stFrameInfo.nWidth;
          frame_height = out_frame.stFrameInfo.nHeight;
          frame_number = out_frame.stFrameInfo.nFrameNum;
          lost_packets = out_frame.stFrameInfo.nLostPacket;

          const std::size_t width = static_cast<std::size_t>(frame_width);
          const std::size_t height = static_cast<std::size_t>(frame_height);
          const bool dimensions_valid =
            width > 0U && height > 0U &&
            width <= std::numeric_limits<std::size_t>::max() / height / 3U;

          if (!dimensions_valid) {
            convert_ret = MV_E_PARAMETER;
          } else {
            const std::size_t rgb_size = width * height * 3U;
            image_msg_.data.resize(rgb_size);

            MV_CC_PIXEL_CONVERT_PARAM convert_param{};
            convert_param.nWidth = out_frame.stFrameInfo.nWidth;
            convert_param.nHeight = out_frame.stFrameInfo.nHeight;
            convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            convert_param.pSrcData = out_frame.pBufAddr;
            convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            convert_param.pDstBuffer = image_msg_.data.data();
            convert_param.nDstBufferSize =
              static_cast<unsigned int>(image_msg_.data.size());

            convert_ret = MV_CC_ConvertPixelType(camera_handle_, &convert_param);
            if (convert_ret == MV_OK) {
              image_msg_.data.resize(convert_param.nDstLen);
              frame_ready = true;
            }
          }

          // Release the SDK-owned source buffer before ROS publication.
          free_ret = MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
        }
      }

      if (!running_.load() || !rclcpp::ok()) {
        break;
      }

      if (get_ret == MV_OK) {
        consecutive_timeouts = 0;

        if (free_ret != MV_OK) {
          RCLCPP_WARN(
            this->get_logger(), "Free image buffer failed: %s",
            sdkErrorText(free_ret).c_str());
        }

        if (!frame_ready || convert_ret != MV_OK) {
          ++consecutive_capture_errors;
          RCLCPP_ERROR(
            this->get_logger(), "Pixel conversion failed: %s (count=%d)",
            sdkErrorText(convert_ret).c_str(), consecutive_capture_errors);

          if (consecutive_capture_errors >= 3) {
            recoverCamera("repeated pixel-conversion failure", convert_ret);
            recovering_from_stream_loss = true;
            consecutive_capture_errors = 0;
            first_frame_logged = false;
          }
          continue;
        }

        consecutive_capture_errors = 0;
        image_msg_.header.stamp = this->now();
        image_msg_.height = frame_height;
        image_msg_.width = frame_width;
        image_msg_.step = frame_width * 3U;

        camera_info_msg_.header = image_msg_.header;
        camera_info_msg_.width = frame_width;
        camera_info_msg_.height = frame_height;
        camera_pub_.publish(image_msg_, camera_info_msg_);

        if (!first_frame_logged) {
          RCLCPP_INFO(
            this->get_logger(), "Publishing image: %ux%u RGB8, first frame=%u",
            frame_width, frame_height, frame_number);
          first_frame_logged = true;
        }

        ++packet_window_frames;
        if (lost_packets > 0U) {
          ++packet_window_affected_frames;
          packet_window_reported_lost += lost_packets;
          packet_window_max_per_frame =
            std::max(packet_window_max_per_frame, lost_packets);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_packet_loss_summary >=
            std::chrono::seconds(packet_loss_summary_interval_sec_))
        {
          GigENetworkStats current_net_stats{};
          int stats_ret = MV_E_HANDLE;
          unsigned int current_transport = 0U;
          {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            current_transport = transport_type_;
            if (camera_handle_ != nullptr && transport_type_ == MV_GIGE_DEVICE) {
              stats_ret = readGigENetworkStats(camera_handle_, current_net_stats);
            }
          }

          int64_t sdk_lost_packet_delta = 0;
          unsigned int sdk_lost_frame_delta = 0U;
          int64_t resend_request_delta = 0;
          int64_t resent_packet_delta = 0;
          if (stats_ret == MV_OK) {
            if (previous_net_stats_valid &&
                current_net_stats.received_bytes >= previous_net_stats.received_bytes)
            {
              sdk_lost_packet_delta = std::max<int64_t>(
                0, current_net_stats.lost_packets - previous_net_stats.lost_packets);
              sdk_lost_frame_delta =
                current_net_stats.lost_frames >= previous_net_stats.lost_frames ?
                current_net_stats.lost_frames - previous_net_stats.lost_frames : 0U;
              resend_request_delta = std::max<int64_t>(
                0, current_net_stats.requested_resend_packets -
                previous_net_stats.requested_resend_packets);
              resent_packet_delta = std::max<int64_t>(
                0, current_net_stats.resent_packets - previous_net_stats.resent_packets);
            }
            previous_net_stats = current_net_stats;
            previous_net_stats_valid = true;
          }

          const bool frame_metadata_reports_loss =
            packet_window_affected_frames > 0U;
          const bool sdk_reports_loss =
            sdk_lost_packet_delta > 0 || sdk_lost_frame_delta > 0U;

          if (frame_metadata_reports_loss || sdk_reports_loss) {
            if (stats_ret == MV_OK) {
              RCLCPP_WARN(
                this->get_logger(),
                "GigE packet-loss summary (%d s): frames=%llu, affected=%llu, "
                "frame_metadata_lost=%llu, max_per_frame=%u, "
                "sdk_lost_packets=%lld, sdk_lost_frames=%u, "
                "resend_requested=%lld, resend_received=%lld",
                packet_loss_summary_interval_sec_,
                static_cast<unsigned long long>(packet_window_frames),
                static_cast<unsigned long long>(packet_window_affected_frames),
                static_cast<unsigned long long>(packet_window_reported_lost),
                packet_window_max_per_frame,
                static_cast<long long>(sdk_lost_packet_delta),
                sdk_lost_frame_delta,
                static_cast<long long>(resend_request_delta),
                static_cast<long long>(resent_packet_delta));
            } else {
              RCLCPP_WARN(
                this->get_logger(),
                "GigE packet-loss summary (%d s): frames=%llu, affected=%llu, "
                "frame_metadata_lost=%llu, max_per_frame=%u; "
                "SDK network statistics unavailable: %s",
                packet_loss_summary_interval_sec_,
                static_cast<unsigned long long>(packet_window_frames),
                static_cast<unsigned long long>(packet_window_affected_frames),
                static_cast<unsigned long long>(packet_window_reported_lost),
                packet_window_max_per_frame,
                current_transport == MV_GIGE_DEVICE ?
                sdkErrorText(stats_ret).c_str() : "not a GigE camera");
            }
          }

          packet_window_frames = 0U;
          packet_window_affected_frames = 0U;
          packet_window_reported_lost = 0U;
          packet_window_max_per_frame = 0U;
          last_packet_loss_summary = now;
        }

        if (now - last_rate_log >= std::chrono::seconds(5)) {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          if (camera_handle_ != nullptr) {
            MVCC_FLOATVALUE value{};
            const int rate_ret =
              MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &value);
            if (rate_ret == MV_OK) {
              RCLCPP_DEBUG(
                this->get_logger(), "ResultingFrameRate: %.3f Hz", value.fCurValue);
            }
          }
          last_rate_log = now;
        }
        continue;
      }

      if (errorEquals(get_ret, static_cast<unsigned int>(MV_E_NODATA))) {
        ++consecutive_timeouts;
        consecutive_capture_errors = 0;

        bool device_connected = false;
        {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          if (camera_handle_ != nullptr) {
            device_connected = MV_CC_IsDeviceConnected(camera_handle_);
          }
        }

        const auto now = std::chrono::steady_clock::now();
        if (consecutive_timeouts == 1 ||
            now - last_timeout_log >= std::chrono::seconds(2))
        {
          RCLCPP_WARN(
            this->get_logger(),
            "GetImageBuffer timeout: no frame for %d ms, consecutive=%d/%d, "
            "device_connected=%s",
            frame_timeout_ms_, consecutive_timeouts, timeout_recovery_threshold_,
            device_connected ? "true" : "false");
          last_timeout_log = now;
        }

        if (consecutive_timeouts >= timeout_recovery_threshold_) {
          recoverCamera(
            device_connected ? "consecutive frame timeouts" : "device disconnected",
            get_ret);
          recovering_from_stream_loss = true;
          consecutive_timeouts = 0;
          first_frame_logged = false;
        }
        continue;
      }

      consecutive_timeouts = 0;
      ++consecutive_capture_errors;
      RCLCPP_ERROR(
        this->get_logger(), "GetImageBuffer failed: %s (count=%d)",
        sdkErrorText(get_ret).c_str(), consecutive_capture_errors);

      recoverCamera("GetImageBuffer fatal error", get_ret);
      recovering_from_stream_loss = true;
      consecutive_capture_errors = 0;
      first_frame_logged = false;
    }

    RCLCPP_INFO(this->get_logger(), "Camera capture thread stopped");
  }

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(camera_mutex_);

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      int ret = MV_OK;

      if (name == "gain") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.successful = false;
          result.reason = "gain must be numeric";
          break;
        }

        const double value =
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
          parameter.as_double() : static_cast<double>(parameter.as_int());
        if (camera_handle_ != nullptr) {
          ret = MV_CC_SetFloatValue(
            camera_handle_, "Gain", static_cast<float>(value));
        }
        if (ret == MV_OK) {
          gain_ = value;
        }
      } else if (name == "exposure_time") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER &&
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.successful = false;
          result.reason = "exposure_time must be numeric";
          break;
        }

        const double value =
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
          static_cast<double>(parameter.as_int()) : parameter.as_double();
        if (camera_handle_ != nullptr) {
          ret = MV_CC_SetFloatValue(
            camera_handle_, "ExposureTime", static_cast<float>(value));
        }
        if (ret == MV_OK) {
          exposure_time_ = static_cast<int>(value);
        }
      } else if (name == "acquisition_frame_rate") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.successful = false;
          result.reason = "acquisition_frame_rate must be numeric";
          break;
        }

        const double value =
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
          parameter.as_double() : static_cast<double>(parameter.as_int());
        if (camera_handle_ != nullptr) {
          ret = MV_CC_SetBoolValue(
            camera_handle_, "AcquisitionFrameRateEnable", true);
          if (ret == MV_OK) {
            ret = MV_CC_SetFloatValue(
              camera_handle_, "AcquisitionFrameRate", static_cast<float>(value));
          }
        }
        if (ret == MV_OK) {
          acquisition_frame_rate_ = value;
        }
      } else {
        result.successful = false;
        result.reason = "Parameter is not dynamically writable: " + name;
        break;
      }

      if (ret != MV_OK) {
        result.successful = false;
        result.reason =
          "Camera rejected " + name + " with " + sdkErrorText(ret);
        break;
      }
    }

    return result;
  }

  std::mutex camera_mutex_;
  void * camera_handle_ = nullptr;
  unsigned int transport_type_ = 0;
  MV_IMAGE_BASIC_INFO img_info_{};

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    params_callback_handle_;

  std::string camera_sn_;
  std::string pixel_format_;
  double acquisition_frame_rate_ = 10.0;
  int exposure_time_ = 120;
  double gain_ = 16.9;
  int image_width_ = 1280;
  int image_height_ = 720;
  int image_node_num_ = 8;

  int frame_timeout_ms_ = 1000;
  int startup_frame_timeout_ms_ = 2000;
  int timeout_recovery_threshold_ = 3;
  int reconnect_interval_ms_ = 1000;
  int open_settle_ms_ = 300;
  bool enable_auto_reconnect_ = true;

  bool reuse_camera_profile_on_reconnect_ = true;
  bool set_gige_packet_size_on_connect_ = true;
  int gige_packet_size_ = 1500;
  int gige_packet_delay_ = 0;
  bool gige_enable_resend_ = true;
  int gige_max_resend_percent_ = 20;
  int gige_resend_timeout_ms_ = 50;
  int gige_resend_retry_times_ = 5;
  int gige_resend_interval_ms_ = 10;
  int packet_loss_summary_interval_sec_ = 5;
  bool camera_profile_applied_ = false;

  bool use_sensor_data_qos_ = true;
  std::string camera_name_;
  std::string frame_id_;
  std::string camera_topic_;
  std::string camera_info_url_;

  std::atomic<bool> running_{true};
  std::thread capture_thread_;
};
}  // namespace hik_camera_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_ros2_driver::HikCameraRos2DriverNode)

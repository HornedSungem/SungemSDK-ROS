

#ifndef HORNED_SUNGEM_LIB_DEVICE_H
#define HORNED_SUNGEM_LIB_DEVICE_H

#include <string>
#include <memory>
#include <horned_sungem_lib/hs_cpp.h>
#include <opencv2/opencv.hpp>
namespace horned_sungem_lib
{
class Device
{
public:
  using Ptr = std::shared_ptr<Device>;
  using ConstPtr = std::shared_ptr<Device const>;

  enum LogLevel
  {
    Nothing,
    Errors,
    Verbose
  };

  Device(int index, LogLevel log_level);
  ~Device();

  void setLogLevel(LogLevel level);
  LogLevel getLogLevel();
  void* getHandle();
  std::string getName() const;
  void monitorThermal() const;
  cv::Mat getDeviceImage(bool truthy);
private:
  enum ThermalThrottlingLevel
  {
    Normal,
    High,
    Aggressive
  };

  void open();
  void close();

  void find();
  ThermalThrottlingLevel  getThermalThrottlingLevel() const;

  int index_;
  std::string name_;
  void* handle_;
};
}  // namespace horned_sungem_lib

#endif  // HORNED_SUNGEM_LIB_DEVICE_H

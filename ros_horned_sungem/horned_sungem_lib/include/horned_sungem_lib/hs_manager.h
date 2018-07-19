

#ifndef HORNED_SUNGEM_LIB_HS_MANAGER_H
#define HORNED_SUNGEM_LIB_HS_MANAGER_H

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "boost/bind.hpp"
#include "boost/function.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include "horned_sungem_lib/device.h"
#include "horned_sungem_lib/connect.h"

#define IMAGE_BUFFER_SIZE 10

namespace horned_sungem_lib
{
typedef boost::function<void(ClassificationResultPtr result, std_msgs::Header header)> FUNP_C;
typedef boost::function<void(DetectionResultPtr result, std_msgs::Header header)> FUNP_D;
typedef boost::function<void(ClassificationResultPtr result, cv::Mat &image)> FUNP_E;
typedef boost::function<void(DetectionResultPtr result, cv::Mat &image)> FUNP_F;

struct ImageFrame
{
  cv::Mat image;
  std_msgs::Header header;
};

class HSManager
{
public:
  HSManager(const int &max_device_number,
            const int &device_index,
            const Device::LogLevel &log_level,
            const std::string &camera,
            const int &pixels,
            const std::string &cnn_type,
            const std::string &graph_file_path,
            const std::string &category_file_path,
            const int &network_dimension,
            const std::vector<float> &mean,
            const float &scale,
            const int &top_n);
  ~HSManager();

  void startThreads();

  void classifyStream(const cv::Mat &image, FUNP_C cbGetClassificationResult,
                      const sensor_msgs::ImageConstPtr &image_msg);
  void detectStream(const cv::Mat &image, FUNP_D cbGetDetectionResult,
                    const sensor_msgs::ImageConstPtr &image_msg);
  void startClassifyThreads(FUNP_E cbGetClassificationResultBySelf);
  void startDetectThreads(FUNP_F cbGetDetectionResultBySelf);

private:
  void initDeviceManager();
  void deviceThread(int device_index);
  void deviceBySelfThread(int device_index);
  const int max_device_number_;
  const int start_device_index_;
  const Device::LogLevel log_level_;
  const std::string camera_;
  const int pixels_;
  const std::string cnn_type_;
  const std::string graph_file_path_;
  const std::string category_file_path_;
  const int network_dimension_;
  const std::vector<float> mean_;
  const float scale_;
  const int top_n_;
  void *user_param_;

  std::vector<std::shared_ptr<horned_sungem_lib::HS>> hs_handle_list_;
  int device_count_;

  FUNP_C p_c_;
  FUNP_D p_d_;
  FUNP_E p_e_;
  FUNP_F p_f_;

  std::vector<ImageFrame> image_list_;
  std::mutex mtx_;
  std::vector<std::thread> threads_;
};
} // namespace horned_sungem_lib
#endif // HORNED_SUNGEM_LIB_HS_MANAGER_H



#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "horned_sungem_lib/hs_manager.h"

namespace horned_sungem_lib
{
HSManager::HSManager(const int &max_device_number,
                     const int &start_device_index,
                     const Device::LogLevel &log_level,
                     const std::string &camera,
                     const int &pixels,
                     const std::string &cnn_type,
                     const std::string &graph_file_path,
                     const std::string &category_file_path,
                     const int &network_dimension,
                     const std::vector<float> &mean,
                     const float &scale,
                     const int &top_n)
    : max_device_number_(max_device_number),
      start_device_index_(start_device_index),
      log_level_(log_level),
      camera_(camera),
      pixels_(pixels),
      cnn_type_(cnn_type),
      graph_file_path_(graph_file_path),
      category_file_path_(category_file_path),
      network_dimension_(network_dimension),
      mean_(mean),
      scale_(scale),
      top_n_(top_n),
      user_param_(nullptr),
      device_count_(0)
{
  initDeviceManager();
}

HSManager::~HSManager()
{
}

void HSManager::initDeviceManager()
{
  char device_name[HS_MAX_NAME_SIZE];
  while (hsGetDeviceName(device_count_, device_name, sizeof(device_name)) != HS_DEVICE_NOT_FOUND)
  {

    device_count_++;

    if (device_count_ == max_device_number_)
    {
      break;
    }
  }

  assert(start_device_index_ <= device_count_);

  for (int device_index = start_device_index_; device_index < device_count_; device_index++)
  {
    auto hs_handle = std::make_shared<horned_sungem_lib::HS>(device_index, static_cast<Device::LogLevel>(log_level_), camera_,
                                                              cnn_type_, graph_file_path_, category_file_path_,
                                                              network_dimension_, mean_, scale_, top_n_);
    hs_handle_list_.push_back(hs_handle);
  }
}

void join(std::thread &t)
{
  t.join();
}

void HSManager::deviceThread(int device_index)
{

  while (1)
  {
    if (!image_list_.empty())
    {
      mtx_.lock();
      auto first_image = image_list_[0].image;
      auto first_image_header = image_list_[0].header;

      if (!image_list_.empty())
      {
        image_list_.erase(image_list_.begin());
      }
      else
      {
        mtx_.unlock();
        continue;
      }
      mtx_.unlock();
      if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet") || !cnn_type_.compare("inception_v1") ||
          !cnn_type_.compare("inception_v2") || !cnn_type_.compare("inception_v3") ||
          !cnn_type_.compare("inception_v4") || !cnn_type_.compare("mobilenet") || !cnn_type_.compare("squeezenet"))
      {

        hs_handle_list_[device_index]->loadTensor(first_image);
        hs_handle_list_[device_index]->classify();
        ClassificationResultPtr result = hs_handle_list_[device_index]->getClassificationResult();
        p_c_(result, first_image_header);
      }
      else
      {

        hs_handle_list_[device_index]->loadTensor(first_image);

        hs_handle_list_[device_index]->detect();
        DetectionResultPtr result = hs_handle_list_[device_index]->getDetectionResult();

        p_d_(result, first_image_header);
      }
    }
    else
    {
      usleep(1);
    }
  }
}
void HSManager::deviceBySelfThread(int device_index)
{
  while (1)
  {
    cv::Mat mat;
    if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet") || !cnn_type_.compare("inception_v1") ||
        !cnn_type_.compare("inception_v2") || !cnn_type_.compare("inception_v3") ||
        !cnn_type_.compare("inception_v4") || !cnn_type_.compare("mobilenet") || !cnn_type_.compare("squeezenet"))
    {
      if (pixels_ == 360)
        mat = hs_handle_list_[device_index]->getImage(true);
      else
        mat = hs_handle_list_[device_index]->getImage(false);
      hs_handle_list_[device_index]->classify();
      ClassificationResultPtr result = hs_handle_list_[device_index]->getClassificationResult();
      p_e_(result, mat);
    }
    else
    {
      if (pixels_ == 360)
        mat = hs_handle_list_[device_index]->getImage(true);
      else
        mat = hs_handle_list_[device_index]->getImage(false);
      hs_handle_list_[device_index]->detect();
      DetectionResultPtr result = hs_handle_list_[device_index]->getDetectionResult();
      p_f_(result, mat);
    }
  }
}
void HSManager::startThreads()
{
  for (int i = 0; i < device_count_ - start_device_index_; i++)
  {
    if (camera_.compare("hs"))
      threads_.push_back(std::thread(&HSManager::deviceThread, this, i));
    else
      threads_.push_back(std::thread(&HSManager::deviceBySelfThread, this, i));
  }

  std::for_each(threads_.begin(), threads_.end(), join);

  ROS_INFO("started %d threads", device_count_ - start_device_index_);
}
void HSManager::startClassifyThreads(FUNP_E cbGetClassificationResultBySelf)
{
  p_e_ = cbGetClassificationResultBySelf;
  startThreads();
}
void HSManager::startDetectThreads(FUNP_F cbGetDetectionResultBySelf)
{
  p_f_ = cbGetDetectionResultBySelf;
  startThreads();
}


void HSManager::classifyStream(const cv::Mat &image, FUNP_C cbGetClassificationResult,
                               const sensor_msgs::ImageConstPtr &image_msg)
{
  p_c_ = cbGetClassificationResult;

  ImageFrame imageFrame;
  imageFrame.header = image_msg->header;
  imageFrame.image = image;

  mtx_.lock();
  if (image_list_.size() > IMAGE_BUFFER_SIZE)
  {
    image_list_.erase(image_list_.begin());
  }
  image_list_.push_back(imageFrame);
  mtx_.unlock();
}

void HSManager::detectStream(const cv::Mat &image, FUNP_D cbGetDetectionResult,
                             const sensor_msgs::ImageConstPtr &image_msg)
{
  p_d_ = cbGetDetectionResult;
  ImageFrame imageFrame;
  imageFrame.header = image_msg->header;
  imageFrame.image = image;
  mtx_.lock();
  if (image_list_.size() > IMAGE_BUFFER_SIZE)
  {
    image_list_.erase(image_list_.begin());
  }
  image_list_.push_back(imageFrame);
  mtx_.unlock();
}
} // namespace horned_sungem_lib

/*
 * Copyright (c) 2018 HornedSungem Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HORNED_SUNGEM_STREAM_HS_NODELET_H
#define HORNED_SUNGEM_STREAM_HS_NODELET_H

#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "horned_sungem_lib/connect.h"
#include "horned_sungem_lib/hs_manager.h"
#include "horned_sungem_lib/result.h"
#include "horned_sungem_lib/device.h"
namespace horned_sungem_stream
{
typedef boost::function<void(horned_sungem_lib::ClassificationResultPtr result, std_msgs::Header header)> FUNP_C;
typedef boost::function<void(horned_sungem_lib::DetectionResultPtr result, std_msgs::Header header)> FUNP_D;
typedef boost::function<void(horned_sungem_lib::ClassificationResultPtr result, cv::Mat &image)> FUNP_E;
typedef boost::function<void(horned_sungem_lib::DetectionResultPtr result, cv::Mat &image)> FUNP_F;

class HSImpl
{
public:
  HSImpl(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~HSImpl();

  void cbGetClassificationResult(horned_sungem_lib::ClassificationResultPtr result, std_msgs::Header header);
  void cbGetDetectionResult(horned_sungem_lib::DetectionResultPtr result, std_msgs::Header header);
  void cbGetClassificationResultBySelf(horned_sungem_lib::ClassificationResultPtr result, cv::Mat &image);
  void cbGetDetectionResultBySelf(horned_sungem_lib::DetectionResultPtr result, cv::Mat &image);

private:
  void cbClassify(const sensor_msgs::ImageConstPtr &image);
  void cbDetect(const sensor_msgs::ImageConstPtr &image);
  void getParameters();
  void init();

  std::shared_ptr<horned_sungem_lib::HSManager> hs_manager_handle_;

  ros::Publisher pub_;

  image_transport::Publisher pubframe_;

  image_transport::Subscriber sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  horned_sungem_lib::Device::Ptr device_;
  int max_device_number_;
  int start_device_index_;
  int log_level_;
  std::string camera_;
  int pixels_;
  std::string cnn_type_;
  std::string graph_file_path_;
  std::string category_file_path_;
  int network_dimension_;
  std::vector<float> mean_;
  float scale_;
  int top_n_;
};

class HSNodelet : public nodelet::Nodelet
{
public:
  virtual ~HSNodelet();

private:
  virtual void onInit();

  std::shared_ptr<HSImpl> impl_;
};
} // namespace horned_sungem_stream

#endif //HORNED_SUNGEM_STREAM_HS_NODELET_H

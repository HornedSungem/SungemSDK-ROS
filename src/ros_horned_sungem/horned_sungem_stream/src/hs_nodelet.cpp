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

#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <object_msgs/Objects.h>
#include <object_msgs/ObjectsInBoxes.h>
#include "horned_sungem_stream/hs_nodelet.h"
#include "horned_sungem_lib/exception.h"
#include "horned_sungem_lib/exception_util.h"
#include "horned_sungem_lib/result.h"

using horned_sungem_lib::ClassificationResultPtr;
using horned_sungem_lib::DetectionResultPtr;
using horned_sungem_lib::Device;
using image_transport::ImageTransport;

namespace horned_sungem_stream
{
HSImpl::HSImpl(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : hs_manager_handle_(nullptr),
      nh_(nh),
      pnh_(pnh),
      device_(nullptr),
      max_device_number_(255),
      start_device_index_(0),
      log_level_(Device::Errors),
      camera_(""),
      pixels_(0),
      cnn_type_(""),
      graph_file_path_(""),
      category_file_path_(""),
      network_dimension_(0),
      mean_(0),
      scale_(1.0),
      top_n_(1)
{
  getParameters();
  init();
}

HSImpl::~HSImpl()
{
}

void HSImpl::getParameters()
{
  ROS_DEBUG("HSImpl get parameters");

  if (!pnh_.getParam("max_device_number", max_device_number_))
  {
    ROS_WARN("param max_device_number not set, use default");
  }

  if (max_device_number_ <= 0)
  {
    ROS_ERROR_STREAM("invalid param max_device_number = " << max_device_number_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use max_device_number = " << max_device_number_);

  if (!pnh_.getParam("start_device_index", start_device_index_))
  {
    ROS_WARN("param start_device_index not set, use default");
  }

  if (start_device_index_ < 0)
  {
    ROS_ERROR_STREAM("invalid param start_device_index = " << start_device_index_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use start_device_index = " << start_device_index_);

  if (!pnh_.getParam("log_level", log_level_))
  {
    ROS_WARN("param log_level not set, use default");
  }

  if (log_level_ < Device::Nothing || log_level_ > Device::Verbose)
  {
    ROS_WARN_STREAM("invalid param log_level = " << log_level_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use log_level = " << log_level_);

  if (!pnh_.getParam("graph_file_path", graph_file_path_))
  {
    ROS_WARN("param graph_file_path not set, use default");
  }

  if (!boost::filesystem::exists(graph_file_path_))
  {
    ROS_ERROR_STREAM("graph_file_path = " << graph_file_path_ << " not exists");
    throw std::exception();
  }

  ROS_INFO_STREAM("use graph_file_path = " << graph_file_path_);

  if (!pnh_.getParam("category_file_path", category_file_path_))
  {
    ROS_WARN("param category_file_path not set, use default");
  }

  if (!boost::filesystem::exists(category_file_path_))
  {
    ROS_ERROR_STREAM("category_file_path = " << category_file_path_ << " not exists");
    throw std::exception();
  }

  ROS_INFO_STREAM("use category_file_path = " << category_file_path_);
  if (!pnh_.getParam("camera", camera_))
  {
    ROS_WARN("param camera not set, use default");
  }
  ROS_INFO_STREAM("use camera = " << camera_);

  if (!pnh_.getParam("pixels", pixels_))
  {
    ROS_WARN("param pixels not set, use default");
  }

  if (pixels_ < 0)
  {
    ROS_ERROR_STREAM("invalid param pixels_ = " << pixels_);
    throw std::exception();
  }
  ROS_INFO_STREAM("use pixels = " << pixels_);

  if (!pnh_.getParam("cnn_type", cnn_type_))
  {
    ROS_WARN("param cnn_type not set, use default");
  }

  if (cnn_type_.compare("alexnet") && cnn_type_.compare("googlenet") && cnn_type_.compare("inception_v1") &&
      cnn_type_.compare("inception_v2") && cnn_type_.compare("inception_v3") && cnn_type_.compare("inception_v4") &&
      cnn_type_.compare("mobilenet") && cnn_type_.compare("squeezenet") && cnn_type_.compare("yolo") &&
      cnn_type_.compare("mobilenetssd"))
  {
    ROS_WARN_STREAM("invalid cnn_type=" << cnn_type_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use cnn_type = " << cnn_type_);

  if (!pnh_.getParam("network_dimension", network_dimension_))
  {
    ROS_WARN("param network_dimension not set, use default");
  }

  if (network_dimension_ < 0)
  {
    ROS_WARN_STREAM("invalid network_dimension = " << network_dimension_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use network_dimension = " << network_dimension_);

  for (int i = 0; i < 3; i++)
  {
    std::ostringstream oss;
    oss << "channel" << (i + 1) << "_mean";
    std::string mean_param_name = oss.str();
    float mean_val;
    if (!pnh_.getParam(mean_param_name, mean_val))
    {
      ROS_WARN_STREAM("param " << mean_param_name << "not set, use default");
    }
    ROS_INFO_STREAM("use " << mean_param_name << " = " << mean_val);
    mean_.push_back(mean_val);
  }

  if (!pnh_.getParam("top_n", top_n_))
  {
    ROS_WARN("param top_n not set, use default");
  }

  if (top_n_ < 1)
  {
    ROS_WARN_STREAM("invalid top_n = " << top_n_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use top_n = " << top_n_);

  if (!pnh_.getParam("scale", scale_))
  {
    ROS_WARN("param scale not set, use default");
  }
  if (scale_ < 0)
  {
    ROS_WARN_STREAM("invalid param scale = " << scale_);
    throw std::exception();
  }

  ROS_INFO_STREAM("use scale = " << scale_);
}

void HSImpl::init()
{
   device_.reset(new Device(start_device_index_,
                           static_cast<Device::LogLevel>(log_level_)));

  hs_manager_handle_ = std::make_shared<horned_sungem_lib::HSManager>(device_,
      max_device_number_,
      start_device_index_,
      static_cast<Device::LogLevel>(log_level_),
      camera_,
      pixels_,
      cnn_type_,
      graph_file_path_,
      category_file_path_,
      network_dimension_,
      mean_,
      scale_,
      top_n_);


  std::shared_ptr<ImageTransport> it = std::make_shared<ImageTransport>(nh_);
  
  if (!cnn_type_.compare("alexnet") || !cnn_type_.compare("googlenet") || !cnn_type_.compare("inception_v1") ||
      !cnn_type_.compare("inception_v2") || !cnn_type_.compare("inception_v3") || !cnn_type_.compare("inception_v4") ||
      !cnn_type_.compare("mobilenet") || !cnn_type_.compare("squeezenet"))
  {
    if (!camera_.compare("hs"))
    {
      image_transport::ImageTransport it2(nh_);
      pubframe_ = it2.advertise("/hs/camera/image_raw", 1);
      pub_ = nh_.advertise<object_msgs::Objects>("classified_objects", 1);
      FUNP_E classification_result_self_callback = boost::bind(&HSImpl::cbGetClassificationResultBySelf, this, _1, _2);
      hs_manager_handle_->startClassifyThreads(classification_result_self_callback);
    }
    else
    {
      sub_ = it->subscribe("/hs/camera/image_raw", 1, &HSImpl::cbClassify, this);
      pub_ = nh_.advertise<object_msgs::Objects>("classified_objects", 1);
      hs_manager_handle_->startThreads();
    }
  }
  else
  {
    if (!camera_.compare("hs"))
    {
      image_transport::ImageTransport it2(nh_);
      pubframe_ = it2.advertise("/hs/camera/image_raw", 1);
      pub_ = nh_.advertise<object_msgs::ObjectsInBoxes>("detected_objects", 1);
      FUNP_F detection_result_self_callback = boost::bind(&HSImpl::cbGetDetectionResultBySelf, this, _1, _2);
      hs_manager_handle_->startDetectThreads(detection_result_self_callback);
    }
    else
    {
      sub_ = it->subscribe("/hs/camera/image_raw", 1, &HSImpl::cbDetect, this);
      pub_ = nh_.advertise<object_msgs::ObjectsInBoxes>("detected_objects", 1);
      hs_manager_handle_->startThreads();
    }
  }

} // namespace horned_sungem_stream

void HSImpl::cbClassify(const sensor_msgs::ImageConstPtr &image_msg)
{
  if (pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("skip inferring for no subscriber");
    return;
  }

  cv::Mat camera_data = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  FUNP_C classification_result_callback = boost::bind(&HSImpl::cbGetClassificationResult, this, _1, _2);
  hs_manager_handle_->classifyStream(camera_data, classification_result_callback, image_msg);
}

void HSImpl::cbDetect(const sensor_msgs::ImageConstPtr &image_msg)
{
  //重定义此类函数，定义未mat类型即可，不用改变了
  if (pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG_STREAM("skip inferring for no subscriber");
    return;
  }

  cv::Mat camera_data = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  FUNP_D detection_result_callback = boost::bind(&HSImpl::cbGetDetectionResult, this, _1, _2);
  hs_manager_handle_->detectStream(camera_data, detection_result_callback, image_msg);
}

HSNodelet::~HSNodelet()
{
}
void HSImpl::cbGetClassificationResult(horned_sungem_lib::ClassificationResultPtr result, std_msgs::Header header)
{
  object_msgs::Objects objs;

  for (auto item : result->items)
  {
    object_msgs::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    objs.objects_vector.push_back(obj);
  }

  objs.header = header;
  objs.inference_time_ms = result->time_taken;
  pub_.publish(objs);
}
void HSImpl::cbGetClassificationResultBySelf(horned_sungem_lib::ClassificationResultPtr result, cv::Mat &image)
{
  object_msgs::Objects objs;
  for (auto item : result->items)
  {
    object_msgs::Object obj;
    obj.object_name = item.category;
    obj.probability = item.probability;
    objs.objects_vector.push_back(obj);
  }
  objs.inference_time_ms = result->time_taken;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  objs.header = msg->header;
  pub_.publish(objs);
  pubframe_.publish(msg);
}

void HSImpl::cbGetDetectionResult(horned_sungem_lib::DetectionResultPtr result, std_msgs::Header header)
{
  object_msgs::ObjectsInBoxes objs_in_boxes;

  for (auto item : result->items_in_boxes)
  {
    object_msgs::ObjectInBox obj;
    obj.object.object_name = item.item.category;
    obj.object.probability = item.item.probability;
    obj.roi.x_offset = item.bbox.x;
    obj.roi.y_offset = item.bbox.y;
    obj.roi.width = item.bbox.width;
    obj.roi.height = item.bbox.height;
    objs_in_boxes.objects_vector.push_back(obj);
  }
  objs_in_boxes.header = header;
  objs_in_boxes.inference_time_ms = result->time_taken;

  pub_.publish(objs_in_boxes);
}
void HSImpl::cbGetDetectionResultBySelf(horned_sungem_lib::DetectionResultPtr result, cv::Mat &image)
{
  object_msgs::ObjectsInBoxes objs_in_boxes;

  for (auto item : result->items_in_boxes)
  {
    object_msgs::ObjectInBox obj;
    obj.object.object_name = item.item.category;
    obj.object.probability = item.item.probability;
    obj.roi.x_offset = item.bbox.x;
    obj.roi.y_offset = item.bbox.y;
    obj.roi.width = item.bbox.width;
    obj.roi.height = item.bbox.height;
    objs_in_boxes.objects_vector.push_back(obj);
  }
  objs_in_boxes.inference_time_ms = result->time_taken;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  objs_in_boxes.header = msg->header;
  pub_.publish(objs_in_boxes);
  pubframe_.publish(msg);
}

void HSNodelet::onInit()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();
  try
  {
    impl_.reset(new HSImpl(nh, pnh));
  }
  catch (horned_sungem_lib::HsException &e)
  {
    ROS_ERROR_STREAM("Error: " << e.what());
    ros::shutdown();
  }
  catch (...)
  {
    ROS_ERROR("exception caught while starting HSNodelet");
    ros::shutdown();
  }
}

} // namespace horned_sungem_stream

PLUGINLIB_EXPORT_CLASS(horned_sungem_stream::HSNodelet, nodelet::Nodelet);

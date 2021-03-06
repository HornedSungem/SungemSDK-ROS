
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <streambuf>
#include <cerrno>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "horned_sungem_lib/exception.h"
#include "horned_sungem_lib/exception_util.h"
#include "horned_sungem_lib/connect.h"
#include "horned_sungem_lib/tensor.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>

namespace horned_sungem_lib {
    HS::HS(const std::shared_ptr <Device> &device,
            int device_index,
           Device::LogLevel log_level,
           const std::string &camera,
           const std::string &cnn_type,
           const std::string &graph_file_path,
           const std::string &category_file_path,
           const int network_dimension,
           const std::vector<float> &mean,
           const float &scale,
           const int &top_n)
            : device_(device),
              graph_(nullptr),
              tensor_(nullptr),
              device_index_(device_index),
              log_level_(log_level),
              camera_(camera),
              cnn_type_(cnn_type),
              network_dimension_(network_dimension),
              mean_(mean),
              scale_(scale),
              top_n_(top_n),
              user_param_(nullptr) {
        loadGraph(graph_file_path);
        loadCategories(category_file_path);
        tensor_ = std::make_shared<Tensor>(std::pair<int, int>(network_dimension_, network_dimension_),
                                           mean_, scale_);
        result_ = std::make_shared<Result>(cnn_type_);

    }

    HS::~HS() {
    }

    cv::Mat HS::getImage(bool truthy) {
        assert(graph_->getHandle() != nullptr);
        cv::Mat mat = graph_->getGraphImage(user_param_, scale_, mean_[0] * scale_, truthy);
        tensor_->clearTensor();
        tensor_->loadImageData(mat);
        return mat;
    }

    cv::Mat HS::getDeviceImage(bool truthy) {
        assert(device_->getHandle() != nullptr);
        cv::Mat mat = device_->getDeviceImage(truthy);
        return mat;
    }

    void HS::classify() {
        assert(graph_->getHandle() != nullptr);
        try {
            uint16_t *probabilities;
            unsigned int length;
            int ret = hsGetResult(graph_->getHandle(), reinterpret_cast<void **>(&probabilities),
                                  &length, &user_param_);
            ExceptionUtil::tryToThrowHsException(ret);
            std::vector <uint16_t> result_vector(reinterpret_cast<uint16_t *>(probabilities),
                                                 reinterpret_cast<uint16_t *>(probabilities) + length);
            ItemsPtr items = std::make_shared<Items>();
            for (size_t index = 0; index < length / 2; ++index) {
                float fp32;
#if defined(__i386__) || defined(__x86_64__)
                fp32 = _cvtsh_ss(probabilities[index]);
#else
                Tensor::fp16tofp32(&fp32, probabilities[index]);
#endif
                Item item;
                item.category = categories_[index];
                item.probability = fp32;
                items->push_back(item);
            }

            auto cmp = [](const Item &a, const Item &b) {
                return a.probability > b.probability;
            };
            std::sort(items->begin(), items->end(), cmp);

            if (!result_->getClassificationResult()->items.empty()) {
                result_->getClassificationResult()->items.clear();
            }

            for (auto i : *items) {
                result_->setClassificationResult(i);
                if (static_cast<int>(result_->getClassificationResult()->items.size()) == top_n_) {
                    break;
                }
            }
            result_->setClassificationResult(graph_->getTimeTaken());
            device_->monitorThermal();
        }
        catch (HsMyriadError &e) {
            ROS_ERROR_STREAM(e.what());
            std::string debug_info = graph_->getDebugInfo();
        }
        catch (HSException &e) {
            ROS_ERROR_STREAM(e.what());
        }
    }

    void HS::detect() {
        assert(graph_->getHandle() != nullptr);
        try {
            uint16_t *result;
            unsigned int length;
            int ret = hsGetResult(graph_->getHandle(),
                                  reinterpret_cast<void **>(&result),
                                  &length,
                                  &user_param_);
            ExceptionUtil::tryToThrowHsException(ret);

            std::vector <uint16_t> result16_vector(reinterpret_cast<uint16_t *>(result),
                                                   reinterpret_cast<uint16_t *>(result) + length / 2);
            std::vector<float> result32_vector;

            for (auto fp16 : result16_vector) {
                float fp32;
#if defined(__i386__) || defined(__x86_64__)
                fp32 = _cvtsh_ss(fp16);
#else
                Tensor::fp16tofp32(&fp32, fp16);
#endif
                result32_vector.push_back(fp32);
            }

            if (!cnn_type_.compare("yolo")) {
                result_->parseYoloResult(result32_vector, categories_, tensor_->getImageWidth(),
                                         tensor_->getImageHeight());
                result_->setDetectionResult(graph_->getTimeTaken());
            } else if (!cnn_type_.compare("mobilenetssd")) {
                result_->parseSSDResult(result32_vector, categories_, tensor_->getImageWidth(),
                                        tensor_->getImageHeight());
                result_->setDetectionResult(graph_->getTimeTaken());
            } else {
                ROS_ERROR("unsupported cnn model");
            }
            device_->monitorThermal();
        }
        catch (HsMyriadError &e) {
            ROS_ERROR_STREAM(e.what());
            std::string debug_info = graph_->getDebugInfo();
            ROS_ERROR_STREAM("HornedSungem debug info: " << debug_info);
        }
        catch (std::exception &e) {
            ROS_ERROR_STREAM(e.what());
        }
    }

    void HS::loadTensor(const cv::Mat &image) {
        tensor_->clearTensor();
        tensor_->loadImageData(image);

        assert(graph_->getHandle() != nullptr);
        int ret = hsLoadTensor(graph_->getHandle(),
                               tensor_->raw(),
                               graph_->getId(),
                               tensor_->size(),
                               user_param_);
        ExceptionUtil::tryToThrowHsException(ret);
    }

    ClassificationResultPtr HS::getClassificationResult() {
        return result_->getClassificationResult();
    }

    DetectionResultPtr HS::getDetectionResult() {
        return result_->getDetectionResult();
    }

    void HS::initDevice() {
        device_.reset(new Device(device_index_,
                                 static_cast<Device::LogLevel>(log_level_)));

        ROS_INFO("device is opened");
    }

    void HS::loadGraph(const std::string &graph_file_path) {
        std::string graph = getFileContent(graph_file_path);
        graph_.reset(new Graph(device_, graph, network_dimension_));

        ROS_INFO("graph is loaded");
    }

    void HS::loadCategories(const std::string &category_file_path) {
        try {
            std::string content = getFileContent(category_file_path);
            splitIntoLines(content, categories_);
            std::string first = categories_[0];
            boost::trim_right(first);

            if (boost::iequals(first, "classes")) {
                categories_.erase(categories_.begin());
            }
        }
        catch (int &e) {
            throw HSLoadCategoriesError();
        }
    }

    void HS::splitIntoLines(const std::string &content,
                            std::vector <std::string> &lines) {
        std::stringstream ss(content);
        std::string line;

        while (std::getline(ss, line, '\n')) {
            lines.push_back(line);
        }
    }

    std::string HS::getFileContent(const std::string &filename) {
        std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);

        if (!in) {
            throw (errno);
        }

        std::string content;
        in.seekg(0, std::ios::end);
        content.reserve(in.tellg());
        in.seekg(0, std::ios::beg);
        content.assign(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
        in.close();
        return content;
    }
} // namespace horned_sungem_lib

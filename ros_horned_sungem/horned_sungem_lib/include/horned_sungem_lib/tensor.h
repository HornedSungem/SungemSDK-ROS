
#ifndef HORNED_SUNGEM_LIB_TENSOR_H
#define HORNED_SUNGEM_LIB_TENSOR_H

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include "horned_sungem_lib/hs_cpp.h"

namespace horned_sungem_lib
{
class Tensor
{
public:
  using Ptr = std::shared_ptr<Tensor>;
  using ConstPtr = std::shared_ptr<Tensor const>;

  Tensor(const std::pair<int, int>& net_dim,
         const std::vector<float>& mean,
         const float& scale);

  void loadImageData(const cv::Mat& image);
  void clearTensor();

  inline const uint16_t* raw() const
  {
    return &tensor_[0];
  }
  inline const size_t size() const
  {
    return 3 * net_height_ * net_width_ * sizeof(uint16_t);
  }
  inline int getImageWidth()
  {
    return image_width_;
  }
  inline int getImageHeight()
  {
    return image_height_;
  }
#if !defined(__i386__) && !defined(__x86_64__)
  static void fp32tofp16(uint16_t* __restrict out, float in);
  static void fp16tofp32(float* __restrict out, uint16_t in);
#endif

private:
  std::vector<uint16_t> tensor_;
  int net_width_;
  int net_height_;
  int image_width_;
  int image_height_;
  std::vector<float> mean_;
  float scale_;
};
}   // namespace horned_sungem_lib

#endif  // HORNED_SUNGEM_LIB_TENSOR_H

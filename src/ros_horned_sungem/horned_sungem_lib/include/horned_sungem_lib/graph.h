

#ifndef HORNED_SUNGEM_LIB_GRAPH_H
#define HORNED_SUNGEM_LIB_GRAPH_H

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include "horned_sungem_lib/hs_cpp.h"
#include "horned_sungem_lib/result.h"
#include "horned_sungem_lib/tensor.h"
#include "horned_sungem_lib/device.h"

namespace horned_sungem_lib {
    class Graph {
    public:
        using Ptr = std::shared_ptr<Graph>;
        using ConstPtr = std::shared_ptr<Graph const>;

        Graph(const Device::Ptr &device,
              const std::string &graph_file,
              int network_dimension);

        ~Graph();

        void allocate(void *device_handle);

        void deallocate();

        float getTimeTaken();

        std::string getDebugInfo();

        int getGraphId();

        void *getHandle();

        int getId();

        void setId(int id);

        cv::Mat getGraphImage(void *user_param, float std, float mean, bool truthy);


        int getNetworkDim() const {
            return network_dimension_;
        }

    private:
        std::string graph_buf_;
        const int network_dimension_;
        void *handle_;
        int graphId;

    };
}   // namespace horned_sungem_lib

#endif  // HORNED_SUNGEM_LIB_GRAPH_H

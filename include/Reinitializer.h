#pragma once

#include "vendor/Eigen/Eigen" 
#include <librealsense2/rs.hpp>
#include <torch/torch.h>
#include <torch/script.h>

#include <memory>
#include <atomic>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/eigen.hpp>

class Reinitializer
{
private:
    std::atomic_bool m_ready;
    std::shared_ptr<torch::jit::script::Module> m_module;
    //FootSegNet m_footSegNet;
    unsigned int count;
    
public:
    Reinitializer();
    int forward(rs2::depth_frame& depth_frame, std::vector<float>& pred, cv::Mat& mask); 
    int forward(cv::Mat& input_frame, int bbox_x, int bbox_y, int bbox_width, std::vector<float>& pred) ;
    inline bool is_ready() const { return m_ready; }
};

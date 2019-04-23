#include "Reinitializer.h"


cv::Rect Square(cv::Rect rect);
cv::Rect FindBoundingBox(cv::Mat mask);

Reinitializer::Reinitializer()
    :m_module(torch::jit::load("../res/ml/real_02_80.pt"))
{
    assert(m_module != nullptr);
    m_module->to(at::kCUDA);

    m_ready = true;
    count = 0;
}


int Reinitializer::forward(cv::Mat& input_frame, int bbox_x, int bbox_y, int bbox_width, std::vector<float>& pred) 
{
    if (m_ready == false) { return 0;} 
    else { m_ready = false; }
    
    cv::Mat inp = input_frame.clone();
    double min, max;
    //cv::minMaxLoc(cv::SparseMat(inp) , &min, &max);
    //inp = (inp - min) / (max-min) * 2 - 1;
    //inp.setTo(1, inp<-1);
    
    inp.setTo(10, inp>10);
    inp.setTo(10, inp==0);
    
    //cv::imwrite("/home/monocle/Dev/Monocle/Monocle/build/tmp/before.png", (inp - min) / (max-min) * 255);
    cv::medianBlur(inp, inp, 5);
    cv::medianBlur(inp, inp, 5);
    cv::medianBlur(inp, inp, 5);
    cv::Scalar mean, stddev;
    cv::meanStdDev ( inp, mean, stddev, inp!=10 );
    //std::cout << "MEAN: " << stddev << std::endl; 
    //rs_input[abs(rs_input - np.mean(rs_input)) > 2.5 * np.std(rs_input)] = 1
    
    inp.setTo(1, cv::abs(inp-mean[0]) > 2.5 * stddev[0]);

    inp.setTo(0, inp==10);
    cv::minMaxLoc(cv::SparseMat(inp) , &min, &max);
    //inp.setTo(1, inp==1);

    inp = (inp - min) / (max-min) * 2 - 1;
    inp.setTo(1, inp>1);
    inp.setTo(1, inp<-1);
    //cv::imwrite("/home/monocle/Dev/Monocle/Monocle/build/tmp/after.png", (inp+1)/2*255);
    //cv::minMaxLoc(cv::SparseMat(inp) , &min, &max);
    //std::cout << min << "  AFTER  " << max << std::endl;

    torch::Tensor t = torch::from_blob(inp.data, {1, 1, 128, 128}, at::kFloat);  
    std::vector<torch::jit::IValue> input;
    input.emplace_back(t.to(at::kCUDA));

    /* Get output params */
    auto output = m_module->forward(input).toTensor().to(at::kCPU);
    pred = std::vector<float>(output.data<float>(), output.data<float>() + output.numel());
    float scale_x = 128.f / bbox_width;
    float scale_y = 128.f / bbox_width;

		pred[0] = (pred[0]+1.f)/2.f*128.f / scale_x + bbox_x; 
		pred[1] = (1.f-pred[1])/2.f*128.f / scale_y + bbox_y; 
		pred[2] = (pred[2]+1.f)/2.f * (max-min) + min;
    
    m_ready = true;
    
    return 0;
}


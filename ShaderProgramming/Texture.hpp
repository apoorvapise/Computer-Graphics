#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = 0;
        auto v_img = 0;
        if (u < 0)
            u_img = 0;
        else if (u >= 1)
            u_img = width - 1;
        else
            u_img = u * width; 
 
        if (v < 0)
            v_img = height - 1;
        else if (v >= 1)
            v_img = 0;
        else
            v_img = (1 - v) * height;  

        //auto u_img = u * width;
        //auto v_img = height - 1;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H

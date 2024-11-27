#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "opencv2/opencv.hpp"

class ImageConversionNode : public rclcpp::Node
{
public:
    ImageConversionNode() : Node("image_conversion_node"), mode_(2)
    {
        image_transport::ImageTransport it(this);
        image_sub_ = it.subscribe("/usb_cam/image_raw", 10, &ImageConversionNode::imageCallback, this);
        image_pub_ = it.advertise("/converted_image", 10);

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_conversion_mode",
            std::bind(&ImageConversionNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (mode_ == 1)
        {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
        }

        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void setModeCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        mode_ = request->data ? 1 : 2;
        response->success = true;
        response->message = mode_ == 1 ? "Greyscale mode set" : "Color mode set";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    int mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}


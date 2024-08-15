#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using sensor_msgs::msg::Image;
using depthai_ros_msgs::msg::SpatialDetectionArray;

typedef message_filters::sync_policies::ApproximateTime<Image, SpatialDetectionArray> SyncPolicy;

class BBoxImagePublisher : public rclcpp::Node
{
    public:
        BBoxImagePublisher(): Node("bbox_image_publisher")
        {
	    image_sub_.subscribe(this, "/color/image");
            detections_sub_.subscribe(this, "/ei_yolov5/spatial_detections");

            synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(5), image_sub_, detections_sub_);
	    synchronizer_->registerCallback(std::bind(&BBoxImagePublisher::on_detections, 
				    this, std::placeholders::_1, std::placeholders::_2));
        }

	void setPublisher(image_transport::Publisher pub)
	{
	    bbox_img_pub_ = pub; 
	}
    private:
	void addTextToFrame(cv::Mat& frame, const std::string& text, int x, int y) {
            auto white = cv::Scalar(255, 255, 255);
            auto black = cv::Scalar(0, 0, 0);
            cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
            cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        }

	void on_detections(const Image::ConstSharedPtr& image_msg, const SpatialDetectionArray::ConstSharedPtr& detections_msg) 
	{
	    cv::Mat img = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
	    auto blue = cv::Scalar(255, 0, 0);
	    std::vector<std::string> labelMap = {"penguin", "pig"}; 

	    for(auto& detection : detections_msg->detections) {
		auto bbox = detection.bbox;
                auto x1 = bbox.center.position.x - bbox.size_x / 2.0;
                auto x2 = bbox.center.position.x + bbox.size_x / 2.0;
                auto y1 = bbox.center.position.y - bbox.size_y / 2.0;
                auto y2 = bbox.center.position.y + bbox.size_y / 2.0;

                cv::rectangle(img, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);

		auto result = detection.results[0];
                auto label = labelMap[std::stoi(result.class_id)];
                addTextToFrame(img, label, x1 + 10, y1 + 10);

		auto position = detection.position;
                std::stringstream depthX;
                depthX << "X: " << position.x << " mm";
                addTextToFrame(img, depthX.str(), x1 + 10, y1 + 60);

                std::stringstream depthY;
                depthY << "Y: " << position.y << " mm";
                addTextToFrame(img, depthY.str(), x1 + 10, y1 + 75);

                std::stringstream depthZ;
                depthZ << "Z: " << position.z << " mm";
                addTextToFrame(img, depthZ.str(), x1 + 10, y1 + 90);
	    }

	    Image outMsg;
            cv_bridge::CvImage(image_msg->header, image_msg->encoding, img).toImageMsg(outMsg);
	    bbox_img_pub_.publish(outMsg);
        }

        //rclcpp::Publisher<Image>::SharedPtr bbox_img_pub_;
	image_transport::Publisher bbox_img_pub_;
	message_filters::Subscriber<Image> image_sub_;
        message_filters::Subscriber<SpatialDetectionArray> detections_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BBoxImagePublisher>();
    image_transport::ImageTransport img_transport_(node);
    node->setPublisher(img_transport_.advertise("/ei_yolov5/bbox_image", 1));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

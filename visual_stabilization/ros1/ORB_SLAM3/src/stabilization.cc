#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "System.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <chrono>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

ORB_SLAM3::System* slam;
Sophus::SE3f checkpoint_pose;
std::atomic<bool> has_checkpoint{false};
std::atomic<bool> should_save_pose{false};
std::atomic<bool> shoudl_go_to_pose{false};
double movement_k = 5.0;
double rotation_k = 0.25;

ros::Publisher velocity_pub;

void movementCallback(const std_msgs::Float64::ConstPtr& msg) {
    movement_k = msg->data;
}

void rotationCallback(const std_msgs::Float64::ConstPtr& msg) {
    rotation_k = msg->data;
}

void holdCallback(const std_msgs::Bool::ConstPtr& msg) {
    shoudl_go_to_pose = msg->data;
}

void saveCurrentPoseCallback(const std_msgs::Empty::ConstPtr& msg) {
    should_save_pose = true;
}

void saveCheckpointPose(const Sophus::SE3f& pose) {
    checkpoint_pose = pose;
    should_save_pose = false;
    has_checkpoint = true;

    Eigen::Vector3f translation = checkpoint_pose.translation();
    std::cout << "Saved Pose:\n";
    std::cout << "x: " << translation.x() << ", y: " << translation.y() << ", z: " << translation.z() << std::endl;
}

struct RollPitchYaw {
    float roll;
    float pitch;
    float yaw;
};

geometry_msgs::TwistStamped offsetToDroneVelocity(
    const Eigen::Vector3f& offset,
    const Eigen::Matrix3f& rotation_offset,
    float movement_coefficient,
    float rotation_coefficient
) {
    geometry_msgs::TwistStamped cmd_vel_stamped;
    cmd_vel_stamped.header.stamp = ros::Time::now();
    cmd_vel_stamped.header.frame_id = "base_link";

    float pitch = std::asin(-rotation_offset(2, 0));
    float yaw = std::atan2(rotation_offset(1, 0), rotation_offset(0, 0));
    float roll = std::atan2(rotation_offset(2, 1), rotation_offset(2, 2));
    std::cout << "pitch: " << pitch << ", yaw: " << yaw << ", roll: " << roll << std::endl;


    cmd_vel_stamped.twist.linear.x = offset.y() * movement_coefficient;
    cmd_vel_stamped.twist.linear.y = offset.x() * movement_coefficient;
    cmd_vel_stamped.twist.linear.z = -offset.z() * movement_coefficient;

    cmd_vel_stamped.twist.angular.x = - roll * rotation_coefficient;
    cmd_vel_stamped.twist.angular.y = - pitch * rotation_coefficient;
    cmd_vel_stamped.twist.angular.z = - yaw * rotation_coefficient;

    return cmd_vel_stamped;
}

void processImage(const cv_bridge::CvImageConstPtr& cv_ptr)
{
    Sophus::SE3f pose = slam->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    if (pose.translation().norm() == 0) return;
    
    if (should_save_pose) {
        saveCheckpointPose(pose);
    }

    if (!has_checkpoint) return;
    if (!shoudl_go_to_pose) return;

    Eigen::Vector3f offset = pose.translation() - checkpoint_pose.translation();
    Eigen::Matrix3f rotation_offset = pose.rotationMatrix() * checkpoint_pose.rotationMatrix().transpose();

    std::cout << "dx: " << offset.x() << ", dy: " << offset.y() << ", dz: " << offset.z() << std::endl;
    std::cout << "norm" << offset.norm() << "\n";

    if (offset.norm() < 0.002) return;

    geometry_msgs::TwistStamped cmd_vel_stamped = offsetToDroneVelocity(offset, rotation_offset, movement_k, rotation_k);
    velocity_pub.publish(cmd_vel_stamped);
}

void rawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        processImage(cv_bridge::toCvShare(msg));
    } catch (cv_bridge::Exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try {
        cv::Mat decoded_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->image = decoded_image;
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        cv_ptr->header = msg->header;

        processImage(cv_ptr);
    } catch (const std::exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

std::string getValidatedImageFormat(const std::string& image_format)
{
    bool is_valid = (image_format == "raw") || (image_format == "compressed_jpeg");
    if (!is_valid)
    {
        cerr << endl << "Error: Invalid image format '" << image_format << "'. Allowed formats: 'raw', 'compressed_jpeg'" << endl;
        ros::shutdown();
        exit(1);
    }

    return image_format;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb_slam3_stabilization");
    ros::NodeHandle nh;

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stabilization $camera_topic_name $image_format $path_to_vocabulary $path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }   

    std::string camera_topic_name = argv[1];
    std::string image_format = getValidatedImageFormat(argv[2]);
    std::string vocab_path = argv[3];
    std::string camera_calibration_path = argv[4];

    slam = new ORB_SLAM3::System(vocab_path, camera_calibration_path, ORB_SLAM3::System::MONOCULAR, true);

    velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber sub_movement_k = nh.subscribe("/stabilization/movement_k", 1, movementCallback);
    ros::Subscriber sub_ratoration_k = nh.subscribe("/stabilization/rotation_k", 1, rotationCallback);
    ros::Subscriber sub_hold = nh.subscribe("/stabilization/enable_hold", 1, holdCallback);
    ros::Subscriber sub_save_pose = nh.subscribe("/stabilization/save_current_pose", 1, saveCurrentPoseCallback);
    
    ros::Subscriber sub_image;
    if (image_format == "raw")
    {
        sub_image = nh.subscribe<sensor_msgs::Image>(camera_topic_name, 1, rawImageCallback);
    }
    else if (image_format == "compressed_jpeg")
    {
        sub_image = nh.subscribe(camera_topic_name, 1, compressedImageCallback);
    }

    ros::spin();
    
    slam->Shutdown();
    delete slam;
    return 0;
}

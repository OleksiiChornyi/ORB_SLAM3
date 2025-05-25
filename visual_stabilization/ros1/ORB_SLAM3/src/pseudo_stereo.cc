#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

std::atomic<bool> use_left{false};
std::atomic<int> processed_images{0};
sensor_msgs::CompressedImageConstPtr chooseImage(
    const sensor_msgs::CompressedImageConstPtr& left,
    const sensor_msgs::CompressedImageConstPtr& right
) {
    processed_images = processed_images + 1;
    if (processed_images > 15) {
        processed_images = 1;
        use_left = !use_left;
    }

    return use_left ? left : right;
}

void compressedImageCallback(
    const sensor_msgs::CompressedImageConstPtr& left,
    const sensor_msgs::CompressedImageConstPtr& right
)
{
    sensor_msgs::CompressedImageConstPtr raw_image = chooseImage(left, right);
    try {
        cv::Mat decoded_image = cv::imdecode(cv::Mat(raw_image->data), cv::IMREAD_COLOR);

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->image = decoded_image;
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        cv_ptr->header = raw_image->header;

        processImage(cv_ptr);
    } catch (const std::exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb_slam3_stabilization");
    ros::NodeHandle nh;

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stabilization $camera_topic_name_left $camera_topic_name_right $path_to_vocabulary $path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }   

    std::string camera_topic_name_left = argv[1];
    std::string camera_topic_name_right = argv[2];
    std::string vocab_path = argv[3];
    std::string camera_calibration_path = argv[4];

    slam = new ORB_SLAM3::System(vocab_path, camera_calibration_path, ORB_SLAM3::System::MONOCULAR, true);

    velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber sub_movement_k = nh.subscribe("/stabilization/movement_k", 1, movementCallback);
    ros::Subscriber sub_ratoration_k = nh.subscribe("/stabilization/rotation_k", 1, rotationCallback);
    ros::Subscriber sub_hold = nh.subscribe("/stabilization/enable_hold", 1, holdCallback);
    ros::Subscriber sub_save_pose = nh.subscribe("/stabilization/save_current_pose", 1, saveCurrentPoseCallback);

    message_filters::Subscriber<sensor_msgs::CompressedImage> left_sub(nh, camera_topic_name_left, 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> right_sub(nh, camera_topic_name_right, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&compressedImageCallback, _1, _2));

    ros::spin();

    slam->Shutdown();
    delete slam;
    return 0;
}

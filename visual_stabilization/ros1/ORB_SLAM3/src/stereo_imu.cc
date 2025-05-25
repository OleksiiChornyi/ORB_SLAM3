#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
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

std::queue<sensor_msgs::ImuConstPtr> imuBuf;
std::mutex imuBufMutex;

std::queue<cv_bridge::CvImageConstPtr> imgBuf_left;
std::mutex imgBufMutex_left;

std::queue<cv_bridge::CvImageConstPtr> imgBuf_right;
std::mutex imgBufMutex_right;

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

void processDataAndStibilize(
    const cv_bridge::CvImageConstPtr& cv_img_left,
    const cv_bridge::CvImageConstPtr& cv_img_right,
    const std::vector<ORB_SLAM3::IMU::Point>& imu_vector
) {
    Sophus::SE3f pose = slam->TrackStereo(cv_img_left->image, cv_img_right->image,cv_img_left->header.stamp.toSec(), imu_vector);
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

void addImuToBuffer(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imuBufMutex.lock();
    imuBuf.push(imu_msg);
    imuBufMutex.unlock();
}

void addImageToBuffer_left(const cv_bridge::CvImageConstPtr& cv_ptr)
{
    imgBufMutex_left.lock();
    imgBuf_left.push(cv_ptr);
    imgBufMutex_left.unlock();
}

void addImageToBuffer_right(const cv_bridge::CvImageConstPtr& cv_ptr)
{
    imgBufMutex_right.lock();
    imgBuf_right.push(cv_ptr);
    imgBufMutex_right.unlock();
}

void rawImageCallback_left(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        addImageToBuffer_left(cv_bridge::toCvShare(msg));
    } catch (cv_bridge::Exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

void rawImageCallback_right(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        addImageToBuffer_right(cv_bridge::toCvShare(msg));
    } catch (cv_bridge::Exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

void compressedImageCallback_left(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try {
        cv::Mat decoded_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->image = decoded_image;
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        cv_ptr->header = msg->header;

        addImageToBuffer_left(cv_ptr);
    } catch (const std::exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

void compressedImageCallback_right(const sensor_msgs::CompressedImageConstPtr& msg)
{
    try {
        cv::Mat decoded_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->image = decoded_image;
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
        cv_ptr->header = msg->header;

        addImageToBuffer_right(cv_ptr);
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

std::vector<ORB_SLAM3::IMU::Point> extractImuPointsUpTo(const ros::Time& image_timestamp) {
    std::vector<ORB_SLAM3::IMU::Point> imu_vector;

    std::lock_guard<std::mutex> lock(imuBufMutex);
    while (!imuBuf.empty()) {
        auto imu_msg = imuBuf.front();
        if (imu_msg->header.stamp > image_timestamp) {
            break;
        }

        cv::Point3f acc(
            imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y,
            imu_msg->linear_acceleration.z
        );
        cv::Point3f gyr(
            imu_msg->angular_velocity.x,
            imu_msg->angular_velocity.y,
            imu_msg->angular_velocity.z
        );
        double imu_timestamp = imu_msg->header.stamp.toSec();

        imu_vector.push_back(ORB_SLAM3::IMU::Point(acc, gyr, imu_timestamp));
        imuBuf.pop();
    }

    return imu_vector;
}

cv_bridge::CvImageConstPtr extractOldestImage_left() {
    while (1) {
        if (imgBuf_left.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
    
        imgBufMutex_left.lock();
        cv_bridge::CvImageConstPtr cv_img = imgBuf_left.front();
        imgBuf_left.pop();
        imgBufMutex_left.unlock();
    
        return cv_img;
    }
}

cv_bridge::CvImageConstPtr extractOldestImage_right() {
    while (1) {
        if (imgBuf_right.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
    
        imgBufMutex_right.lock();
        cv_bridge::CvImageConstPtr cv_img = imgBuf_right.front();
        imgBuf_right.pop();
        imgBufMutex_right.unlock();
    
        return cv_img;
    }
}

void doStabilizationSync() {
    while (1) {        
        cv_bridge::CvImageConstPtr cv_img_left = extractOldestImage_left();
        cv_bridge::CvImageConstPtr cv_img_right = extractOldestImage_right();

        std::vector<ORB_SLAM3::IMU::Point> imu_points = extractImuPointsUpTo(cv_img_left->header.stamp);
        processDataAndStibilize(cv_img_left, cv_img_right, imu_points);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orb_slam3_stabilization");
    ros::NodeHandle nh;

    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stabilization $camera_topic_name_left $camera_topic_name_right $image_format $path_to_vocabulary $path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }   

    std::string camera_topic_name_left = argv[1];
    std::string camera_topic_name_right = argv[2];
    std::string image_format = getValidatedImageFormat(argv[3]);
    std::string vocab_path = argv[4];
    std::string camera_calibration_path = argv[5];

    slam = new ORB_SLAM3::System(vocab_path, camera_calibration_path, ORB_SLAM3::System::STEREO, true);

    velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber sub_movement_k = nh.subscribe("/stabilization/movement_k", 1, movementCallback);
    ros::Subscriber sub_ratoration_k = nh.subscribe("/stabilization/rotation_k", 1, rotationCallback);
    ros::Subscriber sub_hold = nh.subscribe("/stabilization/enable_hold", 1, holdCallback);
    ros::Subscriber sub_save_pose = nh.subscribe("/stabilization/save_current_pose", 1, saveCurrentPoseCallback);
    ros::Subscriber sub_imu = nh.subscribe("/mavros/imu/data", 1000, addImuToBuffer);
    
    ros::Subscriber sub_image_left;
    ros::Subscriber sub_image_right;
    if (image_format == "raw")
    {
        sub_image_left = nh.subscribe<sensor_msgs::Image>(camera_topic_name_left, 100, rawImageCallback_left);
        sub_image_right = nh.subscribe<sensor_msgs::Image>(camera_topic_name_right, 100, rawImageCallback_right);
    }
    else if (image_format == "compressed_jpeg")
    {
        sub_image_left = nh.subscribe(camera_topic_name_left, 100, compressedImageCallback_left);
        sub_image_right = nh.subscribe(camera_topic_name_right, 100, compressedImageCallback_right);
    }

    std::thread myThread(doStabilizationSync);
    ros::spin();
    
    slam->Shutdown();
    delete slam;
    return 0;
}

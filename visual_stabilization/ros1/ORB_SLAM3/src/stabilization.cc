#include "System.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ORB_SLAM3::System* slam;
ros::Publisher vision_pose_pub;

geometry_msgs::PoseStamped to_trasformed_orientation(
    const Sophus::SE3f& pose,
    const ros::Time& pose_timestamp
) {
    float roll_cam = 0.0;           // Camera rotation around X (rad)
    float pitch_cam = 0.0;          // Camera rotation around Y (rad)
    float yaw_cam = 1.5707963f;    // Camera rotation around Z (rad)
    float scaleX = 8;
    float scaleY = 8;
    float scaleZ = 8;
    
    // 1. Scale coordinates to represent real world.
    Eigen::Vector3f position_orig = pose.translation();
    Eigen::Matrix3f rotation_orig = pose.rotationMatrix();
    float gamma_world = std::atan2(rotation_orig(1, 0), rotation_orig(0, 0));
    
    // 2. Rotation from original world frame to world frame with Y forward
    // We use gamma from SLAM yaw, as SLAM switch x and y pose based on its yaw orientation.
    // Shortly we reset coordinates to direction used on yaw 0, when SLAM was initialized.
    Eigen::Vector3f position_body;
    position_body.x() =  std::cos(gamma_world) * position_orig.x() + std::sin(gamma_world) * position_orig.y();
    position_body.y() = -std::sin(gamma_world) * position_orig.x() + std::cos(gamma_world) * position_orig.y();
    position_body.z() = (position_orig.z() > 0.0f) ? position_orig.z() + 20.0f : position_orig.z();

    // 3. Camera to body frame rotation
    Eigen::Quaternionf quat_camera = Eigen::Quaternionf(rotation_orig);
    Eigen::Quaternionf quat_camera_x = Eigen::Quaternionf(Eigen::AngleAxisf(roll_cam,  Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf quat_camera_y = Eigen::Quaternionf(Eigen::AngleAxisf(pitch_cam, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf quat_camera_z = Eigen::Quaternionf(Eigen::AngleAxisf(yaw_cam,   Eigen::Vector3f::UnitZ()));

    Eigen::Quaternionf quat_body = quat_camera * quat_camera_x * quat_camera_y * quat_camera_z;
    quat_body.normalize();

    // 4. Convert left-handed rotation to right-handed
    Eigen::Quaternionf quat_right_hand = Eigen::Quaternionf(quat_body.w(),  - quat_body.x(), - quat_body.y(), quat_body.z());
    quat_right_hand.normalize();

    // Output
    geometry_msgs::PoseStamped msg_body_pose; 
    msg_body_pose.header.stamp = pose_timestamp;
    msg_body_pose.header.frame_id = "map";
    // Convet to WNU to ENU by negating some coordinates
    msg_body_pose.pose.position.x = -position_body.x() * scaleX;
    msg_body_pose.pose.position.y = position_body.y() * scaleY;
    msg_body_pose.pose.position.z = position_body.z() * scaleZ;
    msg_body_pose.pose.orientation.x = -quat_right_hand.y();
    msg_body_pose.pose.orientation.y = -quat_right_hand.x(); 
    msg_body_pose.pose.orientation.z = quat_right_hand.z();
    msg_body_pose.pose.orientation.w = quat_right_hand.w();
    return msg_body_pose;
}



void processImage(const cv_bridge::CvImageConstPtr& cv_ptr)
{
    Sophus::SE3f pose = slam->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    vision_pose_pub.publish(to_trasformed_orientation(pose, cv_ptr->header.stamp));
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

    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

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

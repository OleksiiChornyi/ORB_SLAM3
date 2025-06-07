#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "System.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <atomic>

using sensor_msgs::msg::CompressedImage;
using std::placeholders::_1;

class SLAMStabilizer : public rclcpp::Node {
public:
    SLAMStabilizer(
        const std::string& camera_topic_name_left,
        const std::string& camera_topic_name_right,
        const std::string& imu_topic_name,
        const std::string& vocab_path,
        const std::string& camera_calibration_path
    ): Node("orb_slam3_stabilization") {

        slam = new ORB_SLAM3::System(vocab_path, camera_calibration_path, ORB_SLAM3::System::STEREO, true);

        velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        sub_movement_k = this->create_subscription<std_msgs::msg::Float64>(
            "/stabilization/movement_k", 1, std::bind(&SLAMStabilizer::movementCallback, this, _1)
        );

        sub_rotation_k = this->create_subscription<std_msgs::msg::Float64>(
            "/stabilization/rotation_k", 1, std::bind(&SLAMStabilizer::rotationCallback, this, _1)
        );

        sub_hold = this->create_subscription<std_msgs::msg::Bool>(
            "/stabilization/enable_hold", 1, std::bind(&SLAMStabilizer::holdCallback, this, _1)
        );

        sub_save_pose = this->create_subscription<std_msgs::msg::Empty>(
            "/stabilization/save_current_pose", 1, std::bind(&SLAMStabilizer::saveCurrentPoseCallback, this, _1)
        );

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_name, 1000, std::bind(&SLAMStabilizer::trackImuCallback, this, _1)
        );

        img_left_sub.subscribe(this, camera_topic_name_left);
        img_right_sub.subscribe(this, camera_topic_name_right);
        image_sync_sub = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), img_left_sub, img_right_sub);
        image_sync_sub->registerCallback(&SLAMStabilizer::compressedImageCallback, this);

        stabilization_thread = std::thread(&SLAMStabilizer::doStabilizationSync, this);
    }

    ~SLAMStabilizer() {
        slam->Shutdown();
        delete slam;
    }

private:
    ORB_SLAM3::System* slam;
    Sophus::SE3f checkpoint_pose;
    std::atomic<bool> has_checkpoint{false};
    std::atomic<bool> should_save_pose{false};
    std::atomic<bool> should_go_to_pose{false};

    std::atomic<int> processed_images{0};

    double movement_k = 5.0;
    double rotation_k = 0.25;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
    std::mutex imu_buf_mutex;

    std::queue<cv_bridge::CvImageConstPtr> img_left_buf;
    std::queue<cv_bridge::CvImageConstPtr> img_right_buf;
    std::mutex img_buf_mutex;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_movement_k;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_rotation_k;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_hold;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_save_pose;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> img_left_sub;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> img_right_sub;

    typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> image_sync_sub;

    std::thread stabilization_thread;

    void movementCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        std::cout << msg->data << "movementCallback:\n";
        movement_k = msg->data;
    }

    void rotationCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        rotation_k = msg->data;
    }

    void holdCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        should_go_to_pose = msg->data;
    }

    void saveCurrentPoseCallback(const std_msgs::msg::Empty::SharedPtr) {
        should_save_pose = true;
    }

    void saveCheckpointPose(const Sophus::SE3f& pose) {
        checkpoint_pose = pose;
        has_checkpoint = true;
        should_save_pose = false;

        Eigen::Vector3f translation = checkpoint_pose.translation();
        std::cout << "Saved Pose:\n";
        std::cout << "x: " << translation.x() << ", y: " << translation.y() << ", z: " << translation.z() << std::endl;
    }

    geometry_msgs::msg::TwistStamped offsetToDroneVelocity(
        const Eigen::Vector3f& offset,
        const Eigen::Matrix3f& rot_offset
    ) {
        geometry_msgs::msg::TwistStamped velocity_command;
        velocity_command.header.stamp = this->now();
        velocity_command.header.frame_id = "base_link";

        float pitch = std::asin(-rot_offset(2, 0));
        float yaw = std::atan2(rot_offset(1, 0), rot_offset(0, 0));
        float roll = std::atan2(rot_offset(2, 1), rot_offset(2, 2));
        std::cout << "pitch: " << pitch << ", yaw: " << yaw << ", roll: " << roll << std::endl;

        velocity_command.twist.linear.x = offset.y() * movement_k;
        velocity_command.twist.linear.y = offset.x() * movement_k;
        velocity_command.twist.linear.z = -offset.z() * movement_k;
        velocity_command.twist.angular.x = -roll * rotation_k;
        velocity_command.twist.angular.y = -pitch * rotation_k;
        velocity_command.twist.angular.z = -yaw * rotation_k;
        return velocity_command;
    }

    void trackImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        imu_buf_mutex.lock();
        imu_buf.push(imu_msg);
        imu_buf_mutex.unlock();
    }

    void compressedImageCallback(
        const sensor_msgs::msg::CompressedImage::SharedPtr left,
        const sensor_msgs::msg::CompressedImage::SharedPtr right
    ) {
        try {
            cv::Mat decoded_image_left = cv::imdecode(cv::Mat(left->data), cv::IMREAD_COLOR);

            cv_bridge::CvImagePtr cv_ptr_left(new cv_bridge::CvImage);
            cv_ptr_left->image = decoded_image_left;
            cv_ptr_left->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr_left->header = left->header;

            cv::Mat decoded_image_right = cv::imdecode(cv::Mat(right->data), cv::IMREAD_COLOR);

            cv_bridge::CvImagePtr cv_ptr_right(new cv_bridge::CvImage);
            cv_ptr_right->image = decoded_image_right;
            cv_ptr_right->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr_right->header = right->header;

            img_buf_mutex.lock();
            img_left_buf.push(cv_ptr_left);
            img_right_buf.push(cv_ptr_right);
            img_buf_mutex.unlock();
        } catch (const std::exception& e) {
            throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
        }
    }

    std::pair<cv_bridge::CvImageConstPtr, cv_bridge::CvImageConstPtr> extractOldestImagePair() {
        while (1) {
            if (img_left_buf.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        
            img_buf_mutex.lock();

            cv_bridge::CvImageConstPtr left = img_left_buf.front();
            img_left_buf.pop();
            cv_bridge::CvImageConstPtr right = img_right_buf.front();
            img_right_buf.pop();

            img_buf_mutex.unlock();
        
            return { left, right };
        }
    }

    std::vector<ORB_SLAM3::IMU::Point> extractImuPointsUpTo(const rclcpp::Time& image_timestamp) {
        std::vector<ORB_SLAM3::IMU::Point> imu_vector;

        std::lock_guard<std::mutex> lock(imu_buf_mutex);
        while (!imu_buf.empty()) {
            auto imu_msg = imu_buf.front();
            auto imu_timestamp =  rclcpp::Time(imu_msg->header.stamp);
            if (imu_timestamp > image_timestamp) {
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

            std::cout << "Got IMU:" << imu_timestamp.seconds() << "\n";
            imu_vector.push_back(ORB_SLAM3::IMU::Point(acc, gyr, imu_timestamp.seconds()));
            imu_buf.pop();
        }

        return imu_vector;
    }

    void processDataAndStibilize(
        const cv_bridge::CvImageConstPtr& left,
        const cv_bridge::CvImageConstPtr& right,
        const std::vector<ORB_SLAM3::IMU::Point>& imu_vector
    ) {
        processed_images = processed_images + 1;

        double timestamp = rclcpp::Time(left->header.stamp).seconds();
        Sophus::SE3f pose = slam->TrackStereo(left->image,right->image,timestamp, imu_vector);

        std::cout << "Counter:" << processed_images << "\n";

        if (pose.translation().norm() == 0) return;

        if (should_save_pose) {
            saveCheckpointPose(pose);
        }

        if (!has_checkpoint) return;
        if (!should_go_to_pose) return;

        Eigen::Vector3f offset = pose.translation() - checkpoint_pose.translation();
        Eigen::Matrix3f rotation_offset = pose.rotationMatrix() * checkpoint_pose.rotationMatrix().transpose();

        std::cout << "dx: " << offset.x() << ", dy: " << offset.y() << ", dz: " << offset.z() << std::endl;
        std::cout << "norm" << offset.norm() << "\n";

        if (offset.norm() < 0.002) return;

        geometry_msgs::msg::TwistStamped cmd_vel_stamped = offsetToDroneVelocity(offset, rotation_offset);
        velocity_pub->publish(cmd_vel_stamped);
    }

    void doStabilizationSync() {
        while (1) {        
            auto [image_left, image_right] = extractOldestImagePair();
            std::cout << "Got images:" << rclcpp::Time(image_left->header.stamp).seconds() << "\n";
            std::vector<ORB_SLAM3::IMU::Point> imu_points = extractImuPointsUpTo(image_left->header.stamp);

            processDataAndStibilize(image_left, image_right, imu_points);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if(argc != 6) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stabilization $camera_topic_name_left $camera_topic_name_right $imu_topic_name $path_to_vocabulary $path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }   

    std::string camera_topic_name_left = argv[1];
    std::string camera_topic_name_right = argv[2];
    std::string imu_topic_name = argv[3];
    std::string vocab_path = argv[4];
    std::string camera_calibration_path = argv[5];

    auto node = std::make_shared<SLAMStabilizer>(
        camera_topic_name_left,
        camera_topic_name_right,
        imu_topic_name,
        vocab_path,
        camera_calibration_path
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

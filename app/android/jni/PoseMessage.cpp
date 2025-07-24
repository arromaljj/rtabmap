#include "PoseMessage.h"
#include <iostream>
#include <sstream>
#include <ctime>

// For iOS, we'll use a simpler logging system
#define LOG_TAG "PoseMessage"
#define LOGI(msg) std::cout << "[INFO][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGW(msg) std::cout << "[WARN][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGE(msg) std::cout << "[ERROR][" << LOG_TAG << "] " << (msg) << std::endl

PoseMessage::PoseMessage(const std::string& topic, const std::string& frameId)
    : Message(topic, frameId)
{
}

PoseMessage::~PoseMessage()
{
}

void PoseMessage::setPose(const rtabmap::Transform& pose)
{
    // Extract position
    x_ = pose.x();
    y_ = pose.y();
    z_ = pose.z();
    
    // Extract orientation as quaternion
    Eigen::Quaternionf q = pose.getQuaternionf();
    qx_ = q.x();
    qy_ = q.y();
    qz_ = q.z();
    qw_ = q.w();
}

void PoseMessage::setPose(float x, float y, float z, float qx, float qy, float qz, float qw)
{
    x_ = x;
    y_ = y;
    z_ = z;
    qx_ = qx;
    qy_ = qy;
    qz_ = qz;
    qw_ = qw;
}

std::string PoseMessage::createPoseJsonPayload() const
{
    std::stringstream ss;

    // Get timestamp
    uint32_t sec = std::time(nullptr);

    // Create JSON payload (geometry_msgs/PoseStamped format)
    ss << "{";
    ss << "\"header\":{";
    ss << "\"stamp\":{\"sec\":" << sec << ",\"nanosec\":0},";
    ss << "\"frame_id\":\"" << frameId_ << "\"}";
    ss << ",";
    ss << "\"pose\":{";
    ss << "\"position\":{\"x\":" << x_ << ",\"y\":" << y_ << ",\"z\":" << z_ << "},";
    ss << "\"orientation\":{\"x\":" << qx_ << ",\"y\":" << qy_ << ",\"z\":" << qz_ << ",\"w\":" << qw_ << "}";
    ss << "}";
    ss << "}";

    return ss.str();
}

std::string PoseMessage::createTfJsonPayload(const std::string& childFrameId) const
{
    std::stringstream ss;

    // Get timestamp
    uint32_t sec = std::time(nullptr);

    // Create JSON payload (tf2_msgs/TFMessage containing one TransformStamped)
    ss << "{";
    ss << "\"transforms\":[{";
    ss << "\"header\":{";
    ss << "\"stamp\":{\"sec\":" << sec << ",\"nanosec\":0},";
    ss << "\"frame_id\":\"" << frameId_ << "\"}";
    ss << ",";
    ss << "\"child_frame_id\":\"" << childFrameId << "\",";
    ss << "\"transform\":{";
    ss << "\"translation\":{\"x\":" << x_ << ",\"y\":" << y_ << ",\"z\":" << z_ << "},";
    ss << "\"rotation\":{\"x\":" << qx_ << ",\"y\":" << qy_ << ",\"z\":" << qz_ << ",\"w\":" << qw_ << "}";
    ss << "}";
    ss << "}]";
    ss << "}";

    return ss.str();
}

std::string PoseMessage::createOdomJsonPayload(const std::string& childFrameId) const
{
    std::stringstream ss;

    // Get timestamp
    uint32_t sec = std::time(nullptr);

    // Create JSON payload (nav_msgs/Odometry format)
    ss << "{";
    ss << "\"header\":{";
    ss << "\"stamp\":{\"sec\":" << sec << ",\"nanosec\":0},";
    ss << "\"frame_id\":\"" << frameId_ << "\"}";
    ss << ",";
    ss << "\"child_frame_id\":\"" << childFrameId << "\",";
    ss << "\"pose\":{";
    ss << "\"pose\":{";
    ss << "\"position\":{\"x\":" << x_ << ",\"y\":" << y_ << ",\"z\":" << z_ << "},";
    ss << "\"orientation\":{\"x\":" << qx_ << ",\"y\":" << qy_ << ",\"z\":" << qz_ << ",\"w\":" << qw_ << "}";
    ss << "},";
    ss << "\"covariance\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]";
    ss << "},";
    ss << "\"twist\":{";
    ss << "\"twist\":{";
    ss << "\"linear\":{\"x\":0,\"y\":0,\"z\":0},";
    ss << "\"angular\":{\"x\":0,\"y\":0,\"z\":0}";
    ss << "},";
    ss << "\"covariance\":[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]";
    ss << "}";
    ss << "}";

    return ss.str();
} 
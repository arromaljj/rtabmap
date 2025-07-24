#pragma once
#include "Message.h"
#include <rtabmap/core/Transform.h>
#include <Eigen/Core>

class PoseMessage : public Message {
public:
    PoseMessage(const std::string& topic = "rtabmap/pose", const std::string& frameId = "map");
    ~PoseMessage();

    // Set pose from Transform
    void setPose(const rtabmap::Transform& pose);
    
    // Set direct values
    void setPose(float x, float y, float z, float qx, float qy, float qz, float qw);
    
    // Create JSON payload for pose (geometry_msgs/PoseStamped format)
    virtual std::string createPoseJsonPayload() const;

    // Create JSON payload for TF transform (geometry_msgs/TransformStamped format within TFMessage)
    std::string createTfJsonPayload(const std::string& childFrameId = "base_link") const;

    // Create JSON payload for odometry (nav_msgs/Odometry format)
    std::string createOdomJsonPayload(const std::string& childFrameId = "base_link") const;
    
    // Getters
    float getX() const { return x_; }
    float getY() const { return y_; }
    float getZ() const { return z_; }
    float getQx() const { return qx_; }
    float getQy() const { return qy_; }
    float getQz() const { return qz_; }
    float getQw() const { return qw_; }
    
private:
    // Position
    float x_{0.0f};
    float y_{0.0f};
    float z_{0.0f};
    
    // Orientation (quaternion)
    float qx_{0.0f};
    float qy_{0.0f};
    float qz_{0.0f};
    float qw_{1.0f}; // Default to identity quaternion
}; 
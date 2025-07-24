#include "RTABMapPublishers.h"

// OdometryPublisher implementation
OdometryPublisher::OdometryPublisher(const rtabmap::Transform& pose, double stamp)
    : pose_(pose), stamp_(stamp) {}

OdometryPublisher& OdometryPublisher::publishToROS() {
    // To be implemented
    return *this;
}

OdometryPublisher& OdometryPublisher::publishJSON() {
    // Implementation below
    return *this;
}

Json::Value OdometryPublisher::toJSON() const {
    Json::Value root;
    root["type"] = "odometry";
    root["timestamp"] = stamp_;
    
    if(!pose_.isNull()) {
        float x, y, z, roll, pitch, yaw;
        pose_.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
        
        // Translation
        root["pose"]["translation"]["x"] = x;
        root["pose"]["translation"]["y"] = y;
        root["pose"]["translation"]["z"] = z;
        
        // Rotation in Euler angles
        root["pose"]["rotation"]["roll"] = roll;
        root["pose"]["rotation"]["pitch"] = pitch;
        root["pose"]["rotation"]["yaw"] = yaw;
        
        // Add transformation matrix
        const float* data = pose_.data();
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                root["pose"]["matrix"][i][j] = data[i*4 + j];
            }
        }
    } else {
        root["pose"]["valid"] = false;
    }
    
    return root;
}

// RGBImagePublisher implementation
RGBImagePublisher::RGBImagePublisher(const cv::Mat& rgb, double stamp)
    : rgb_(rgb), stamp_(stamp) {}

RGBImagePublisher& RGBImagePublisher::publishToROS() {
    // To be implemented
    return *this;
}

RGBImagePublisher& RGBImagePublisher::publishJSON() {
    // Implementation below
    return *this;
}

Json::Value RGBImagePublisher::toJSON() const {
    Json::Value root;
    root["type"] = "rgb_image";
    root["timestamp"] = stamp_;
    
    // Image properties
    root["properties"]["width"] = rgb_.cols;
    root["properties"]["height"] = rgb_.rows;
    root["properties"]["channels"] = rgb_.channels();
    root["properties"]["type"] = rgb_.type();
    root["properties"]["continuous"] = rgb_.isContinuous();
    root["properties"]["empty"] = rgb_.empty();
    
    // Add basic statistics if image is not empty
    if(!rgb_.empty()) {
        cv::Scalar mean, stddev;
        cv::meanStdDev(rgb_, mean, stddev);
        for(int i = 0; i < rgb_.channels(); ++i) {
            root["statistics"]["mean"][i] = mean[i];
            root["statistics"]["stddev"][i] = stddev[i];
        }
    }
    
    return root;
}

// DepthImagePublisher implementation
DepthImagePublisher::DepthImagePublisher(const cv::Mat& depth, double stamp)
    : depth_(depth), stamp_(stamp) {}

DepthImagePublisher& DepthImagePublisher::publishToROS() {
    // To be implemented
    return *this;
}

DepthImagePublisher& DepthImagePublisher::publishJSON() {
    // Implementation below
    return *this;
}

Json::Value DepthImagePublisher::toJSON() const {
    Json::Value root;
    root["type"] = "depth_image";
    root["timestamp"] = stamp_;
    
    // Image properties
    root["properties"]["width"] = depth_.cols;
    root["properties"]["height"] = depth_.rows;
    root["properties"]["type"] = depth_.type();
    root["properties"]["continuous"] = depth_.isContinuous();
    root["properties"]["empty"] = depth_.empty();
    
    // Add depth statistics if image is not empty
    if(!depth_.empty()) {
        double minVal, maxVal;
        cv::minMaxLoc(depth_, &minVal, &maxVal);
        root["statistics"]["min_depth"] = minVal;
        root["statistics"]["max_depth"] = maxVal;
        
        // Calculate number of valid depth points
        int validPoints = cv::countNonZero(depth_ > 0);
        root["statistics"]["valid_points"] = validPoints;
        root["statistics"]["valid_percentage"] = (float)validPoints / (depth_.rows * depth_.cols) * 100.0f;
    }
    
    return root;
}

// CameraInfoPublisher implementation
CameraInfoPublisher::CameraInfoPublisher(const rtabmap::CameraModel& model, double stamp, bool isDepth)
    : model_(model), stamp_(stamp), isDepth_(isDepth) {}

CameraInfoPublisher& CameraInfoPublisher::publishToROS() {
    // To be implemented
    return *this;
}

CameraInfoPublisher& CameraInfoPublisher::publishJSON() {
    // Implementation below
    return *this;
}

Json::Value CameraInfoPublisher::toJSON() const {
    Json::Value root;
    root["type"] = isDepth_ ? "depth_camera_info" : "rgb_camera_info";
    root["timestamp"] = stamp_;
    
    // Intrinsic parameters
    root["intrinsics"]["fx"] = model_.fx();
    root["intrinsics"]["fy"] = model_.fy();
    root["intrinsics"]["cx"] = model_.cx();
    root["intrinsics"]["cy"] = model_.cy();
    
    // Image dimensions
    root["dimensions"]["width"] = model_.imageWidth();
    root["dimensions"]["height"] = model_.imageHeight();
    
    // Distortion parameters
    const cv::Mat& D = model_.D();
    if(!D.empty()) {
        for(int i = 0; i < D.cols; ++i) {
            root["distortion_coefficients"][i] = D.at<double>(i);
        }
    }
    
    // Local transform if available
    rtabmap::Transform localTransform = model_.localTransform();
    if(!localTransform.isNull()) {
        float x, y, z, roll, pitch, yaw;
        localTransform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
        root["local_transform"]["translation"]["x"] = x;
        root["local_transform"]["translation"]["y"] = y;
        root["local_transform"]["translation"]["z"] = z;
        root["local_transform"]["rotation"]["roll"] = roll;
        root["local_transform"]["rotation"]["pitch"] = pitch;
        root["local_transform"]["rotation"]["yaw"] = yaw;
    }
    
    return root;
}

// PointCloudPublisher implementation
PointCloudPublisher::PointCloudPublisher(const rtabmap::LaserScan& scan, double stamp)
    : scan_(scan), stamp_(stamp) {}

PointCloudPublisher& PointCloudPublisher::publishToROS() {
    // To be implemented
    return *this;
}

PointCloudPublisher& PointCloudPublisher::publishJSON() {
    // Implementation below
    return *this;
}

Json::Value PointCloudPublisher::toJSON() const {
    Json::Value root;
    root["type"] = "point_cloud";
    root["timestamp"] = stamp_;
    
    // Basic properties
    root["properties"]["size"] = scan_.size();
    root["properties"]["format"] = scan_.format();
    root["properties"]["is_dense"] = !scan_.is2d();
    root["properties"]["max_points"] = scan_.maxPoints();
    root["properties"]["has_normals"] = scan_.hasNormals();
    root["properties"]["has_intensity"] = scan_.hasIntensity();
    root["properties"]["has_rgb"] = scan_.hasRGB();
    
    // Transform information
    rtabmap::Transform localTransform = scan_.localTransform();
    if(!localTransform.isNull()) {
        float x, y, z, roll, pitch, yaw;
        localTransform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
        root["local_transform"]["translation"]["x"] = x;
        root["local_transform"]["translation"]["y"] = y;
        root["local_transform"]["translation"]["z"] = z;
        root["local_transform"]["rotation"]["roll"] = roll;
        root["local_transform"]["rotation"]["pitch"] = pitch;
        root["local_transform"]["rotation"]["yaw"] = yaw;
    }
    
    return root;
}

// FeaturesPublisher implementation
FeaturesPublisher::FeaturesPublisher(const std::vector<cv::KeyPoint>& kpts,
                                   const std::vector<cv::Point3f>& kpts3d,
                                   double stamp)
    : kpts_(kpts), kpts3d_(kpts3d), stamp_(stamp) {}

FeaturesPublisher& FeaturesPublisher::publishToROS() {
    // To be implemented
    return *this;
}

FeaturesPublisher& FeaturesPublisher::publishJSON() {
    // Implementation below
    return *this;
}

Json::Value FeaturesPublisher::toJSON() const {
    Json::Value root;
    root["type"] = "features";
    root["timestamp"] = stamp_;
    
    // Basic statistics
    root["statistics"]["num_keypoints"] = (int)kpts_.size();
    root["statistics"]["num_3d_points"] = (int)kpts3d_.size();
    
    // Sample of keypoints (first 5)
    Json::Value keypoints;
    for(size_t i = 0; i < std::min((size_t)5, kpts_.size()); ++i) {
        Json::Value kpt;
        kpt["x"] = kpts_[i].pt.x;
        kpt["y"] = kpts_[i].pt.y;
        kpt["size"] = kpts_[i].size;
        kpt["angle"] = kpts_[i].angle;
        kpt["response"] = kpts_[i].response;
        kpt["octave"] = kpts_[i].octave;
        keypoints.append(kpt);
        
        // Add corresponding 3D point if available
        if(i < kpts3d_.size()) {
            Json::Value pt3d;
            pt3d["x"] = kpts3d_[i].x;
            pt3d["y"] = kpts3d_[i].y;
            pt3d["z"] = kpts3d_[i].z;
            root["keypoints_3d_sample"].append(pt3d);
        }
    }
    root["keypoints_sample"] = keypoints;
    
    // Additional statistics
    if(!kpts_.empty()) {
        float avgResponse = 0.0f;
        float avgSize = 0.0f;
        for(const auto& kp : kpts_) {
            avgResponse += kp.response;
            avgSize += kp.size;
        }
        root["statistics"]["average_response"] = avgResponse / kpts_.size();
        root["statistics"]["average_size"] = avgSize / kpts_.size();
    }
    
    if(!kpts3d_.empty()) {
        float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
        float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
        for(const auto& pt : kpts3d_) {
            minX = std::min(minX, pt.x);
            minY = std::min(minY, pt.y);
            minZ = std::min(minZ, pt.z);
            maxX = std::max(maxX, pt.x);
            maxY = std::max(maxY, pt.y);
            maxZ = std::max(maxZ, pt.z);
        }
        root["statistics"]["3d_bounds"]["min"]["x"] = minX;
        root["statistics"]["3d_bounds"]["min"]["y"] = minY;
        root["statistics"]["3d_bounds"]["min"]["z"] = minZ;
        root["statistics"]["3d_bounds"]["max"]["x"] = maxX;
        root["statistics"]["3d_bounds"]["max"]["y"] = maxY;
        root["statistics"]["3d_bounds"]["max"]["z"] = maxZ;
    }
    
    return root;
} 
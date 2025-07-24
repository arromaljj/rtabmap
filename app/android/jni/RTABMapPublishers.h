#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/LaserScan.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <json/json.h>

// Base publisher class
class PublisherBase {
public:
    virtual ~PublisherBase() {}
    virtual Json::Value toJSON() const = 0;
    virtual void publishToROS() = 0;
};

// Odometry publisher
class OdometryPublisher : public PublisherBase {
private:
    const rtabmap::Transform& pose_;
    double stamp_;

public:
    OdometryPublisher(const rtabmap::Transform& pose, double stamp);
    OdometryPublisher& publishToROS() override;
    OdometryPublisher& publishJSON();
    Json::Value toJSON() const override;
};

// RGB Image publisher
class RGBImagePublisher : public PublisherBase {
private:
    const cv::Mat& rgb_;
    double stamp_;

public:
    RGBImagePublisher(const cv::Mat& rgb, double stamp);
    RGBImagePublisher& publishToROS() override;
    RGBImagePublisher& publishJSON();
    Json::Value toJSON() const override;
};

// Depth Image publisher
class DepthImagePublisher : public PublisherBase {
private:
    const cv::Mat& depth_;
    double stamp_;

public:
    DepthImagePublisher(const cv::Mat& depth, double stamp);
    DepthImagePublisher& publishToROS() override;
    DepthImagePublisher& publishJSON();
    Json::Value toJSON() const override;
};

// Camera Info publisher
class CameraInfoPublisher : public PublisherBase {
private:
    const rtabmap::CameraModel& model_;
    double stamp_;
    bool isDepth_;

public:
    CameraInfoPublisher(const rtabmap::CameraModel& model, double stamp, bool isDepth);
    CameraInfoPublisher& publishToROS() override;
    CameraInfoPublisher& publishJSON();
    Json::Value toJSON() const override;
};

// Point Cloud publisher
class PointCloudPublisher : public PublisherBase {
private:
    const rtabmap::LaserScan& scan_;
    double stamp_;

public:
    PointCloudPublisher(const rtabmap::LaserScan& scan, double stamp);
    PointCloudPublisher& publishToROS() override;
    PointCloudPublisher& publishJSON();
    Json::Value toJSON() const override;
};

// Features publisher
class FeaturesPublisher : public PublisherBase {
private:
    const std::vector<cv::KeyPoint>& kpts_;
    const std::vector<cv::Point3f>& kpts3d_;
    double stamp_;

public:
    FeaturesPublisher(const std::vector<cv::KeyPoint>& kpts,
                     const std::vector<cv::Point3f>& kpts3d,
                     double stamp);
    FeaturesPublisher& publishToROS() override;
    FeaturesPublisher& publishJSON();
    Json::Value toJSON() const override;
}; 
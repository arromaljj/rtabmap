#pragma once
#include <string>
#include <memory>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <condition_variable>
#include <Eigen/Core>
#include <ixwebsocket/IXWebSocket.h>
#include <opencv2/core/core.hpp>
#include "Message.h"
#include "PoseMessage.h"
#include "PointCloudMessage.h"
#include "RGBImageMessage.h"

// Define error codes
#define PUBSUB_ERROR_NONE 0
#define PUBSUB_ERROR_CONNECTION 1
#define PUBSUB_ERROR_SEND 2
#define PUBSUB_ERROR_INTERNAL 3

// Define queue size limits to prevent memory explosion
#define MAX_QUEUE_SIZE 10
#define MAX_POINT_CLOUD_SIZE 10000 // Limit points to prevent memory issues

// Forward declaration for WebSocket library
namespace ix { class WebSocket; }

// Paho MQTT C++ Headers (Replace placeholders)
#include "mqtt/async_client.h"
#include "mqtt/callback.h"
#include "mqtt/connect_options.h"
#include "mqtt/message.h"

class DataPublisher : public std::enable_shared_from_this<DataPublisher>
{
public:
    // Helper for PointCloud serialization
    struct SerializedPointCloud2 { 
        std::string data; 
        // Other fields would go here in a full implementation
    };
    
    // MQTT Callback Handler (Derived from mqtt::callback)
    class MqttCallbackHandler : public virtual mqtt::callback, public virtual mqtt::iaction_listener
    {   // Inherit from iaction_listener for connect success/failure
        DataPublisher& publisher_;
        // Reconnect vars (basic example)
        mqtt::connect_options& connOpts_;

        void on_failure(const mqtt::token& tok) override {
            LOGE("MQTT Connection attempt failed (token: " + std::to_string(tok.get_message_id()) + ")");
            publisher_.connected_ = false; 
            // Consider adding backoff/retry logic here or rely on automatic reconnect
            publisher_.setError(PUBSUB_ERROR_CONNECTION, "MQTT Connection Failed");
        }

        void on_success(const mqtt::token& tok) override {
            LOGI("MQTT Connection success (token: " + std::to_string(tok.get_message_id()) + ")");
            // Callback 'connected()' should be called by the library after this
        }

        void connected(const std::string& cause) override {
            LOGI("MQTT Connection successful. Cause: " + cause);
            publisher_.connected_ = true;
            publisher_.setError(PUBSUB_ERROR_NONE, "");
            // No need to advertise topics in MQTT
        }

        void connection_lost(const std::string& cause) override {
            std::string errMsg = "MQTT Connection Lost";
            if (!cause.empty())
                errMsg += ": " + cause;
            LOGE(errMsg);
            publisher_.connected_ = false;
            publisher_.setError(PUBSUB_ERROR_CONNECTION, errMsg);
            // Note: If automatic_reconnect is true in connOpts, Paho handles reconnection.
            // If not, you might manually trigger publisher_.connectToMqttBroker() here (with care for loops).
        }

        void delivery_complete(mqtt::delivery_token_ptr tok) override {
            LOGV("MQTT Delivery complete for token: " + (tok ? std::to_string(tok->get_message_id()) : "null"));
        }

    public:
        MqttCallbackHandler(DataPublisher& pub, mqtt::connect_options& connOpts)
            : publisher_(pub), connOpts_(connOpts) {}
    };

    // Friend declaration to allow callback access
    friend class MqttCallbackHandler;

    // UPDATED: Constructor for MQTT configuration
    DataPublisher(const std::string& mqttBrokerUrl = "tcp://localhost:1883",
                 const std::string& mqttClientId = "rtabmap_ios_client_01",
                 const std::string& deviceId = "rtabmap_device_01",
                 const std::string& mqttUsername = "",
                 const std::string& mqttPassword = "");
    ~DataPublisher();

    // Control methods
    void start();
    void stop();
    
    // Publishing methods - now non-blocking, queue messages for async processing
    void publishPointCloud(const cv::Mat& points);
    void publishPose(const rtabmap::Transform& pose);
    void publishStringMessage(const std::string& message);
    void publishOdometry(const rtabmap::Transform& pose, const std::string& childFrameId = "base_link");
    void publishRGBImage(const cv::Mat& rgb);

    // Configure bridge
    bool isConnected() const { return connected_; }
    bool isRunning() const { return running_; }

    // Error handling
    int getLastErrorCode() const;
    std::string getLastErrorMsg() const;
    
    // Memory tracking
    static void logMemoryUsage();

private:
    // Base class for message tasks in the queue
    class PublishTask {
    public:
        virtual ~PublishTask() {}
        virtual void execute(DataPublisher* publisher) = 0;
    };
    
    // Specific task implementations with improved memory handling
    class PointCloudTask : public PublishTask {
    public:
        // Use shared_ptr to avoid copying large matrices
        PointCloudTask(const cv::Mat& points);
        void execute(DataPublisher* publisher) override;
    private:
        cv::Mat points_; // Still using cv::Mat as it's efficient with ref counting
    };
    
    class PoseTask : public PublishTask {
    public:
        PoseTask(const rtabmap::Transform& pose) : pose_(pose) {}
        void execute(DataPublisher* publisher) override;
    private:
        rtabmap::Transform pose_;
    };
    
    class StringTask : public PublishTask {
    public:
        StringTask(const std::string& message) : message_(message) {}
        void execute(DataPublisher* publisher) override;
    private:
        std::string message_;
    };
    
    class OdometryTask : public PublishTask {
    public:
        OdometryTask(const rtabmap::Transform& pose, const std::string& childFrameId) 
            : pose_(pose), childFrameId_(childFrameId) {}
        void execute(DataPublisher* publisher) override;
    private:
        rtabmap::Transform pose_;
        std::string childFrameId_;
    };
    
    class RGBImageTask : public PublishTask {
    public:
        RGBImageTask(const cv::Mat& rgb);
        void execute(DataPublisher* publisher) override;
    private:
        cv::Mat rgb_;
    };

    // Worker thread function
    void workerThread();
    
    // Task queue management with improved handling
    void queueTask(std::unique_ptr<PublishTask> task);
    void processTasks();
    
    // UPDATED: Connection management methods for MQTT
    void connectToMqttBroker();
    void disconnectFromMqttBroker();
    
    // MQTT related members
    std::string mqttBrokerUrl_;
    std::string mqttClientId_;
    std::string deviceId_;
    std::string mqttUsername_;
    std::string mqttPassword_;

    // Store connection options for reconnects/callbacks
    mqtt::connect_options connOpts_;
    // Store TLS options separately if complex
    mqtt::ssl_options sslOpts_;
    // Callback handler needs to be accessible
    MqttCallbackHandler callbackHandler_;

    std::string frameId_{"map"}; // Keep frameId if needed for payload content

    // MQTT Client
    std::shared_ptr<mqtt::async_client> mqttClient_;

    std::mutex mutex_;              // Main mutex for connection state
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    
    // Error tracking
    mutable std::mutex errorMutex_;
    int lastErrorCode_{PUBSUB_ERROR_NONE};
    std::string lastErrorMsg_;
    
    // Message instances
    std::shared_ptr<PoseMessage> poseMessage_;
    std::shared_ptr<PointCloudMessage> pointCloudMessage_;
    std::shared_ptr<RGBImageMessage> rgbImageMessage_; // UPDATED Type
    
    // Async processing queue and thread with size limiting
    std::queue<std::unique_ptr<PublishTask>> taskQueue_;
    std::mutex queueMutex_;
    std::condition_variable queueCondition_;
    std::thread workerThread_;
    std::atomic<bool> threadRunning_{false};
    std::atomic<size_t> queueSize_{0}; // Track queue size atomically
    
    // Throttling variables
    std::atomic<double> lastPointCloudTime_{0};
    
    // Internal error handling
    void setError(int errorCode, const std::string& errorMsg);
};
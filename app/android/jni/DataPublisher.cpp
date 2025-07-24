#include "DataPublisher.h"
#include <sstream>
#include <ctime>
#include <iostream>
#include <cmath>
#include <sys/resource.h> // For memory tracking

// Paho MQTT C++ Headers (Already included via DataPublisher.h)

// For iOS, we'll use a simpler logging system
#define LOG_TAG "DataPublisher"
#define LOGI(msg) std::cout << "[INFO][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGW(msg) std::cout << "[WARN][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGE(msg) std::cout << "[ERROR][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGD(msg) std::cout << "[DEBUG][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGV(msg) std::cout << "[VERBOSE][" << LOG_TAG << "] " << (msg) << std::endl

// Memory-efficient constructor for PointCloudTask
DataPublisher::PointCloudTask::PointCloudTask(const cv::Mat& points) {
    // Only copy if necessary (if points has multiple references)
    if (points.rows > MAX_POINT_CLOUD_SIZE) {
        // Downsample large point clouds
        int factor = static_cast<int>(std::ceil(static_cast<float>(points.rows) / MAX_POINT_CLOUD_SIZE));
        points_ = cv::Mat(MAX_POINT_CLOUD_SIZE, points.cols, points.type());
        
        for (int i = 0; i < MAX_POINT_CLOUD_SIZE; ++i) {
            int srcIdx = i * factor;
            if (srcIdx < points.rows) {
                for (int j = 0; j < points.cols; ++j) {
                    points_.at<float>(i, j) = points.at<float>(srcIdx, j);
                }
            }
        }
        LOGW("Point cloud downsampled from " + std::to_string(points.rows) + 
             " to " + std::to_string(MAX_POINT_CLOUD_SIZE) + " points");
    } else {
        points_ = points;
    }
}

// UPDATED: Rename class and update constructor
DataPublisher::RGBImageTask::RGBImageTask(const cv::Mat& rgb) {
    rgb_ = rgb;
}

// Implementation of PublishTask execute methods
void DataPublisher::PointCloudTask::execute(DataPublisher* publisher) {
    if (!publisher->mqttClient_ || !publisher->connected_) { 
        LOGE("MQTT client not ready/connected, cannot publish point cloud.");
        return;
    }
    if (publisher->pointCloudMessage_) {
        publisher->pointCloudMessage_->setPointCloud(points_);
        std::string topic = "rtabmap/ios/" + publisher->deviceId_ + "/pointcloud";
        std::string payload = publisher->pointCloudMessage_->createJsonPayload();
        
        if (!payload.empty()) {
            try {
                LOGI("Publishing PointCloud to MQTT topic: " + topic);
                mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
                pubmsg->set_qos(0); // Set QoS (0 = at most once)
                publisher->mqttClient_->publish(pubmsg);
                 // delivery_complete callback will confirm for QoS 1/2
            } catch (const mqtt::exception& exc) {
                LOGE("MQTT Exception publishing point cloud: " + std::string(exc.what()));
                publisher->setError(PUBSUB_ERROR_SEND, "MQTT publish failed: " + std::string(exc.what()));
            } catch (const std::exception& e) {
                LOGE("Std Exception publishing point cloud: " + std::string(e.what()));
                publisher->setError(PUBSUB_ERROR_SEND, "MQTT publish failed: " + std::string(e.what()));
            }
        } else {
             LOGW("Skipping MQTT publish for point cloud: empty payload created.");
        }
        points_.release();
    }
}

void DataPublisher::PoseTask::execute(DataPublisher* publisher) {
    if (!publisher->mqttClient_ || !publisher->connected_) { 
        LOGE("MQTT client not ready/connected, cannot publish pose/TF.");
        return;
    }
    if (publisher->poseMessage_) {
        publisher->poseMessage_->setPose(pose_);
        
        // Publish Pose
        try {
            std::string poseTopic = "rtabmap/ios/" + publisher->deviceId_ + "/pose";
            std::string posePayload = publisher->poseMessage_->createPoseJsonPayload();
            if (!posePayload.empty()) {
                LOGI("Publishing Pose to MQTT topic: " + poseTopic);
                mqtt::message_ptr pubmsg = mqtt::make_message(poseTopic, posePayload);
                pubmsg->set_qos(0);
                publisher->mqttClient_->publish(pubmsg);
            }
        } catch (const mqtt::exception& exc) {
            LOGE("MQTT Exception publishing pose: " + std::string(exc.what()));
            publisher->setError(PUBSUB_ERROR_SEND, "MQTT pose publish failed: " + std::string(exc.what()));
        } catch (const std::exception& e) {
            LOGE("Std Exception publishing pose: " + std::string(e.what()));
            publisher->setError(PUBSUB_ERROR_SEND, "MQTT pose publish failed: " + std::string(e.what()));
        }

        // Publish TF
        try {
            std::string tfTopic = "rtabmap/ios/" + publisher->deviceId_ + "/tf";
            std::string tfPayload = publisher->poseMessage_->createTfJsonPayload("base_link"); 
            if (!tfPayload.empty()) {
                LOGI("Publishing TF to MQTT topic: " + tfTopic);
                mqtt::message_ptr pubmsg = mqtt::make_message(tfTopic, tfPayload);
                pubmsg->set_qos(0);
                publisher->mqttClient_->publish(pubmsg);
            }
        } catch (const mqtt::exception& exc) {
            LOGE("MQTT Exception publishing TF: " + std::string(exc.what()));
            publisher->setError(PUBSUB_ERROR_SEND, "MQTT TF publish failed: " + std::string(exc.what()));
        } catch (const std::exception& e) {
            LOGE("Std Exception publishing TF: " + std::string(e.what()));
            publisher->setError(PUBSUB_ERROR_SEND, "MQTT TF publish failed: " + std::string(e.what()));
        }
    }
}

void DataPublisher::StringTask::execute(DataPublisher* publisher) {
    if (!publisher->mqttClient_ || !publisher->connected_) { 
        LOGE("MQTT client not ready/connected, cannot publish string message.");
        return;
    }
    std::string topic = "rtabmap/ios/" + publisher->deviceId_ + "/status";
    std::string payload = "{\"data\":\"" + message_ + "\"}"; 
    
    try {
        LOGI("Publishing String to MQTT topic: " + topic);
        mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
        pubmsg->set_qos(0);
        publisher->mqttClient_->publish(pubmsg);
    } catch (const mqtt::exception& exc) {
        LOGE("MQTT Exception publishing string: " + std::string(exc.what()));
        publisher->setError(PUBSUB_ERROR_SEND, "MQTT string publish failed: " + std::string(exc.what()));
    } catch (const std::exception& e) {
         LOGE("Std Exception publishing string: " + std::string(e.what()));
         publisher->setError(PUBSUB_ERROR_SEND, "MQTT string publish failed: " + std::string(e.what()));
    }
}

void DataPublisher::OdometryTask::execute(DataPublisher* publisher) {
    if (!publisher->mqttClient_ || !publisher->connected_) { 
        LOGE("MQTT client not ready/connected, cannot publish odometry.");
        return;
    }
    if (publisher->poseMessage_) {
        publisher->poseMessage_->setPose(pose_);
        std::string topic = "rtabmap/ios/" + publisher->deviceId_ + "/odometry";
        std::string payload = publisher->poseMessage_->createOdomJsonPayload(childFrameId_); 
        
        if (!payload.empty()) {
            try {
                LOGI("Publishing Odometry to MQTT topic: " + topic);
                mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
                pubmsg->set_qos(0);
                publisher->mqttClient_->publish(pubmsg);
            } catch (const mqtt::exception& exc) {
                LOGE("MQTT Exception publishing odometry: " + std::string(exc.what()));
                publisher->setError(PUBSUB_ERROR_SEND, "MQTT odom publish failed: " + std::string(exc.what()));
            } catch (const std::exception& e) {
                 LOGE("Std Exception publishing odometry: " + std::string(e.what()));
                 publisher->setError(PUBSUB_ERROR_SEND, "MQTT odom publish failed: " + std::string(e.what()));
            }
        } else {
             LOGW("Skipping MQTT publish for odometry: empty payload created.");
        }
    }
}

// UPDATED: Class name, execute logic for MQTT, use rgbImageMessage_
void DataPublisher::RGBImageTask::execute(DataPublisher* publisher) {
    if (!publisher->mqttClient_ || !publisher->connected_) { 
        LOGE("MQTT client not ready/connected, cannot publish RGB Image.");
        return;
    }
    if (publisher->rgbImageMessage_) {
        LOGI("Executing RGBImage task: Image size " + std::to_string(rgb_.cols) + "x" + std::to_string(rgb_.rows));
        if (rgb_.empty()) {
            LOGE("RGB image is empty, cannot publish RGB message");
            return;
        }
        try {
            publisher->rgbImageMessage_->setRGBImage(rgb_); 
            std::string topic = "rtabmap/ios/" + publisher->deviceId_ + "/image/rgb";
            std::string payload = publisher->rgbImageMessage_->createJsonPayload();
            
            if (!payload.empty()) {
                LOGI("Publishing RGB Image to MQTT topic: " + topic);
                mqtt::message_ptr pubmsg = mqtt::make_message(topic, payload);
                pubmsg->set_qos(0);
                publisher->mqttClient_->publish(pubmsg);
                // No immediate success confirmation here for QoS 0
            } else {
                 LOGE("Failed to create RGB Image payload for MQTT");
            }
        } catch (const mqtt::exception& exc) {
            LOGE("MQTT Exception publishing RGB message: " + std::string(exc.what()));
             publisher->setError(PUBSUB_ERROR_SEND, "MQTT RGB publish failed: " + std::string(exc.what()));
        } catch (const std::exception& e) {
            LOGE("Std Exception publishing RGB message: " + std::string(e.what()));
             publisher->setError(PUBSUB_ERROR_SEND, "MQTT RGB publish failed: " + std::string(e.what()));
        }
        rgb_.release();
    } else {
        LOGE("Cannot publish RGB message: message object is null");
    }
}

// Add memory tracking method
void DataPublisher::logMemoryUsage() {
    #if defined(__APPLE__) || defined(__linux__)
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        LOGI("Memory usage: " + std::to_string(usage.ru_maxrss / 1024) + " MB");
    }
    #else
    LOGI("Memory tracking not implemented for this platform");
    #endif
}

DataPublisher::DataPublisher(const std::string& mqttBrokerUrl,
                            const std::string& mqttClientId,
                            const std::string& deviceId,
                            const std::string& mqttUsername,
                            const std::string& mqttPassword)
    : mqttBrokerUrl_(mqttBrokerUrl),
      mqttClientId_(mqttClientId),
      deviceId_(deviceId),
      mqttUsername_(mqttUsername),
      mqttPassword_(mqttPassword),
      mqttClient_(nullptr),
      callbackHandler_(*this, connOpts_), // Initialize callback handler
      running_(false),
      connected_(false),
      threadRunning_(false),
      queueSize_(0),
      lastErrorCode_(PUBSUB_ERROR_NONE),
      lastPointCloudTime_(0)
{
    poseMessage_ = std::make_shared<PoseMessage>(frameId_);
    pointCloudMessage_ = std::make_shared<PointCloudMessage>(frameId_);
    rgbImageMessage_ = std::make_shared<RGBImageMessage>(frameId_);
    
    // --- Configure Connection Options --- 
    connOpts_.set_keep_alive_interval(20); // Example keep-alive
    connOpts_.set_clean_session(true);
    connOpts_.set_automatic_reconnect(true); // Let Paho handle basic reconnects

    // Set credentials if provided
    if (!mqttUsername_.empty()) {
        connOpts_.set_user_name(mqttUsername_);
        connOpts_.set_password(mqttPassword_);
    }

    // --- Configure TLS/SSL Options --- 
    // Check if broker URL starts with ssl:// or mqtts:// (case-insensitive)
    std::string lowerBrokerUrl = mqttBrokerUrl_;
    std::transform(lowerBrokerUrl.begin(), lowerBrokerUrl.end(), lowerBrokerUrl.begin(), ::tolower);
    if (lowerBrokerUrl.rfind("ssl://", 0) == 0 || lowerBrokerUrl.rfind("mqtts://", 0) == 0) {
        LOGI("TLS/SSL connection requested.");
        // Enable SSL/TLS
        // IMPORTANT: This assumes the system's default trust store is sufficient 
        // or that Paho can find it. For robust iOS deployment, you might need
        // to explicitly set the trust store (e.g., using bundled CAs).
        sslOpts_.set_enable_server_cert_auth(true); // Verify server certificate
        // sslOpts_.set_trust_store("/path/to/ca.crt"); // Explicit CA bundle if needed
        // sslOpts_.set_key_store(...); // Client certificate if needed
        // sslOpts_.set_private_key(...); // Client private key if needed

        connOpts_.set_ssl(sslOpts_);
    } else {
         LOGI("Using non-TLS connection.");
    }

    LOGI("DataPublisher created for MQTT. Broker: " + mqttBrokerUrl_ + ", ClientID: " + mqttClientId_);
}

DataPublisher::~DataPublisher() {
    stop(); // Ensure stop is called to disconnect and clean up
}

void DataPublisher::setError(int errorCode, const std::string& errorMsg) {
    std::lock_guard<std::mutex> lock(errorMutex_);
    lastErrorCode_ = errorCode;
    lastErrorMsg_ = errorMsg;
    LOGE("Error set: Code=" + std::to_string(errorCode) + ", Msg=" + errorMsg);
}

void DataPublisher::workerThread() {
    LOGI("DataPublisher worker thread started");
    
    // Log initial memory usage
    logMemoryUsage();
    
    // Counter for periodic memory logging
    int taskCounter = 0;
    
    while (threadRunning_) {
        processTasks();
        
        // If disconnected but should be running, try to reconnect
        if (running_ && !connected_) {
            LOGI("Worker thread: Not connected, attempting MQTT reconnect...");
            connectToMqttBroker();
            // Sleep to avoid hammering the server with reconnect attempts
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        // Periodically log memory usage (every 100 tasks)
        if (++taskCounter >= 100) {
            logMemoryUsage();
            taskCounter = 0;
        }
        
        // Short sleep if idle to prevent busy-waiting
        if (!connected_ || taskQueue_.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    
    // Log final memory usage
    logMemoryUsage();
    
    LOGI("DataPublisher worker thread stopped");
}

void DataPublisher::queueTask(std::unique_ptr<PublishTask> task) {
    if (!running_ || !threadRunning_) {
        LOGW("Cannot queue task: publisher not running");
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        
        // Limit queue size to prevent memory explosion
        if (taskQueue_.size() >= MAX_QUEUE_SIZE) {
            LOGW("Task queue full (" + std::to_string(taskQueue_.size()) + 
                 " items), dropping oldest task");
            taskQueue_.pop(); // Remove oldest task
        }
        
        // Add new task
        taskQueue_.push(std::move(task));
        queueSize_ = taskQueue_.size(); // Update atomic counter
    }
    
    queueCondition_.notify_one();
}

void DataPublisher::processTasks() {
    std::unique_ptr<PublishTask> task;
    bool hasTask = false;
    
    {
        std::unique_lock<std::mutex> lock(queueMutex_);
        // Wait for up to 100ms for a task or until notified
        if (queueCondition_.wait_for(lock, std::chrono::milliseconds(100), 
                                    [this] { return !taskQueue_.empty(); })) {
            // Get the next task
            task = std::move(taskQueue_.front());
            taskQueue_.pop();
            queueSize_ = taskQueue_.size(); // Update atomic counter
            hasTask = true;
        }
    }
    
    // Process the task outside the lock only if connected 
    if (hasTask && task && connected_) { 
        try {
            // Execute the task
            task->execute(this);
            
            // Explicitly destroy the task to release memory
            task.reset();
        } catch (const std::exception& e) {
            LOGE("Exception in task execution: " + std::string(e.what()));
            setError(PUBSUB_ERROR_INTERNAL, std::string("Task execution failed: ") + e.what());
            
            // Ensure task is destroyed even on exception
            task.reset();
        } catch (...) {
            LOGE("Unknown exception in task execution");
            setError(PUBSUB_ERROR_INTERNAL, "Unknown exception in task execution");
            
            // Ensure task is destroyed even on exception
            task.reset();
        }
    }
}

void DataPublisher::connectToMqttBroker() {
    std::lock_guard<std::mutex> lock(mutex_); 

    // Prevent reconnect attempts if already connecting or connected
    if ((mqttClient_ && mqttClient_->is_connected()) || !running_) {
        if (!running_) { 
             LOGW("connectToMqttBroker called but publisher is not running."); 
        } else {
             LOGW("connectToMqttBroker called but already connected/connecting.");
        }
        return;
    }

    LOGI("Attempting to connect to MQTT broker at " + mqttBrokerUrl_);

    try {
        // Create the client instance if it doesn't exist
        if (!mqttClient_) {
             mqttClient_ = std::make_shared<mqtt::async_client>(mqttBrokerUrl_, mqttClientId_);
             // Set the callbacks BEFORE connecting
             mqttClient_->set_callback(callbackHandler_);
        }
        
        LOGI("Initiating MQTT Connection...");
        // Connect using the stored options. 
        // The callback handler (specifically on_success/on_failure) will handle the result.
        mqttClient_->connect(connOpts_, nullptr, callbackHandler_); 
        // nullptr context, callbackHandler acts as action listener

    } catch (const mqtt::exception& exc) {
        LOGE("Exception during MQTT client setup/connect call: " + std::string(exc.what()));
        setError(PUBSUB_ERROR_CONNECTION, std::string("MQTT connect exception: ") + exc.what());
        // Don't reset client here, Paho's reconnect might still work depending on the error
        connected_ = false; // Ensure connected is false
    } catch (const std::exception& e) {
        LOGE("Std Exception during MQTT client setup/connect call: " + std::string(e.what()));
        setError(PUBSUB_ERROR_CONNECTION, std::string("MQTT connect std::exception: ") + e.what());
        connected_ = false; 
    }
}

void DataPublisher::disconnectFromMqttBroker() {
    std::lock_guard<std::mutex> lock(mutex_); 

    if (mqttClient_ && mqttClient_->is_connected()) {
        LOGI("Disconnecting MQTT client...");
        try {
            // Perform a clean disconnect
            mqttClient_->disconnect()->wait(); // Wait for disconnect to complete
            LOGI("MQTT client disconnected.");
        } catch (const mqtt::exception& exc) {
             LOGE("Exception during MQTT disconnect: " + std::string(exc.what()));
        } catch (const std::exception& e) {
             LOGE("Std Exception during MQTT disconnect: " + std::string(e.what()));
        }
    } else if (mqttClient_) {
         LOGW("disconnectFromMqttBroker called but mqttClient_ is not connected.");
    } else {
        LOGW("disconnectFromMqttBroker called but mqttClient_ is null.");
    }
    // Setting connected_ = false is handled by the connection_lost callback
    // Setting it here might race with the callback in some scenarios.
    // However, explicitly setting it after a manual disconnect is safer.
    connected_ = false; 
}

void DataPublisher::start() {
    if (running_) {
        LOGW("Start called but already running.");
        return;
    }

    // Log initial memory usage
    logMemoryUsage();

    running_ = true;
    threadRunning_ = true;
    LOGI("DataPublisher starting for MQTT...");
    
    // Start the worker thread
    workerThread_ = std::thread(&DataPublisher::workerThread, this);
    
    // Attempt initial connection
    connectToMqttBroker();

    LOGI("DataPublisher started for MQTT.");
}

void DataPublisher::stop() {
    if (!running_) {
        LOGW("Stop called but not running.");
        return;
    }
    
    LOGI("DataPublisher stopping MQTT...");
    running_ = false;
    
    // Stop worker thread
    if (threadRunning_) {
        threadRunning_ = false;
        queueCondition_.notify_all(); // Wake up the worker thread if it's waiting
        
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }

    // Clear any pending tasks to free memory
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        while (!taskQueue_.empty()) {
            taskQueue_.pop();
        }
        queueSize_ = 0;
    }

    // Disconnect MQTT
    disconnectFromMqttBroker();
    
    // Log memory usage after cleanup
    logMemoryUsage();

    LOGI("DataPublisher stopped for MQTT.");
}

void DataPublisher::publishPose(const rtabmap::Transform& pose) {
    // Create a task and add it to the queue for async processing
    auto task = std::make_unique<PoseTask>(pose);
    queueTask(std::move(task));
}

void DataPublisher::publishStringMessage(const std::string& message) {
    // Create a task and add it to the queue for async processing
    auto task = std::make_unique<StringTask>(message);
    queueTask(std::move(task));
}

void DataPublisher::publishOdometry(const rtabmap::Transform& pose, const std::string& childFrameId) {
    // Create a task and add it to the queue for async processing
    auto task = std::make_unique<OdometryTask>(pose, childFrameId);
    queueTask(std::move(task));
}

void DataPublisher::publishPointCloud(const cv::Mat& points) {
    if (points.empty()) {
        LOGV("Skipping publishPointCloud (points matrix is empty).");
        return;
    }
    
    // Throttle point cloud publishing based on time
    // Only publish every 0.5 seconds to reduce memory pressure
    double currentTime = std::time(nullptr);
    double lastTime = lastPointCloudTime_.load();
    if (currentTime - lastTime < 0.5) {
        LOGV("Skipping point cloud publication (throttled)");
        return;
    }
    lastPointCloudTime_ = currentTime;

    // Create a task and add it to the queue for async processing
    auto task = std::make_unique<PointCloudTask>(points);
    queueTask(std::move(task));
}

void DataPublisher::publishRGBImage(const cv::Mat& rgb) {
    if (rgb.empty()) {
        LOGV("Skipping publishRGBImage (rgb is empty).");
        return;
    }
    auto task = std::make_unique<RGBImageTask>(rgb);
    queueTask(std::move(task));
}

int DataPublisher::getLastErrorCode() const {
    std::lock_guard<std::mutex> lock(errorMutex_);
    return lastErrorCode_;
}

std::string DataPublisher::getLastErrorMsg() const {
    std::lock_guard<std::mutex> lock(errorMutex_);
    return lastErrorMsg_;
}

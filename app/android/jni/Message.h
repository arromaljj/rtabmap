#pragma once
#include <string>
#include <memory>
#include <mutex>
#include <ctime>
#include <sstream>
#include <ixwebsocket/IXWebSocket.h>

// Define error codes
#define PUBSUB_ERROR_NONE 0
#define PUBSUB_ERROR_CONNECTION 1
#define PUBSUB_ERROR_SEND 2
#define PUBSUB_ERROR_INTERNAL 3

namespace ix { class WebSocket; }

class Message {
public:
    Message(const std::string& topic, const std::string& frameId = "map");
    virtual ~Message();

    // Create ROS-compatible message
    virtual std::string createRosbridgeMessage() const = 0;
    
    // Send message through WebSocket
    bool publish(std::shared_ptr<ix::WebSocket> wsClient);
    
    // Getters and setters
    const std::string& getTopic() const { return topic_; }
    void setTopic(const std::string& topic) { topic_ = topic; }
    
    const std::string& getFrameId() const { return frameId_; }
    void setFrameId(const std::string& frameId) { frameId_ = frameId; }
    
    int getLastErrorCode() const;
    std::string getLastErrorMsg() const;

protected:
    std::string topic_;
    std::string frameId_;
    
    // Error tracking
    mutable std::mutex errorMutex_;
    mutable int lastErrorCode_{PUBSUB_ERROR_NONE};
    mutable std::string lastErrorMsg_;
    
    // Helper function for base64 encoding (for derived classes)
    std::string base64_encode(const uint8_t* data, size_t length) const;

    // Internal error handling
    void setError(int errorCode, const std::string& errorMsg) const;
}; 
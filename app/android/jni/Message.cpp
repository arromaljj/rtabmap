#include "Message.h"
#include <iostream>
#include <ixwebsocket/IXWebSocketSendData.h>
#include <ixwebsocket/IXWebSocketMessage.h>

// For iOS, we'll use a simpler logging system
#define LOG_TAG "Message"
#define LOGI(msg) std::cout << "[INFO][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGW(msg) std::cout << "[WARN][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGE(msg) std::cout << "[ERROR][" << LOG_TAG << "] " << (msg) << std::endl

Message::Message(const std::string& topic, const std::string& frameId)
    : topic_(topic), 
      frameId_(frameId)
{
}

Message::~Message()
{
}

bool Message::publish(std::shared_ptr<ix::WebSocket> wsClient)
{
    if (!wsClient || wsClient->getReadyState() != ix::ReadyState::Open) {
        setError(PUBSUB_ERROR_CONNECTION, "WebSocket not connected");
        return false;
    }
    
    try {
        std::string message = createRosbridgeMessage();
        wsClient->send(message);
        setError(PUBSUB_ERROR_NONE, "");
        return true;
    } catch (const std::exception& e) {
        std::string error = "Failed to publish message: ";
        error += e.what();
        setError(PUBSUB_ERROR_SEND, error);
        LOGE(error);
        return false;
    }
}

int Message::getLastErrorCode() const
{
    std::lock_guard<std::mutex> lock(errorMutex_);
    return lastErrorCode_;
}

std::string Message::getLastErrorMsg() const
{
    std::lock_guard<std::mutex> lock(errorMutex_);
    return lastErrorMsg_;
}

void Message::setError(int errorCode, const std::string& errorMsg) const
{
    std::lock_guard<std::mutex> lock(errorMutex_);
    lastErrorCode_ = errorCode;
    lastErrorMsg_ = errorMsg;
    if (errorCode != PUBSUB_ERROR_NONE) {
        LOGE("Error set: Code=" + std::to_string(errorCode) + ", Msg=" + errorMsg);
    }
}

// Helper function to base64 encode binary data
std::string Message::base64_encode(const uint8_t* data, size_t length) const
{
    static const std::string base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
        
    std::string ret;
    ret.reserve((length + 2) / 3 * 4);

    for (size_t i = 0; i < length; i += 3) {
        uint32_t octet_a = i < length ? data[i] : 0;
        uint32_t octet_b = i + 1 < length ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < length ? data[i + 2] : 0;

        uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

        ret.push_back(base64_chars[(triple >> 18) & 0x3F]);
        ret.push_back(base64_chars[(triple >> 12) & 0x3F]);
        ret.push_back(base64_chars[(triple >> 6) & 0x3F]);
        ret.push_back(base64_chars[triple & 0x3F]);
    }

    // Add padding if necessary
    switch (length % 3) {
        case 1:
            ret[ret.size() - 2] = '=';
            ret[ret.size() - 1] = '=';
            break;
        case 2:
            ret[ret.size() - 1] = '=';
            break;
    }

    return ret;
} 
#include "PCLUtil.h"
#include <stdexcept>
#include <iostream>
#include <cstring>
#include <vector>





//std::vector<uint8_t> serializeLaserScan2(const rtabmap::LaserScan& scan){
//    if(scan.empty()) {
//        throw std::runtime_error("Cannot serialize empty LaserScan");
//    }
//
//    std::vector<uint8_t> data;
//    // data.reserve(1024);  // Pre-allocate some space
//    uint8_t num_fields = 7;
//    data.push_back(num_fields);
//    
//
//
//
//
//
//
//}


std::vector<uint8_t> serializeLaserScan(const rtabmap::LaserScan& scan) {
    if(scan.empty()) {
        throw std::runtime_error("Cannot serialize empty LaserScan");
    }

    std::vector<uint8_t> data;
    data.reserve(1024);  // Pre-allocate some space

    // Helper function to append any POD type to the vector
    auto appendData = [&data](const auto& value) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        data.insert(data.end(), bytes, bytes + sizeof(value));
    };

    // Serialize format
    int format = static_cast<int>(scan.format());
    appendData(format);

    // Serialize basic parameters
    appendData(scan.maxPoints());
    appendData(scan.rangeMin());
    appendData(scan.rangeMax());
    appendData(scan.angleMin());
    appendData(scan.angleMax());
    appendData(scan.angleIncrement());

    // Serialize cv::Mat data
    const cv::Mat& mat = scan.data();
    int rows = mat.rows;
    int cols = mat.cols;
    int type = mat.type();
    
    appendData(rows);
    appendData(cols);
    appendData(type);

    if(rows * cols > 0) {
        size_t dataSize = mat.total() * mat.elemSize();
        const uint8_t* matData = mat.data;
        data.insert(data.end(), matData, matData + dataSize);
    }

    return data;
}

rtabmap::LaserScan deserializeLaserScan(const std::vector<uint8_t>& data) {
    if(data.empty()) {
        throw std::runtime_error("Cannot deserialize empty data");
    }

    size_t offset = 0;

    // Helper function to read any POD type from the vector
    auto readData = [&data, &offset](auto& value) {
        if(offset + sizeof(value) > data.size()) {
            throw std::runtime_error("Corrupted data: unexpected end of buffer");
        }
        memcpy(&value, data.data() + offset, sizeof(value));
        offset += sizeof(value);
    };

    // Read format
    int formatInt;
    readData(formatInt);
    rtabmap::LaserScan::Format format = static_cast<rtabmap::LaserScan::Format>(formatInt);

    if(format < rtabmap::LaserScan::kUnknown || format > rtabmap::LaserScan::kXYZIT) {
        throw std::runtime_error("Corrupted data: invalid format");
    }

    // Read basic parameters
    int maxPoints;
    float rangeMin, rangeMax, angleMin, angleMax, angleIncrement;
    
    readData(maxPoints);
    readData(rangeMin);
    readData(rangeMax);
    readData(angleMin);
    readData(angleMax);
    readData(angleIncrement);

    // Read cv::Mat data
    int rows, cols, type;
    readData(rows);
    readData(cols);
    readData(type);

    if(rows < 0 || cols < 0) {
        throw std::runtime_error("Corrupted data: invalid matrix dimensions");
    }

    cv::Mat mat;
    if(rows * cols > 0) {
        mat.create(rows, cols, type);
        size_t dataSize = mat.total() * mat.elemSize();
        
        if(offset + dataSize > data.size()) {
            throw std::runtime_error("Corrupted data: matrix data exceeds buffer size");
        }
        
        memcpy(mat.data, data.data() + offset, dataSize);
        offset += dataSize;
    }

    // Create new LaserScan
    if(angleIncrement != 0.0f) {
        return rtabmap::LaserScan(mat, format, rangeMin, rangeMax, 
                                 angleMin, angleMax, angleIncrement);
    } else {
        return rtabmap::LaserScan(mat, maxPoints, rangeMax, format);
    }
}

SerializedPointCloud2 convertLaserScanToSerializedPointCloud2(
    const rtabmap::LaserScan& scan,
    const std::string& frame_id,
    uint64_t timestamp_ns) {
    
    if(scan.empty()) {
        throw std::runtime_error("Cannot convert empty LaserScan");
    }
    
    SerializedPointCloud2 result;
    result.frame_id = frame_id;
    result.timestamp = timestamp_ns;
    
    std::vector<uint8_t>& data = result.data;
    data.reserve(scan.data().total() * scan.data().elemSize() + 1024); // Reserve space for the data and header
    
    // Helper to append data to the buffer
    auto appendData = [&data](const auto& value) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        data.insert(data.end(), bytes, bytes + sizeof(value));
    };
    
    // Helper to append a string to the buffer
    auto appendString = [&data, &appendData](const std::string& str) {
        // First write the string length
        uint32_t length = static_cast<uint32_t>(str.length());
        appendData(length);
        
        // Then write the string data
        data.insert(data.end(), str.begin(), str.end());
    };
    
    // Get point cloud info from the scan
    const cv::Mat& cloud = scan.data();
    int points = cloud.rows * cloud.cols;
    rtabmap::LaserScan::Format format = scan.format();
    
    // Header fields for a ROS PointCloud2 message
    uint32_t height = 1;
    uint32_t width = points;
    
    // Add height and width to data
    appendData(height);
    appendData(width);
    
    // Define fields based on the format
    std::vector<std::pair<std::string, int>> fields;
    uint32_t point_step = 0;
    
    switch(format) {
        case rtabmap::LaserScan::kXYZ:
        case rtabmap::LaserScan::kXYZRGB:
        case rtabmap::LaserScan::kXYZI:
        case rtabmap::LaserScan::kXYZNormal:
        case rtabmap::LaserScan::kXYZRGBNormal:
        case rtabmap::LaserScan::kXYZINormal:
            // Add XYZ fields
            fields.push_back({"x", 0});
            fields.push_back({"y", 4});
            fields.push_back({"z", 8});
            point_step = 12;
            
            // Handle RGB if present
            if(format == rtabmap::LaserScan::kXYZRGB || format == rtabmap::LaserScan::kXYZRGBNormal) {
                fields.push_back({"rgb", 12});
                point_step += 4;
            }
            
            // Handle intensity if present
            if(format == rtabmap::LaserScan::kXYZI || format == rtabmap::LaserScan::kXYZINormal) {
                fields.push_back({"intensity", 12});
                point_step += 4;
            }
            
            // Handle normals if present
            if(format == rtabmap::LaserScan::kXYZNormal || 
               format == rtabmap::LaserScan::kXYZRGBNormal || 
               format == rtabmap::LaserScan::kXYZINormal) {
                fields.push_back({"normal_x", point_step});
                fields.push_back({"normal_y", point_step + 4});
                fields.push_back({"normal_z", point_step + 8});
                point_step += 12;
            }
            break;
            
        default:
            throw std::runtime_error("Unsupported LaserScan format for PointCloud2 conversion");
    }
    
    // Write field count
    uint32_t field_count = static_cast<uint32_t>(fields.size());
    appendData(field_count);
    
    // Write fields
    for(const auto& field : fields) {
        // Write field name
        appendString(field.first);
        
        // Write field offset
        uint32_t offset = field.second;
        appendData(offset);
        
        // Write field datatype (7 = FLOAT32)
        uint8_t datatype = 7; // FLOAT32
        appendData(datatype);
        
        // Write field count (1 for each component)
        uint32_t count = 1;
        appendData(count);
    }
    
    // Write point_step
    appendData(point_step);
    
    // Write row_step (point_step * width for dense clouds)
    uint32_t row_step = point_step * width;
    appendData(row_step);
    
    // Write is_dense (true if no NaN or Inf points)
    uint32_t is_dense = 1; // Assuming dense cloud
    appendData(is_dense);
    
    // Now add the actual point cloud data
    // For simplicity, we're just going to add the raw data from the LaserScan
    // This assumes the LaserScan's data format matches our PointCloud2 layout
    if(cloud.total() > 0) {
        size_t dataSize = cloud.total() * cloud.elemSize();
        data.insert(data.end(), cloud.data, cloud.data + dataSize);
    }
    
    return result;
}

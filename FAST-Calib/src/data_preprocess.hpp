/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <fstream>
#include "common_lib.h"

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

class DataPreprocess
{
public:
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        string bag_path   = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;

        // 读图像
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            RCLCPP_ERROR(rclcpp::get_logger("DataPreprocess"), "%s", msg.c_str());
            return;
        }

        // ROS2: bag 路径为目录（rosbag2 格式，如 /path/to/my_bag）
        RCLCPP_INFO(rclcpp::get_logger("DataPreprocess"), "Loading the rosbag %s", bag_path.c_str());

        rosbag2_cpp::Reader bag_reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            bag_reader.open(storage_options, converter_options);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("DataPreprocess"), "LOADING BAG FAILED: %s", e.what());
            return;
        }

        auto metadata = bag_reader.get_metadata();
        std::string lidar_type_name;
        for (const auto &t : metadata.topics_with_message_count)
        {
            if (t.topic_metadata.name == lidar_topic)
            {
                lidar_type_name = t.topic_metadata.type;
                break;
            }
        }
        if (lidar_type_name.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("DataPreprocess"), "Topic %s not found in bag.", lidar_topic.c_str());
            return;
        }

        rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serialization;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serialization;
        const bool is_livox = (lidar_type_name == "livox_ros_driver2/msg/CustomMsg");

        while (bag_reader.has_next())
        {
            auto msg = bag_reader.read_next();
            if (msg->topic_name != lidar_topic)
                continue;

            if (is_livox)
            {
                livox_ros_driver2::msg::CustomMsg livox_msg;
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                livox_serialization.deserialize_message(&serialized_msg, &livox_msg);
                lidar_type_ = LiDARType::Solid;
                cloud_input_->reserve(cloud_input_->size() + livox_msg.point_num);
                for (uint32_t i = 0; i < livox_msg.point_num; ++i)
                {
                    Common::Point p;
                    p.x = livox_msg.points[i].x;
                    p.y = livox_msg.points[i].y;
                    p.z = livox_msg.points[i].z;
                    p.ring = static_cast<std::uint16_t>(livox_msg.points[i].line);
                    cloud_input_->push_back(p);
                }
            }
            else
            {
                sensor_msgs::msg::PointCloud2 pcl_msg;
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                pc2_serialization.deserialize_message(&serialized_msg, &pcl_msg);

                bool has_ring = false;
                for (const auto &f : pcl_msg.fields)
                {
                    if (f.name == "ring") { has_ring = true; break; }
                }
                if (has_ring)
                    lidar_type_ = LiDARType::Mech;
                else
                    lidar_type_ = LiDARType::Solid;

                sensor_msgs::PointCloud2ConstIterator<float> it_x(pcl_msg, "x");
                sensor_msgs::PointCloud2ConstIterator<float> it_y(pcl_msg, "y");
                sensor_msgs::PointCloud2ConstIterator<float> it_z(pcl_msg, "z");
                std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
                if (has_ring)
                    it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(pcl_msg, "ring"));

                const size_t n = static_cast<size_t>(pcl_msg.width) * pcl_msg.height;
                cloud_input_->reserve(cloud_input_->size() + n);
                for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
                {
                    Common::Point p;
                    p.x = *it_x;
                    p.y = *it_y;
                    p.z = *it_z;
                    if (has_ring)
                    {
                        p.ring = **it_ring_ptr;
                        ++(*it_ring_ptr);
                    }
                    else
                        p.ring = 0xFFFF;
                    cloud_input_->push_back(p);
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("DataPreprocess"), "Loaded %zu points from the rosbag.", cloud_input_->size());
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif

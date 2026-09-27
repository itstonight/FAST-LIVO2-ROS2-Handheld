/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include <iomanip>
#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mono_qr_pattern");

    Params params = loadParameters(node);

    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(node, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(node, params));

    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;

    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);

    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);

    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;
        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;
        default:
            std::cerr << BOLDYELLOW << "[Main] Unknown LiDAR type." << RESET << std::endl;
            break;
    }

    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    sortPatternCenters(qr_center_cloud, qr_centers, "camera");
    sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0)
    {
        std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
                  << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {
        if (DEBUG)
        {
            auto stamp = node->now();
            std::string frame_id = "map";

            sensor_msgs::msg::PointCloud2 qr_centers_msg;
            pcl::toROSMsg(*qr_centers, qr_centers_msg);
            qr_centers_msg.header.stamp = stamp;
            qr_centers_msg.header.frame_id = frame_id;
            qrDetectPtr->qr_pub_->publish(qr_centers_msg);

            sensor_msgs::msg::PointCloud2 lidar_centers_msg;
            pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
            lidar_centers_msg.header.stamp = stamp;
            lidar_centers_msg.header.frame_id = frame_id;
            lidarDetectPtr->center_pub_->publish(lidar_centers_msg);

            sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
            pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
            filtered_cloud_msg.header.stamp = stamp;
            filtered_cloud_msg.header.frame_id = frame_id;
            lidarDetectPtr->filtered_pub_->publish(filtered_cloud_msg);

            sensor_msgs::msg::PointCloud2 plane_cloud_msg;
            pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
            plane_cloud_msg.header.stamp = stamp;
            plane_cloud_msg.header.frame_id = frame_id;
            lidarDetectPtr->plane_pub_->publish(plane_cloud_msg);

            sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
            pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
            aligned_cloud_msg.header.stamp = stamp;
            aligned_cloud_msg.header.frame_id = frame_id;
            lidarDetectPtr->aligned_pub_->publish(aligned_cloud_msg);

            sensor_msgs::msg::PointCloud2 edge_cloud_msg;
            pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
            edge_cloud_msg.header.stamp = stamp;
            edge_cloud_msg.header.frame_id = frame_id;
            lidarDetectPtr->edge_pub_->publish(edge_cloud_msg);

            sensor_msgs::msg::PointCloud2 lidar_centers_z0_msg;
            pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), lidar_centers_z0_msg);
            lidar_centers_z0_msg.header.stamp = stamp;
            lidar_centers_z0_msg.header.frame_id = frame_id;
            lidarDetectPtr->center_z0_pub_->publish(lidar_centers_z0_msg);

            sensor_msgs::msg::PointCloud2 aligned_lidar_centers_msg;
            pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
            aligned_lidar_centers_msg.header.stamp = stamp;
            aligned_lidar_centers_msg.header.frame_id = frame_id;
            aligned_lidar_centers_pub->publish(aligned_lidar_centers_msg);

            sensor_msgs::msg::PointCloud2 colored_cloud_msg;
            pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
            colored_cloud_msg.header.stamp = stamp;
            colored_cloud_msg.header.frame_id = frame_id;
            colored_cloud_pub->publish(colored_cloud_msg);
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

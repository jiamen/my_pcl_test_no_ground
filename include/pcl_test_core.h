//
// Created by zlc on 2021/3/13.
//

#ifndef _MY_PCL_TEST_NO_GROUND_PCL_TEST_CORE_H_
#define _MY_PCL_TEST_NO_GROUND_PCL_TEST_CORE_H_

#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>


#define CLIP_HEIGHT 0.2
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18   // 径向切割器角度 0.18度是VLP32C雷达的水平光束发散间隔。
#define SENSOR_HEIGHT 1.78          // SENSOR_HEIGHT表示lidar挂载的高度，-SNESOR_HEIGHT即表示水平地面。


#define concentric_divider_distance_ 0.01   // 0.1 meters default  距离最小值
#define min_height_threshold_ 0.05
#define local_max_slope_ 8      // max slope of the ground between points, degree  同条射线上邻近两点的坡度阈值
#define general_max_slope_ 5    // max slope of the ground in entire point 整个地面的坡度阈值，这两个坡度阈值的单位为度（degree）

#define reclass_distance_threshold_ 0.2


class PclTestCore
{
private:
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_ground_, pub_no_ground_;

    struct PointXYZIRTColor
    {
        pcl::PointXYZI point;

        float radius;   // cylindric coords on XY plane  XY平面上的半径
        float theta;    // angle deg on XY plane         XY平面上的角度，与车辆前行方向（X方向）的夹角

        size_t radial_div;      // index of the radial division to which this point belongs to
        size_t concentric_div;  // index of the concentric division to which this points belongs to

        size_t original_index;  // index of this point in the source pointcloud
    };

    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

    size_t radial_dividers_num_;        // 角度微分，将360°进行细微分割，得到2000条射线
    size_t concentric_dividers_num_;    // 距离微分

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr);

    void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                          PointCloudXYZIRTColor& out_organized_points,
                          std::vector<pcl::PointIndices>& out_radial_divided_indices,
                          std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds);

    void classify_pc(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
                     pcl::PointIndices& out_ground_indices,
                     pcl::PointIndices& out_no_ground_indices);

    void publish_cloud(const ros::Publisher& in_publisher,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                       const std_msgs::Header& in_header);

public:
    PclTestCore(ros::NodeHandle& nh);
    ~PclTestCore();

    void Spin();
};


#endif // _MY_PCL_TEST_NO_GROUND_PCL_TEST_CORE_H_

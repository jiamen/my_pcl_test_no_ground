//
// Created by zlc on 2021/3/13.
//

#include "pcl_test_core.h"


PclTestCore::PclTestCore(ros::NodeHandle& nh)
{
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PclTestCore::point_cb, this);

    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);

    ros::spin();
}

PclTestCore::~PclTestCore() {  }

void PclTestCore::Spin()
{

}


// △△△ 主要流程 △△△
void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    // 1.信息转换 sensor_msgs ==> pcl::PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    // 2.去除过高点
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);

    // 3.去除太近的点
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);
    remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);

    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    // std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);        // 将360°进行细微分割，得到2000条射线

    // 4. 数据类型转换 PointXYZI ==>  PointCloudXYZIRTColor
    XYZI_to_RTZColor(remove_close, organized_points,
                  radial_division_indices, radial_ordered_clouds);

    // 5. 根据角度组织的射线排列方式的点集radial_ordered_clouds，区分地面点和非地面点
    pcl::PointIndices ground_indices, no_ground_indices;
    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(remove_close);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false);  // true removes the indices, false leaves only the indices
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true);   //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);


    publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_no_ground_,no_ground_cloud_ptr, in_cloud_ptr->header);
}


// 因为是分割地面，所以去除太高的点
// 要分割地面和非地面，那么过高的区域首先就可以忽略不计，
// 我们先对点云进行高度的裁剪。我们实验用的bag在录制的时候lidar的高度约为1.78米，我们剪裁掉1.28米以上的部分
void PclTestCore::clip_above(double clip_height,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;

#pragma omp for
    for( size_t i=0; i<in->points.size(); i ++ )
    {
        if( in->points[i].z > clip_height )
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*out);
}


// 去除太近的点
void PclTestCore::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}


// 为了方便地对点进行半径和夹角的表示，我们使用PointCloudXYZIRTColor代替pcl::PointCloudXYZI:
void PclTestCore::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   PointCloudXYZIRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for ( size_t i=0; i < in_cloud->points.size(); i ++ )
    {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
                in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y
        );      // 这里求的距离，实际就是半径

        // 弧度变角度
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if ( theta < 0 )
        {
            theta += 360;
        }

        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);  // 角度的微分描述,这里的微分是细微分割的意思
        auto concentric_div = (size_t)floor( fabs(radius/concentric_divider_distance_) );   // 半径的微分

        new_point.point  = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta  = theta;
        new_point.radial_div = radial_div;              // 描述角度微分
        new_point.concentric_div = concentric_div;      // 描述距离微分
        new_point.original_index = i;                   // 保留原始索引,后面会用到

        out_organized_points[i] = new_point;

        // radial divisions 使用角度的微分组成一条射线，序号表示哪些点在radial_div角度的射线上
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);
    }   // end for

    // 将同一根射线上的点按照半径（距离）排序
#pragma omp for
    // 其中，#pragma omp for语法OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速的效果。
    for ( size_t i=0; i < radial_dividers_num_; i ++ )
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor& a, const PointXYZIRTColor& b)
                  {
                        return a.radius < b.radius;
                  });
    }
}



void PclTestCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();         // 地面点
    out_no_ground_indices.indices.clear();      // 非地面点


#pragma omp for
    // sweep through each radial division 遍历每一根射线
    for( size_t i=0; i < in_radial_ordered_clouds.size(); i ++ )
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool  prev_ground = false;
        bool  current_ground = false;


        // 遍历每一个射线上的所有点
        for( size_t j=0; j<in_radial_ordered_clouds[i].size(); j ++ )
        {
            float points_distance  = in_radial_ordered_clouds[i][j].radius - prev_radius;   // 与前一个点的距离
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;      // 当前点的实际z值
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;


            // for points which are very close causing the height threshold to be tiny, set a minimum value
            // 对于非常接近导致高度阈值很小的点，请设置一个最小值
            if ( points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_ )
            {
                height_threshold = min_height_threshold_;
            }


            // check current point height against the LOCAL threshold (previous point)  对照局部阈值检查当前点高度（上一点）
            if ( current_height <= (prev_height+height_threshold) && current_height >= (prev_height-height_threshold) )
            {
                // check again using general geometry (radius from origin) if previous points wasn't ground
                if ( !prev_ground )     // 前一点不是地面点，需要进一步判断当前点
                {
                    if (current_height <= (-SENSOR_HEIGHT+general_height_threshold) && current_height >= (-SENSOR_HEIGHT-general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else    // 上一点是地面点，当前点在上一点阈值高度范围内，所以这一点一定是地面点
                {
                    current_ground = true;
                }
            }
            else
            {
                // check if previous point is too far from previous one, if so classify again
                // 检查当前点是否离上一个点太远，如果是，重新分类
                if ( points_distance > reclass_distance_threshold_ &&
                     (current_height <= (-SENSOR_HEIGHT+height_threshold) && current_height >= (-SENSOR_HEIGHT-height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }


            // 最终判断当前点是否为地面点
            if ( current_ground )
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }

}


// 发布点云
void PclTestCore::publish_cloud(const ros::Publisher& in_publisher,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header& in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    in_publisher.publish(cloud_msg);
}



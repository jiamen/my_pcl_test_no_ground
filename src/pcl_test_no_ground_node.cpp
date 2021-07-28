//
// Created by zlc on 2021/3/13.
//

#include "pcl_test_core.h"

int main(int argc, char* *argv)
{
    ros::init(argc, argv, "pcl_test_no_ground");

    ros::NodeHandle nh;

    PclTestCore core(nh);

    // core.Spin();

    return 0;
}

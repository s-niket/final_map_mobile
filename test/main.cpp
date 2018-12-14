#include <ros/ros.h>
#include <gtest/gtest.h>

/**
 * @brief      main
 *
 * @param  argc  The argc
 * @param  argv  The argv
 *
 * @return     0 if all tests pass
 */
int main(int argc, char** argv) {
//    ros::init(argc, argv, "Clean");
    ros::init(argc, argv, "map_mobile_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
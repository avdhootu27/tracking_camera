#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "example-utils.hpp"
#include <unistd.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

#define SAMPLES_FOR_VARIANCE 2000

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "tracking_camera");
    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/camera/odom", 10);

    ros::Rate loop_rate(1000);

    ROS_INFO("Connecting with T265...");

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    pipe.start(cfg);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_odom_frame";
    odom.child_frame_id = "base_link";
    int count = 1;

    // ---------- Calculating variances ----------
    ROS_INFO("Calculating variances...");
    std::vector<float> mean, variance;
    std::vector<std::vector<float>> dataForVariance;
    std::vector<float> data_for_variance;
    float m_pxx=0,m_pyy=0,m_pzz=0,m_oxx=0,m_oyy=0,m_ozz=0;
    float m_lxx=0,m_lyy=0,m_lzz=0,m_axx=0,m_ayy=0,m_azz=0;


    for(int i=0;i<12;i++){
        mean.push_back(0);
        variance.push_back(0);
    }

    for(int i=0;i<SAMPLES_FOR_VARIANCE;i++){
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::pose_frame pose_frame = frames.get_pose_frame();
        rs2_pose pose_data = pose_frame.get_pose_data();

        mean[0] += -pose_data.translation.z;
        mean[1] += -pose_data.translation.x;
        mean[2] += pose_data.translation.y;
        mean[3] += -pose_data.rotation.z;
        mean[4] += -pose_data.rotation.x;
        mean[5] += pose_data.rotation.y;

        mean[6] += -pose_data.velocity.z;
        mean[7] += -pose_data.velocity.x;
        mean[8] += pose_data.velocity.y;
        mean[9] += -pose_data.angular_velocity.z;
        mean[10] += -pose_data.angular_velocity.x;
        mean[11] += pose_data.angular_velocity.y;

        data_for_variance.clear();

        data_for_variance.push_back(-pose_data.translation.z);
        data_for_variance.push_back(-pose_data.translation.x);
        data_for_variance.push_back(pose_data.translation.y);
        data_for_variance.push_back(-pose_data.rotation.z);
        data_for_variance.push_back(-pose_data.rotation.x);
        data_for_variance.push_back(pose_data.rotation.y);

        data_for_variance.push_back(-pose_data.velocity.z);
        data_for_variance.push_back(-pose_data.velocity.x);
        data_for_variance.push_back(pose_data.velocity.y);
        data_for_variance.push_back(-pose_data.angular_velocity.z);
        data_for_variance.push_back(-pose_data.angular_velocity.x);
        data_for_variance.push_back(pose_data.angular_velocity.y);

        dataForVariance.push_back(data_for_variance);
    }
    
    mean[0] /= SAMPLES_FOR_VARIANCE;
    mean[1] /= SAMPLES_FOR_VARIANCE;
    mean[2] /= SAMPLES_FOR_VARIANCE;
    mean[3] /= SAMPLES_FOR_VARIANCE;
    mean[4] /= SAMPLES_FOR_VARIANCE;
    mean[5] /= SAMPLES_FOR_VARIANCE;

    mean[6] /= SAMPLES_FOR_VARIANCE;
    mean[7] /= SAMPLES_FOR_VARIANCE;
    mean[8] /= SAMPLES_FOR_VARIANCE;
    mean[9] /= SAMPLES_FOR_VARIANCE;
    mean[10] /= SAMPLES_FOR_VARIANCE;
    mean[11] /= SAMPLES_FOR_VARIANCE;

    for(int i=0;i<SAMPLES_FOR_VARIANCE;i++){
        for(int j=0;j<12;j++){
            variance[j] += pow(dataForVariance[i][j] - mean[j], 2);
        }
    }
    for(int j=0;j<12;j++){
        variance[j] /= (SAMPLES_FOR_VARIANCE-1);
    }

    odom.pose.covariance[0] = variance[0];
    odom.pose.covariance[7] = variance[1];
    odom.pose.covariance[14] = variance[2];
    odom.pose.covariance[21] = variance[3];
    odom.pose.covariance[28] = variance[4];
    odom.pose.covariance[35] = variance[5];

    odom.twist.covariance[0] = variance[6];
    odom.twist.covariance[7] = variance[7];
    odom.twist.covariance[14] = variance[8];
    odom.twist.covariance[21] = variance[9];
    odom.twist.covariance[28] = variance[10];
    odom.twist.covariance[35] = variance[11];

    dataForVariance.clear();
    data_for_variance.clear();
    mean.clear();
    variance.clear();
    // ---------- Variances Calculated ----------

    ROS_INFO("Publishing on topic /camera/odom");
    while (ros::ok()){
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::pose_frame pose_frame = frames.get_pose_frame();
        rs2_pose pose_data = pose_frame.get_pose_data();

        odom.header.seq = count;
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = -pose_data.translation.z;
        odom.pose.pose.position.y = -pose_data.translation.x;
        odom.pose.pose.position.z = pose_data.translation.y;
        odom.pose.pose.orientation.x = -pose_data.rotation.z;
        odom.pose.pose.orientation.y = -pose_data.rotation.x;
        odom.pose.pose.orientation.z = pose_data.rotation.y;
        odom.pose.pose.orientation.w = pose_data.rotation.w;
        odom.twist.twist.linear.x = -pose_data.velocity.z;
        odom.twist.twist.linear.y = -pose_data.velocity.x;
        odom.twist.twist.linear.z = pose_data.velocity.y;
        odom.twist.twist.angular.x = -pose_data.angular_velocity.z;
        odom.twist.twist.angular.y = -pose_data.angular_velocity.x;
        odom.twist.twist.angular.z = pose_data.angular_velocity.y;

        odom_pub.publish(odom);

        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
// g++ tracking.cpp -o track -lrealsense2
// sudo ./track
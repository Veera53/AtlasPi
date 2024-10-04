#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include "hector_slam_lib/MapRepresentationInterface.h"
#include "hector_slam_lib/HectorSlamProcessor.h"

class HectorMappingNode {
public:
    HectorMappingNode()
        : slamProcessor(0.05f, 50.0f, Eigen::Vector2i(1024, 1024), Eigen::Vector2f(50.0f, 50.0f)) {
        
        ros::NodeHandle nh;

        // Subscriber to LiDAR scan topic
        laserScanSubscriber = nh.subscribe("/scan", 1, &HectorMappingNode::laserScanCallback, this);
        
        // Publisher for occupancy grid (map)
        mapPublisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        
        // Publisher for pose
        posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);

        // Initialize map message
        map.header.frame_id = "map";
        map.info.resolution = 0.05f;
        map.info.width = 1024;
        map.info.height = 1024;
        map.info.origin.position.x = -25.0;
        map.info.origin.position.y = -25.0;
        map.data.resize(map.info.width * map.info.height, -1);
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        Eigen::Vector3f poseEstimate(0.0f, 0.0f, 0.0f); // Initial pose estimate
        
        slamProcessor.update(scan->ranges, scan->angle_min, scan->angle_increment, poseEstimate);

        // Update map
        updateMap();

        // Publish map and pose
        publishMap();
        publishPose();
    }

private:
    ros::Subscriber laserScanSubscriber;
    ros::Publisher mapPublisher;
    ros::Publisher posePublisher;

    nav_msgs::OccupancyGrid map;

    HectorSlamProcessor slamProcessor;

    void updateMap() {
        const auto& mapRep = slamProcessor.getMap();
        for (int y = 0; y < map.info.height; ++y) {
            for (int x = 0; x < map.info.width; ++x) {
                int index = x + y * map.info.width;
                map.data[index] = mapRep.getGridValue(x, y);
            }
        }
    }

    void publishMap() {
        map.header.stamp = ros::Time::now();
        mapPublisher.publish(map);
    }

    void publishPose() {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.header.frame_id = "map";

        const Eigen::Vector3f& poseEstimate = slamProcessor.getLastPose();
        poseMsg.pose.position.x = poseEstimate[0];
        poseMsg.pose.position.y = poseEstimate[1];
        poseMsg.pose.orientation.w = cos(poseEstimate[2] * 0.5f);
        poseMsg.pose.orientation.z = sin(poseEstimate[2] * 0.5f);

        posePublisher.publish(poseMsg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hector_mapping_node");

    HectorMappingNode hectorMapping;

    ros::spin();

    return 0;
}

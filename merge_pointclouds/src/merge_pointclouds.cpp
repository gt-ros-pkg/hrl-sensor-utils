#include <vector>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include <merge_pointclouds/GetMergedPointcloud.h>

typedef pcl::PointXYZRGB PointT;

class MergePointClouds
{
    std::string merge_frame, node_name;
    float min_sample_distance_, max_correspondence_distance_;
    int pc_count;
    bool run_scan;
    ros::ServiceServer triggerMergeService, returnMergedService;
    pcl::PointCloud<PointT>::Ptr incoming_pc, merged_pc;
    tf::TransformListener *tf_listener;
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    pcl::StatisticalOutlierRemoval<PointT> *outlier_filter;
    pcl::PassThrough<PointT> *passthrough_filter;
    pcl::VoxelGrid<PointT> *voxel_grid_filter;
    pcl::CropBox<PointT> *crop_box_filter;
    ros::Timer limitingTimer;

    public:
        MergePointClouds(): nh("~"),
                            tf_listener(new tf::TransformListener),
                            incoming_pc(new pcl::PointCloud<PointT>),
                            merged_pc(new pcl::PointCloud<PointT>),
                            pc_count(0),
                            run_scan(false),
                            outlier_filter(new pcl::StatisticalOutlierRemoval<PointT>),
                            passthrough_filter(new pcl::PassThrough<PointT>),
                            voxel_grid_filter(new pcl::VoxelGrid<PointT>),
                            crop_box_filter(new pcl::CropBox<PointT>)
        {
            node_name = ros::this_node::getName();
            pc_sub = nh.subscribe("/head_mount_kinect/sd/points", 1, &MergePointClouds::pcCallback, this);
            pc_pub = nh.advertise< pcl::PointCloud<PointT> > ("merged_points", 1);
            nh.param<std::string>("merge_frame", merge_frame, "/base_footprint");
            triggerMergeService  = nh.advertiseService("new_merge", &MergePointClouds::triggerCallback, this);
            returnMergedService  = nh.advertiseService("get_merged_pointcloud", &MergePointClouds::returnMerged, this);
            voxel_grid_filter->setLeafSize(0.02f, 0.02f, 0.02f);
            outlier_filter->setMeanK(7);
            outlier_filter->setStddevMulThresh(0.1);
            crop_box_filter->setMin(Eigen::Vector4f(-0.5, -2.0, 0.1, 0.0));
            crop_box_filter->setMax(Eigen::Vector4f(2.0, 2.0, 2.0, 0.0));
            limitingTimer = nh.createTimer(ros::Duration(20), &MergePointClouds::stopMergeTimeout, this, false, false);
            ROS_INFO("[%s] Ready", node_name.c_str());
        };
        ~MergePointClouds(){};

        bool returnMerged(merge_pointclouds::GetMergedPointcloud::Request& request, merge_pointclouds::GetMergedPointcloud::Response& response) {
            stopMerge();
            sensor_msgs::PointCloud2 merged_pc_msg;
            pcl::toROSMsg(*merged_pc, merged_pc_msg);
            response.merged_pointcloud = merged_pc_msg;
            return true;
        };

        void stopMerge(){
            limitingTimer.stop();
            run_scan = false;  // Stop run
            ROS_INFO("[%s] Merged %i pointclouds", node_name.c_str(), pc_count);
            // Downsample via voxelGrid
            voxel_grid_filter->setInputCloud(merged_pc);
            voxel_grid_filter->filter(*merged_pc);
            // Remove statistical outliers
            outlier_filter->setInputCloud(merged_pc);
            outlier_filter->filter(*merged_pc);
            publishPointCloud(merged_pc);
        };

        void stopMergeTimeout(const ros::TimerEvent&) {
            ROS_INFO("[%s] Merge Stopping - Timed Out", node_name.c_str());
            stopMerge();
        };

        void publishPointCloud(pcl::PointCloud<PointT>::Ptr pc_out)
        {
            pc_out->header.frame_id = merge_frame;
            pc_pub.publish(pc_out);
            ROS_INFO("[%s] Publishing Merged Scan PointCloud", node_name.c_str());
        }

        bool triggerCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
            ROS_INFO("[%s] Starting Scan", node_name.c_str());
            pc_count = 0; // Reset count for new run
            merged_pc->clear();  // Reset for new run
            run_scan = true;
            limitingTimer.start();
            response.success = true;
            response.message = "Starting Scan";
            return true;
        }

        void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg)
        {
            // Only run when service is called to start run
            if (!run_scan) { return; };
            incoming_pc->clear();
            // Convert ros msg to pcl
            pcl::fromROSMsg(*pc_msg, *incoming_pc); 
            // Remove NAN's
            std::vector<int> inds;
            pcl::removeNaNFromPointCloud(*incoming_pc, *incoming_pc, inds);
            // Remove statistical outlier
            outlier_filter->setInputCloud(incoming_pc);
            outlier_filter->filter(*incoming_pc);
            // Transform to common/base frame based on tf
            try {
                tf_listener->waitForTransform("/base_link",
                                              (*pc_msg).header.frame_id, 
                                              (*pc_msg).header.stamp,
                                              ros::Duration(4.0));
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return;
            }
            pcl_ros::transformPointCloud("/base_link", *incoming_pc, *incoming_pc, *tf_listener);
            // Filter out points near robot (crude self-filter)
            crop_box_filter->setInputCloud(incoming_pc);
            crop_box_filter->filter(*incoming_pc);
            // Passthrough filters to remove far-away data
//            passthrough_filter->filter(*incoming_pc);
//            // Filter along X-axis
//            passthrough_filter->setFilterFieldName("x");
//            passthrough_filter->setFilterLimits(-0.5f, 2.0f);
//            passthrough_filter->setInputCloud(incoming_pc);
//            // Filter along Y-axis
//            passthrough_filter->setFilterFieldName("y");
//            passthrough_filter->setFilterLimits(-2.0f, 2.0f);
//            passthrough_filter->filter(*incoming_pc);
//            // Filter along Z-axis
//            passthrough_filter->setFilterFieldName("z");
//            passthrough_filter->setFilterLimits(0.05f, 1.8f);
//            passthrough_filter->filter(*incoming_pc);
            *merged_pc += *incoming_pc;
            pc_count += 1;
//            ROS_INFO("[%s] Merged pointcloud", node_name.c_str());
        };
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "merge_pointclouds");
    MergePointClouds merge_node;
    ros::spin();
    return 0;
};

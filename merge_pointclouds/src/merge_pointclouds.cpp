#include <limits>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PointT;

class MergePointClouds
{
    std::string merge_frame, node_name;
    float min_sample_distance_, max_correspondence_distance_;
    int pc_count;
    bool run_scan;
    ros::ServiceServer triggerMergeService;
    pcl::PointCloud<PointT>::Ptr incoming_pc, merged_pc;
    tf::TransformListener *tf_listener;
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    pcl::StatisticalOutlierRemoval<PointT> *outlier_filter;
    pcl::PassThrough<PointT> *passthrough_filter;
    pcl::VoxelGrid<PointT> *voxel_grid_filter;
    ros::Timer sendMergedTimer;

    public:
        MergePointClouds(): nh("~"),
                            tf_listener(new tf::TransformListener),
                            incoming_pc(new pcl::PointCloud<PointT>),
                            merged_pc(new pcl::PointCloud<PointT>),
                            pc_count(0),
                            run_scan(false),
                            outlier_filter(new pcl::StatisticalOutlierRemoval<PointT>),
                            passthrough_filter(new pcl::PassThrough<PointT>),
                            voxel_grid_filter(new pcl::VoxelGrid<PointT>)
        {
            node_name = ros::this_node::getName();
            pc_sub = nh.subscribe("/head_mount_kinect/sd/points", 1, &MergePointClouds::pcCallback, this);
            pc_pub = nh.advertise< pcl::PointCloud<PointT> > ("merged_points", 1);
            nh.param<std::string>("merge_frame", merge_frame, "/base_footprint");
            triggerMergeService  = nh.advertiseService("trigger_scan", &MergePointClouds::triggerScanCallback, this);
            voxel_grid_filter->setLeafSize(0.02f, 0.02f, 0.02f);
            outlier_filter->setMeanK(6);
            outlier_filter->setStddevMulThresh(0.3);
            ROS_INFO("[%s] Ready", node_name.c_str());
        };
        ~MergePointClouds(){};

        void sendMerged(const ros::TimerEvent&) {
            ROS_INFO("[%s] Scan Complete: Merged %i pointclouds", node_name.c_str(), pc_count);
            run_scan = false;  // Stop run
            pc_count = 0; // Reset count for next run

            // Downsample via voxelGrid
            voxel_grid_filter->setInputCloud(merged_pc);
            voxel_grid_filter->filter(*merged_pc);

            publishPointCloud(merged_pc);  // Publish results
            merged_pc->clear();
        };

        void publishPointCloud(pcl::PointCloud<PointT>::Ptr pc_out)
        {
            pc_out->header.frame_id = merge_frame;
            pc_pub.publish(pc_out);
            ROS_INFO("[%s] Publishing Merged Scan PointCloud", node_name.c_str());
        }

        bool triggerScanCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
            ROS_INFO("[%s] Starting Scan", node_name.c_str());
            run_scan = true;
            sendMergedTimer = nh.createTimer(ros::Duration(15), &MergePointClouds::sendMerged, this, true);
            response.success = true;
            response.message = "Starting Scan";
            return true;
        }

        void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg)
        {
            // Only run when service is called to start run
            if (!run_scan) { return; };
            // Convert ros msg to pcl
            pcl::fromROSMsg(*pc_msg, *incoming_pc); 
            // Remove NAN's
            std::vector<int> inds;
            pcl::removeNaNFromPointCloud(*incoming_pc, *incoming_pc, inds);
            // Transform to common/base frame based on tf
            try {
                tf_listener->waitForTransform("/base_link",
                                              (*pc_msg).header.frame_id, 
                                              (*pc_msg).header.stamp,
                                              ros::Duration(10.0));
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return;
            }
            pcl_ros::transformPointCloud("/base_link", *incoming_pc, *incoming_pc, *tf_listener);
            passthrough_filter->filter(*incoming_pc);
            // Filter along X-axis
            passthrough_filter->setFilterFieldName("x");
            passthrough_filter->setFilterLimits(-1.0f, 3.0f);
            passthrough_filter->setInputCloud(incoming_pc);
            // Filter along Y-axis
            passthrough_filter->setFilterFieldName("y");
            passthrough_filter->setFilterLimits(-3.0f, 3.0f);
            passthrough_filter->filter(*incoming_pc);
            // Filter along Z-axis
            passthrough_filter->setFilterFieldName("z");
            passthrough_filter->setFilterLimits(-0.1f, 2.0f);
            passthrough_filter->filter(*incoming_pc);
            // Downsample via voxelGrid
            voxel_grid_filter->setInputCloud(incoming_pc);
            voxel_grid_filter->filter(*incoming_pc);
            // Remove statistical outliers
            outlier_filter->setInputCloud(incoming_pc);
            outlier_filter->filter(*incoming_pc);
            *merged_pc += *incoming_pc;
            pc_count += 1;
            ROS_INFO("[%s] Merged pointcloud", node_name.c_str());
        }

/*        void filterTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
        {
            //Preprocess cloud by removing distant points
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(target_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.3, 2.0);
            pass.filter(*target_cloud);

            //downsample pointcloud
            pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
            vox_grid.setInputCloud(target_cloud);
            vox_grid.setLeafSize(0.005f,0.005f, 0.005f);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
            vox_grid.filter(*tempCloud);
            *target_cloud = *tempCloud;
        }

        void align(FeatureCloud target_cloud, FeatureCloud template_cloud)
        {
            sac_ia_.setInputTarget(target_cloud.getPointCloud());
            sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());

            sac_ia_.setInputCloud(template_cloud.getPointCloud());
            sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());
            pcl::PointCloud<pcl::PointXYZ> registration_output;
            sac_ia_.align(registration_output);

            float fitness_score = (float) sac_ia_.getFitnessScore(max_correspondence_distance_);
            Eigen::Matrix4f final_transformation = sac_ia_.getFinalTransformation();
            //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            //Print the alignment fitness score (values < 0.00002 are good)
            printf("Fitness Score: %f\n", fitness_score);

            //Print rotation matrix and trans vector
            Eigen::Matrix3f rotation = final_transformation.block<3,3>(0,0);
            Eigen::Vector3f translation = final_transformation.block<3,1>(0,3);

            printf("\n");
            printf("    | %6.3f %6.3f %6.3f | \n", rotation(0,0), rotation(0,1), rotation(0,2));
            printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1,0), rotation(1,1), rotation(1,2));
            printf("    | %6.3f %6.3f %6.3f | \n", rotation(2,0), rotation(2,1), rotation(2,2));
            printf("\n");
            printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*template_cloud.getPointCloud(), *transformed_cloud, final_transformation);
            publishPointCloud(transformed_cloud);
        }
*/
};

/*
#include <Eigen/Core>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};

*/

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */

/*
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }


    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.1);  
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    p->removePointCloud ("source");
    p->removePointCloud ("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO ("Press q to continue the registration.\n");
    p->spin ();

    p->removePointCloud ("source"); 
    p->removePointCloud ("target");

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}
*/
int main (int argc, char** argv)
{

    //Initialize ROS
    ros::init(argc, argv, "merge_pointclouds");
    MergePointClouds merge_node;

/*
    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

    for (size_t i = 1; i < data.size (); ++i)
    {
        source = data[i-1].cloud;
        target = data[i].cloud;

        // Add visualization data
        showCloudsLeft(source, target);

        PointCloud::Ptr temp (new PointCloud);
        PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
        pairAlign (source, target, temp, pairTransform, true);

        //transform current pair into the global transform
        pcl::transformPointCloud (*temp, *result, GlobalTransform);

        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform;

        //save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
        ss << i << ".pcd";
        pcl::io::savePCDFile (ss.str (), *result, true);

    }*/
    ros::spin();
    return 0;
};


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>


#include <vector>
#include <map>

// include eigen
#include <Eigen/Eigen>

#include <actionlib/server/simple_action_server.h>
#include <object_detector_3d/ShelvePlacePositionAction.h>
#include <object_detector_3d/Clustering.h>
#include <object_detector_3d/ShelvePlacePositionAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <limits>
#include <set>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace octomap;

#define ENABLE_RANSAC true

struct PointXYZComparator
{
    bool operator()(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs) const
    {
        if (lhs.x < rhs.x)
            return true;
        if (lhs.x > rhs.x)
            return false;
        if (lhs.y < rhs.y)
            return true;
        if (lhs.y > rhs.y)
            return false;
        if (lhs.z < rhs.z)
            return true;
        return false;
    }
};

struct PlaneParams {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  float mean_z;
  float min_z;
  float max_z;
  int n_objects;

  bool operator() (PlaneParams i, PlaneParams j){return (i.mean_z < j.mean_z);}
  bool operator()(PlaneParams i, float mean_z ){ return i.mean_z < mean_z; } 
};


struct compare_z {
  bool operator()(pcl::PointXYZ a, pcl::PointXYZ b) {
    return a.z < b.z;
  }
};


struct ObjectParams
{
  /* PointCloud Cluster. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
  /* World Z Min Value. */
  float min_z;
  /* World Z Max Value. */
  float max_z;
};


class Detect3DPlace
{
    std::string POINT_CLOUD_TOPIC = std::string("/zed2/zed_node/point_cloud/ds_cloud_registered");
    std::string BASE_FRAME = std::string("base_footprint");
    std::string CAMERA_FRAME = std::string("Cam1");
    
    const std::string name = "Detect3DPlace";
    ros::NodeHandle nh_;

    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;
    actionlib::SimpleActionServer<object_detector_3d::ShelvePlacePositionAction> as_;
    object_detector_3d::ShelvePlacePositionFeedback feedback_;
    object_detector_3d::ShelvePlacePositionResult result_;
    
    // K Means clustering service
    ros::ServiceClient client_;
    object_detector_3d::Clustering cluster_srv_;
    
    bool ignore_moveit_ = true;
    float plane_min_height_ = 0.2;
    float plane_max_height_ = 1.5;
    float target_height_;
    int n_objects_;

    float kRadiusThreshold = 1; 
    int kDebugNumShelves = 5;
    float kDebugShelfHeight = 0.3;
    float kSelfThreshold = 0.2;
    float kAngle = 12.0;
    int n_directions = 2;
    bool debug = true;
    
    bool using_clustering;
    bool using_octomap;

    std::vector<float> excluded_heights_;

public:
    Detect3DPlace() : 
        listener_(buffer_),
        as_(nh_, "detect3d_place_shelves", boost::bind(&Detect3DPlace::handleActionServer, this, _1), false)
    {   
        ros::param::param<bool>("/USING_CLUSTERING", using_clustering, true);
        ros::param::param<bool>("/USING_OCTOMAP", using_octomap, true);

        planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();
        client_ = nh_.serviceClient<object_detector_3d::Clustering>("Clustering");

        tf_listener = new tf::TransformListener();
        as_.start();
        ROS_INFO("Waiting for Clear Octomap service to start.");
        ROS_INFO_STREAM("Action Server Detect3D - Initialized");
        // Load Params.
        nh_.param("/Detection3D/BASE_FRAME", BASE_FRAME, BASE_FRAME);
        nh_.param("/Detection3D/CAMERA_FRAME", CAMERA_FRAME, CAMERA_FRAME);
        nh_.param("/Detection3D/POINT_CLOUD_TOPIC", POINT_CLOUD_TOPIC, POINT_CLOUD_TOPIC);
        ROS_INFO_STREAM("BASE_FRAME: " << BASE_FRAME);
        ROS_INFO_STREAM("CAMERA_FRAME: " << CAMERA_FRAME);
        ROS_INFO_STREAM("POINT_CLOUD_TOPIC: " << POINT_CLOUD_TOPIC);
    }

    /** \brief Handle Action Server Goal Received. */
    void handleActionServer(const object_detector_3d::ShelvePlacePositionGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("Action Server Detect3D - Goal Received");
        ignore_moveit_ = goal->ignore_moveit;
        target_height_ = goal->target_height;
        n_objects_ = goal->n_objects;
        
        int size = sizeof(goal->excluded_heights);
        for (int i = 0; i < size; i++){
            excluded_heights_.push_back(goal->excluded_heights[i]);
        }
        ROS_INFO_STREAM("Ignore Moveit: " << ignore_moveit_);
        feedback_.status = 0;
        result_.success = true;
        result_.target_pose = geometry_msgs::PoseStamped();
        result_.height_plane = 0;

        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO_STREAM("Action Server Detect3D - Preempted");
            as_.setPreempted(); // Set the action state to preempted
            return;
        }

        // Get PointCloud and Transform it to Map Frame
        sensor_msgs::PointCloud2 pc;
        sensor_msgs::PointCloud2 t_pc;
        pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, nh_));
        if (pc.header.frame_id != BASE_FRAME && tf_listener->canTransform(BASE_FRAME, CAMERA_FRAME, ros::Time(0)))
        {
            tf_listener->waitForTransform(BASE_FRAME, CAMERA_FRAME, ros::Time(0), ros::Duration(5.0));
            pc.header.frame_id = CAMERA_FRAME;
            pcl_ros::transformPointCloud(BASE_FRAME, pc, t_pc, *tf_listener);
        }
        else
        {
            t_pc = pc;
        }

        Detect3DPlace::cloudCB(t_pc);
        as_.setSucceeded(result_);
    }

    void removePerpendiculaPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& coefficients) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Eigen::Vector3f axis = Eigen::Vector3f(x, y, z);
        //SampleConsensusModelPerpendicularPlane
        
        seg.setOptimizeCoefficients (true);
        seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
        seg.setEpsAngle(kAngle * (3.1415 / 180.0f));
        seg.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);

        for (int i = 0; i < n_directions; i++) {
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);

            if (inliers->indices.size () == 0){
            ROS_INFO_STREAM("Could not estimate a parallel model for the given dataset.");
            return;
            }
            
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_filtered);
        
            cloud->swap(*cloud_filtered);
        }
    }


    /** \brief  Generate an achiveable PCL by checking magnitude of X,Y coordinates*/
    void generateAchievablePCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &achievable_cloud){
        // Create new cloud only with the points with a z value greater than 0.5
        for (size_t i = 0; i < cloud->points.size(); i++) {
            float magnitude = sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2));
            if (cloud->points[i].z > plane_min_height_ && cloud->points[i].z < plane_max_height_ && magnitude <= kRadiusThreshold) {
            achievable_cloud->points.push_back(cloud->points[i]);
            if (debug){
                pcl::PointXYZ p = cloud->points[i];
                float z = p.z;
                for (int j = 1; j <= kDebugNumShelves; j++) {
                p.z = z + kDebugShelfHeight * j;
                achievable_cloud->points.push_back(p);
                }
            }
            }
        }

        achievable_cloud->width = achievable_cloud->points.size();
        achievable_cloud->height = 1;


        pcl::io::savePCDFile("pcl_achievable_cloud.pcd", *achievable_cloud);
        ROS_INFO_STREAM("Achievable cloud saved");
    }

        /** \brief Find all clusters in a pointcloud.*/
    void removeClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        //Set parameters for the clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01); // 3cm
        ec.setMinClusterSize(25);
        ec.setMaxClusterSize(20000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            ObjectParams object;
            object.cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            object.min_z = cloud->points[it->indices[0]].z;
            object.max_z = cloud->points[it->indices[0]].z;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            object.cluster->points.push_back(cloud->points[*pit]);
            object.min_z = std::min(object.min_z, cloud->points[*pit].z);
            object.max_z = std::max(object.max_z, cloud->points[*pit].z);
            }
            

            // Discard Noise
            if (object.cluster->points.size() < 25){
            continue; 
            }
            object.cluster->width = object.cluster->points.size();
            object.cluster->height = 1;
            object.cluster->is_dense = true;
        }
    }

    int meanShelvesCounter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
        int num_shelves = 1;
        float sum = cloud->points[0].z;
        float mean = sum;
        int count = 1;
        for (size_t i = 1; i < cloud->points.size(); i++) {
            if (cloud->points[i].z - mean > kSelfThreshold) {
            num_shelves++;
            mean = 0;
            sum = 0;
            count = 1;
            } 

            sum += cloud->points[i].z;
            mean = sum / count;

            count++;
        }

        ROS_INFO_STREAM("Number of Shelves: " << num_shelves);

        return num_shelves;
    }


    void getShelves(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::map<float, PlaneParams>& planes) {
        float sum = 0;
        PlaneParams plane;
        plane.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        plane.cloud->points.push_back(cloud->points[0]);
        plane.min_z = cloud->points[0].z;
        plane.max_z = cloud->points[0].z;
        plane.n_objects = 1;

        for (size_t i = 1; i < cloud->points.size(); i++) {
            plane.cloud->points.push_back(cloud->points[i]);
            plane.min_z = std::min(plane.min_z, cloud->points[i].z);
            plane.max_z = std::max(plane.max_z, cloud->points[i].z);
            sum += cloud->points[i].z;
        }

        plane.mean_z = sum / plane.cloud->points.size();
        planes[plane.mean_z] = plane;
    }

    /** \brief PointCloud callback. */
    void cloudCB(const sensor_msgs::PointCloud2 &input){
        ROS_INFO_STREAM("Received PointCloud");
        if (!ignore_moveit_){
            // Reset Planning Scene Interface
            std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();
            planning_scene_interface_->removeCollisionObjects(object_ids);
            ros::Duration(2).sleep();
        }

        // Get cloud ready
        pcl::PCLPointCloud2::Ptr pcl2 (new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2::Ptr pcl_filtered_blob (new pcl::PCLPointCloud2);
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // Indices that correspond to a plane.
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); // Coefficients of the plane.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>()), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr achievable_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl_conversions::toPCL(input, *pcl2);
        pcl::PCDWriter writer;
        sor.setInputCloud (pcl2);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*pcl_filtered_blob);
        pcl::fromPCLPointCloud2(*pcl_filtered_blob, *cloud);

        generateAchievablePCL(cloud, achievable_cloud);
        removePerpendiculaPlane(achievable_cloud, coefficients);

        std::sort(achievable_cloud->points.begin(), achievable_cloud->points.end(), compare_z());

        ROS_INFO_STREAM("PointCloud Pre-Processing Done");

        writer.write<pcl::PointXYZ>("pcl_achievable_cloud.pcd", *achievable_cloud, false);
        
        if (achievable_cloud->points.size() == 0) {
            ROS_INFO_STREAM("No points in the cloud");
            return;
        }

        removeClusters(achievable_cloud);

        int num_shelves = meanShelvesCounter(achievable_cloud);

        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);

        // Detect the plane of the table
        std::map<float, PlaneParams> plane_map;
        int i = 0, nr_points = (int) achievable_cloud->size ();

        // While 30% of the original cloud is still there
        while (achievable_cloud->size () > 0.05 * nr_points && num_shelves--)
        {
            //Segment the largest planar component from the remaining cloud
            seg.setInputCloud (achievable_cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                ROS_INFO_STREAM("Could not estimate a planar model for the given dataset.");
                break;
            }

            // Extract the inliers
            extract.setInputCloud (achievable_cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);

            getShelves(cloud_p, plane_map);

            ROS_INFO_STREAM("Plane found with " << cloud_p->points.size() << " points");
            

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            achievable_cloud.swap (cloud_f);

            i++;
        }
    
        if (cloud->points.size() <= 3){
            return;
        }

        std::map<float, PlaneParams>::iterator it;
        if (target_height_ != -1) {
            it = plane_map.lower_bound(target_height_);

            if (it == plane_map.end()){
                it--;
            }
        } else if (excluded_heights_.size() > 0){
            for (int i = 0; i < excluded_heights_.size(); i++){
                // erase lower bound of excluded heights
                auto erase_it = plane_map.lower_bound(excluded_heights_[i]);
                plane_map.erase(erase_it);
            }
            it = plane_map.begin();
        }
        

        ROS_INFO("Target Height: %f Mean height: %f", target_height_, it->first);

        sensor_msgs::PointCloud2 achievable_cloud_msg;
        pcl::toROSMsg(*it->second.cloud, achievable_cloud_msg);
        achievable_cloud_msg.header.frame_id = "table_frame";
        achievable_cloud_msg.header.stamp = ros::Time::now();
        ROS_INFO_STREAM("Converted pcl to sensor_msgs::PC2");

        cluster_srv_.request.pointcloud = achievable_cloud_msg;
        cluster_srv_.request.n_clusters = n_objects_;

        if (client_.call(cluster_srv_)){
            ROS_INFO_STREAM("Cluster Service Called success");

            geometry_msgs::PoseStamped target_pose;
            target_pose.header.stamp = ros::Time::now();
            target_pose.header.frame_id = BASE_FRAME;
            target_pose.pose.position.x = cluster_srv_.response.x_center;
            target_pose.pose.position.y = cluster_srv_.response.y_center;
            target_pose.pose.position.z = 0;
            
            result_.height_plane = it->first;
            result_.target_pose = target_pose;
        } else {
            ROS_ERROR("Failed to call service Clustering");
        }
        return;
    }

private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf::TransformListener *tf_listener;
};

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "detector_place_position");

    // Start the segmentor
    Detect3DPlace segmentor_place;

    // Spin
    ros::spin();
}
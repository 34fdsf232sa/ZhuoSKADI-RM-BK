/**
 * @file pointcloud_fusion_node.cpp
 * @brief Multi-LiDAR Point Cloud Fusion Node
 * 
 * Fuses point clouds from:
 * - Unitree L2 (360° LiDAR)
 * - Livox Mid-70 (Forward LiDAR)
 * - Berxel P100R (RGB-D depth to pointcloud)
 * 
 * Features:
 * - TF-based coordinate transformation
 * - Time synchronization with message_filters
 * - VoxelGrid downsampling
 * - Statistical outlier removal
 * - Ground segmentation (optional)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

class PointCloudFusionNode : public rclcpp::Node
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::PointCloud2>;

    PointCloudFusionNode() : Node("pointcloud_fusion_node")
    {
        // Declare parameters
        declare_parameters();
        
        // Get parameters
        get_parameters();
        
        // TF Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize subscribers with message_filters
        if (use_sync_) {
            init_synchronized_subscribers();
        } else {
            init_individual_subscribers();
        }
        
        // Publisher
        fused_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            fused_topic_, rclcpp::SensorDataQoS());
        
        // Timer for processing (if not using sync)
        if (!use_sync_) {
            process_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
                std::bind(&PointCloudFusionNode::process_and_publish, this));
        }
        
        RCLCPP_INFO(this->get_logger(), "PointCloud Fusion Node initialized");
        RCLCPP_INFO(this->get_logger(), "  L2 topic: %s", l2_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Mid70 topic: %s", mid70_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth topic: %s", depth_pc_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", fused_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Target frame: %s", target_frame_.c_str());
    }

private:
    void declare_parameters()
    {
        // Topic names
        this->declare_parameter("l2_topic", "/unitree_l2/pointcloud");
        this->declare_parameter("mid70_topic", "/livox/lidar");
        this->declare_parameter("depth_pc_topic", "/berxel/depth/points");
        this->declare_parameter("fused_topic", "/fused_pointcloud");
        
        // Frame IDs
        this->declare_parameter("target_frame", "base_link");
        
        // Processing parameters
        this->declare_parameter("use_sync", true);
        this->declare_parameter("sync_queue_size", 10);
        this->declare_parameter("sync_slop", 0.1);  // seconds
        this->declare_parameter("publish_rate", 20.0);  // Hz
        
        // Voxel filter
        this->declare_parameter("voxel_size", 0.05);  // meters
        this->declare_parameter("enable_voxel_filter", true);
        
        // Outlier removal
        this->declare_parameter("enable_outlier_removal", true);
        this->declare_parameter("outlier_mean_k", 50);
        this->declare_parameter("outlier_stddev", 1.0);
        
        // Range filter
        this->declare_parameter("enable_range_filter", true);
        this->declare_parameter("min_range", 0.3);  // meters
        this->declare_parameter("max_range", 15.0);  // meters
        
        // Ground removal
        this->declare_parameter("enable_ground_removal", false);
        this->declare_parameter("ground_threshold", 0.2);  // meters
        
        // Height filter
        this->declare_parameter("enable_height_filter", true);
        this->declare_parameter("min_height", -0.5);  // meters relative to base_link
        this->declare_parameter("max_height", 2.5);  // meters

        // Self-filter (remove robot body points)
        this->declare_parameter("enable_self_filter", true);
        this->declare_parameter("self_filter_min_x", -0.25);
        this->declare_parameter("self_filter_max_x", 0.25);
        this->declare_parameter("self_filter_min_y", -0.20);
        this->declare_parameter("self_filter_max_y", 0.20);
        this->declare_parameter("self_filter_min_z", -0.30);
        this->declare_parameter("self_filter_max_z", 0.60);
    }
    
    void get_parameters()
    {
        // Topics
        l2_topic_ = this->get_parameter("l2_topic").as_string();
        mid70_topic_ = this->get_parameter("mid70_topic").as_string();
        depth_pc_topic_ = this->get_parameter("depth_pc_topic").as_string();
        fused_topic_ = this->get_parameter("fused_topic").as_string();
        
        // Frames
        target_frame_ = this->get_parameter("target_frame").as_string();
        
        // Processing
        use_sync_ = this->get_parameter("use_sync").as_bool();
        sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();
        sync_slop_ = this->get_parameter("sync_slop").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        
        // Filters
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        enable_voxel_filter_ = this->get_parameter("enable_voxel_filter").as_bool();
        
        enable_outlier_removal_ = this->get_parameter("enable_outlier_removal").as_bool();
        outlier_mean_k_ = this->get_parameter("outlier_mean_k").as_int();
        outlier_stddev_ = this->get_parameter("outlier_stddev").as_double();
        
        enable_range_filter_ = this->get_parameter("enable_range_filter").as_bool();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        
        enable_ground_removal_ = this->get_parameter("enable_ground_removal").as_bool();
        ground_threshold_ = this->get_parameter("ground_threshold").as_double();
        
        enable_height_filter_ = this->get_parameter("enable_height_filter").as_bool();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();

        enable_self_filter_ = this->get_parameter("enable_self_filter").as_bool();
        self_filter_min_x_ = this->get_parameter("self_filter_min_x").as_double();
        self_filter_max_x_ = this->get_parameter("self_filter_max_x").as_double();
        self_filter_min_y_ = this->get_parameter("self_filter_min_y").as_double();
        self_filter_max_y_ = this->get_parameter("self_filter_max_y").as_double();
        self_filter_min_z_ = this->get_parameter("self_filter_min_z").as_double();
        self_filter_max_z_ = this->get_parameter("self_filter_max_z").as_double();
    }
    
    void init_synchronized_subscribers()
    {
        // Message filter subscribers
        l2_sub_.subscribe(this, l2_topic_, rmw_qos_profile_sensor_data);
        mid70_sub_.subscribe(this, mid70_topic_, rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, depth_pc_topic_, rmw_qos_profile_sensor_data);
        
        // Synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(sync_queue_size_), l2_sub_, mid70_sub_, depth_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_));
        sync_->registerCallback(std::bind(
            &PointCloudFusionNode::synchronized_callback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        
        RCLCPP_INFO(this->get_logger(), "Using synchronized subscribers");
    }
    
    void init_individual_subscribers()
    {
        // Individual subscribers (for asynchronous fusion)
        l2_ind_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            l2_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                latest_l2_cloud_ = msg;
            });
        
        mid70_ind_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            mid70_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                latest_mid70_cloud_ = msg;
            });
        
        depth_ind_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            depth_pc_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                latest_depth_cloud_ = msg;
            });
        
        RCLCPP_INFO(this->get_logger(), "Using individual subscribers with timer-based fusion");
    }
    
    void synchronized_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& l2_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& mid70_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& depth_msg)
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Fused cloud
        PointCloudT::Ptr fused_cloud(new PointCloudT);
        
        // Transform and add L2 cloud
        if (l2_msg && l2_msg->width * l2_msg->height > 0) {
            auto transformed = transform_cloud(l2_msg, target_frame_);
            if (transformed) {
                *fused_cloud += *transformed;
            }
        }
        
        // Transform and add Mid-70 cloud
        if (mid70_msg && mid70_msg->width * mid70_msg->height > 0) {
            auto transformed = transform_cloud(mid70_msg, target_frame_);
            if (transformed) {
                *fused_cloud += *transformed;
            }
        }
        
        // Transform and add Depth cloud
        if (depth_msg && depth_msg->width * depth_msg->height > 0) {
            auto transformed = transform_cloud(depth_msg, target_frame_);
            if (transformed) {
                *fused_cloud += *transformed;
            }
        }
        
        if (fused_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Fused cloud is empty!");
            return;
        }
        
        // Apply filters
        fused_cloud = apply_filters(fused_cloud);
        
        // Publish
        publish_cloud(fused_cloud, l2_msg->header.stamp);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        RCLCPP_DEBUG(this->get_logger(), "Fusion took %ld ms, %zu points",
            duration.count(), fused_cloud->size());
    }
    
    void process_and_publish()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        
        PointCloudT::Ptr fused_cloud(new PointCloudT);
        rclcpp::Time stamp = this->now();
        
        // Transform and add available clouds
        if (latest_l2_cloud_) {
            auto transformed = transform_cloud(latest_l2_cloud_, target_frame_);
            if (transformed) {
                *fused_cloud += *transformed;
                stamp = latest_l2_cloud_->header.stamp;
            }
        }
        
        if (latest_mid70_cloud_) {
            auto transformed = transform_cloud(latest_mid70_cloud_, target_frame_);
            if (transformed) {
                *fused_cloud += *transformed;
            }
        }
        
        if (latest_depth_cloud_) {
            auto transformed = transform_cloud(latest_depth_cloud_, target_frame_);
            if (transformed) {
                *fused_cloud += *transformed;
            }
        }
        
        if (fused_cloud->empty()) {
            return;
        }
        
        // Apply filters
        fused_cloud = apply_filters(fused_cloud);
        
        // Publish
        publish_cloud(fused_cloud, stamp);
    }
    
    PointCloudT::Ptr transform_cloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        const std::string& target_frame)
    {
        PointCloudT::Ptr cloud(new PointCloudT);
        
        if (cloud_msg->header.frame_id == target_frame) {
            // No transform needed
            pcl::fromROSMsg(*cloud_msg, *cloud);
            return cloud;
        }
        
        try {
            // Look up transform
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame, cloud_msg->header.frame_id,
                cloud_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            
            // Transform the cloud
            sensor_msgs::msg::PointCloud2 transformed_msg;
            tf2::doTransform(*cloud_msg, transformed_msg, transform);
            
            pcl::fromROSMsg(transformed_msg, *cloud);
            return cloud;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Could not transform from %s to %s: %s",
                cloud_msg->header.frame_id.c_str(), target_frame.c_str(), ex.what());
            return nullptr;
        }
    }
    
    PointCloudT::Ptr apply_filters(PointCloudT::Ptr cloud)
    {
        if (cloud->empty()) return cloud;

        // Self-filter: remove points within robot footprint
        if (enable_self_filter_) {
            pcl::CropBox<PointT> crop;
            crop.setInputCloud(cloud);
            crop.setMin(Eigen::Vector4f(
                self_filter_min_x_, self_filter_min_y_,
                self_filter_min_z_, 1.0));
            crop.setMax(Eigen::Vector4f(
                self_filter_max_x_, self_filter_max_y_,
                self_filter_max_z_, 1.0));
            crop.setNegative(true);
            crop.setKeepOrganized(false);
            PointCloudT::Ptr filtered(new PointCloudT);
            crop.filter(*filtered);
            cloud = filtered;
        }

        if (cloud->empty()) return cloud;

        // Range filter (distance from origin)
        if (enable_range_filter_) {
            PointCloudT::Ptr filtered(new PointCloudT);
            for (const auto& pt : cloud->points) {
                float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                if (dist >= min_range_ && dist <= max_range_) {
                    filtered->points.push_back(pt);
                }
            }
            filtered->width = filtered->points.size();
            filtered->height = 1;
            filtered->is_dense = true;
            cloud = filtered;
        }
        
        if (cloud->empty()) return cloud;
        
        // Height filter
        if (enable_height_filter_) {
            pcl::PassThrough<PointT> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_height_, max_height_);
            PointCloudT::Ptr filtered(new PointCloudT);
            pass.filter(*filtered);
            cloud = filtered;
        }
        
        if (cloud->empty()) return cloud;
        
        // Ground removal using RANSAC
        if (enable_ground_removal_) {
            cloud = remove_ground(cloud);
        }
        
        if (cloud->empty()) return cloud;
        
        // Voxel grid downsampling
        if (enable_voxel_filter_ && cloud->size() > 100) {
            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            PointCloudT::Ptr filtered(new PointCloudT);
            voxel.filter(*filtered);
            cloud = filtered;
        }
        
        if (cloud->empty()) return cloud;
        
        // Statistical outlier removal
        if (enable_outlier_removal_ && cloud->size() > outlier_mean_k_) {
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setMeanK(outlier_mean_k_);
            sor.setStddevMulThresh(outlier_stddev_);
            PointCloudT::Ptr filtered(new PointCloudT);
            sor.filter(*filtered);
            cloud = filtered;
        }
        
        return cloud;
    }
    
    PointCloudT::Ptr remove_ground(PointCloudT::Ptr cloud)
    {
        // RANSAC plane segmentation for ground removal
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(ground_threshold_);
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));  // Z-axis (up)
        seg.setEpsAngle(0.15);  // ~8.5 degrees tolerance
        
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.empty()) {
            return cloud;  // No ground found
        }
        
        // Extract non-ground points
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);  // Keep non-ground points
        PointCloudT::Ptr filtered(new PointCloudT);
        extract.filter(*filtered);
        
        return filtered;
    }
    
    void publish_cloud(PointCloudT::Ptr cloud, const rclcpp::Time& stamp)
    {
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header.stamp = stamp;
        output_msg.header.frame_id = target_frame_;
        
        fused_cloud_pub_->publish(output_msg);
    }

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Synchronized subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> l2_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> mid70_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Individual subscribers (for async mode)
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr l2_ind_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mid70_ind_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_ind_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_l2_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_mid70_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_depth_cloud_;
    std::mutex cloud_mutex_;
    
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr process_timer_;
    
    // Parameters
    std::string l2_topic_;
    std::string mid70_topic_;
    std::string depth_pc_topic_;
    std::string fused_topic_;
    std::string target_frame_;
    
    bool use_sync_;
    int sync_queue_size_;
    double sync_slop_;
    double publish_rate_;
    
    double voxel_size_;
    bool enable_voxel_filter_;
    
    bool enable_outlier_removal_;
    int outlier_mean_k_;
    double outlier_stddev_;
    
    bool enable_range_filter_;
    double min_range_;
    double max_range_;
    
    bool enable_ground_removal_;
    double ground_threshold_;
    
    bool enable_height_filter_;
    double min_height_;
    double max_height_;

    bool enable_self_filter_;
    double self_filter_min_x_, self_filter_max_x_;
    double self_filter_min_y_, self_filter_max_y_;
    double self_filter_min_z_, self_filter_max_z_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#ifndef IAR_AMCL__AMCL_NODE_HPP_
#define IAR_AMCL__AMCL_NODE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav_msgs/srv/set_map.hpp"

#include "nav2_amcl/sensors/laser/laser.hpp"
#include "nav2_amcl/motion_model/motion_model.hpp"

#include "std_srvs/srv/empty.hpp"

#include "message_filters/subscriber.h"


namespace iar_amcl
{
    class AmclNode : public nav2_util::LifecycleNode
    {
    public:
        AmclNode();
        ~AmclNode();
    protected:
        // Lifecycle configure
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        // Lifecycle activate
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        // Lifecycle deactivate
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        // Lifecycle cleanup
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        // Lifecycle shutdown
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

          // Pose hypothesis
        typedef struct
        {
            double weight;             // Total weight (weights sum to 1)
            pf_vector_t pf_pose_mean;  // Mean of pose esimate
            pf_matrix_t pf_pose_cov;   // Covariance of pose estimate
        } amcl_hyp_t;

        //ROS2 TF2, reference https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Tf2-Main.html
        void initTransforms();
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        bool latest_tf_valid_{false};
        bool sent_first_transform_{false};
        tf2::Transform latest_tf_;

        void initParameters();
        // parameters for robot motion models
        double alpha1_;
        double alpha2_;
        double alpha3_;
        double alpha4_;
        // likelihood field sensor model parameters
        double laser_likelihood_max_dist_;
        double z_hit_;
        double z_max_;
        double z_rand_;
        double sigma_hit_;
        int max_beams_;
        // frame parameters
        std::string base_frame_id_;
        std::string global_frame_id_;
        std::string odom_frame_id_;
        // particle filter parameters

        // subscribed topic parameters
        std::string scan_topic_{"scan"};
        std::string map_topic_{"map"};

        bool tf_broadcast_;
        void sendMapToOdomTransform(const tf2::TimePoint & transform_expiration);


        void initPubSub();
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr initial_pose_sub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
        void initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
        geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose_;
        bool initial_pose_is_known_;
        bool init_pose_received_on_inactive{false};
        void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        std::atomic<bool> first_map_received_{false};
        bool first_map_only_{true};
        void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
        map_t * map_{nullptr};
        map_t * convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);
        void freeMapDependentMemory();
        void createFreeSpaceVector();
        static std::vector<std::pair<int, int>> free_space_indices;

        void initOdometry();
        double init_pose_[3];
        double init_cov_[3];
        std::unique_ptr<nav2_amcl::DifferentialMotionModel> motion_model_;
        geometry_msgs::msg::PoseStamped latest_odom_pose_;

        std::vector<nav2_amcl::Laser *> lasers_;
        std::vector<bool> lasers_update_;
        std::map<std::string, int> frame_to_laser_;
        rclcpp::Time last_laser_received_ts_;
        

        static pf_vector_t uniformPoseGenerator(void * arg);

        bool pf_init_;
        pf_vector_t pf_odom_pose_;
        int resample_count_{0};
        pf_t * pf_{nullptr};
        double d_thresh_;
        double a_thresh_;
        int resample_interval_;
        double pf_err_;
        double pf_z_;
        int max_particles_;
        int min_particles_;
        double alpha_fast_;
        double alpha_slow_;

        
        void initServices();
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;
        void globalLocalizationCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response);

        void nomotionUpdateCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response);
        // Nomotion update control. Used to temporarily let amcl update samples even when no motion occurs
        std::atomic<bool> force_update_{false};

        std::recursive_mutex mutex_;
        // Since the sensor data from gazebo or the robot is not lifecycle enabled, we won't
        // respond until we're in the active state
        std::atomic<bool> active_{false};

        void initMessageFilters();
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
        std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
        tf2::Duration transform_tolerance_;
        message_filters::Connection laser_scan_connection_;
        void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);
        rclcpp::Time last_time_printed_msg_;
        bool shouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta);
        bool updateFilter(
            const int & laser_index,
            const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
            const pf_vector_t & pose);
        void pf_resample(pf_t * pf);
        void publishParticleCloud(const pf_sample_set_t * set);
        bool getMaxWeightHyp(
            std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
            int & max_weight_hyp);
        void publishAmclPose(
            const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
            const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp);
        void calculateMaptoOdomTransform(
            const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
            const std::vector<amcl_hyp_t> & hyps,
            const int & max_weight_hyp);
        bool checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time);
        /*
        * @brief Add a new laser scanner if a new one is received in the laser scallbacks
        */
        bool addNewScanner(
            int & laser_index,
            const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
            const std::string & laser_scan_frame_id,
            geometry_msgs::msg::PoseStamped & laser_pose);
        nav2_amcl::Laser * createLaserObject();
        int scan_error_count_{0};


        /*
        * @brief Get robot pose in odom frame using TF
        */
        bool getOdomPose(
            // Helper to get odometric pose from transform system
            geometry_msgs::msg::PoseStamped & pose,
            double & x, double & y, double & yaw,
            const rclcpp::Time & sensor_timestamp, const std::string & frame_id);
        std::atomic<bool> first_pose_sent_;

        void initLaserScan();
        void initParticleFilter();
    };
}// namespace iar_amcl

#endif  // IAR_AMCL__AMCL_NODE_HPP_
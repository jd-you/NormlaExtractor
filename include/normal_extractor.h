#ifndef __NORMAL_EXTRACTOR_H__
#define __NORMAL_EXTRACTOR_H__

#include "estimator.h"
#include "filter.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <normal_extractor/NEXTServerConfig.h>

namespace Planning3D {

class NormalExtractor {
  public:
    NormalExtractor(std::string ns = "~");

    ~NormalExtractor() = default;

    // dyn reconfig callback
    void dynCallback(normal_extractor::NEXTServerConfig &config,
                     uint32_t level);

  private:
    void callback(sensor_msgs::PointCloud2ConstPtr ptr);

    void
    publishPoseArray(const std::vector<std::pair<Vec3f, Quaternion>> &normals);

    void publishMarkerArray(
        const std::vector<std::pair<Vec3f, Quaternion>> &normals);

    visualization_msgs::Marker
    createArrowMarker(const string &frame_id, const string &name_space,
                      double duration, const std_msgs::ColorRGBA &color,
                      const geometry_msgs::Vector3 &scale,
                      const geometry_msgs::Point &position,
                      const geometry_msgs::Quaternion &orientation);

    ros::NodeHandle nh_;

    ros::Subscriber sub_static_cloud_; // subscriber for static point cloud

    ros::Subscriber sub_dynamic_cloud_; // subscriber for real time sensor data

    ros::Publisher pub_;

    ros::Publisher pub_visualize_;

    std::shared_ptr<PCLEstimator> pclEstimator_;
    std::shared_ptr<OcTreeEstimator> treeEstimator_;

    std::shared_ptr<Filter> filter_;

    double resolution_pcl_;
    double resolution_tree_;
    int depth_;
    bool visualize_;

    double filter_z_;

    int mode_;

    int mod_;

    std::string frame_id_;
    double duration_;

    PointCloudXYZ::Ptr cloudPtr_;
};
} // namespace Planning3D

#endif // __NORMAL_EXTRACTOR_H__

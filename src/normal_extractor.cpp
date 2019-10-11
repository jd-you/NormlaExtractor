#include "normal_extractor.h"

namespace Planning3D {

NormalExtractor::NormalExtractor(std::string ns) : nh_(ns) {

    nh_.param("resolution_pcl", resolution_pcl_, 0.3);
    nh_.param("resolution_tree", resolution_tree_, 0.03);
    nh_.param("tree_depth", depth_, 15);
    nh_.param("visualize", visualize_, false);
    nh_.param("mode", mode_, 1);
    nh_.param("frame_id", frame_id_, std::string("map"));
    nh_.param("duration", duration_, 1000.);
    nh_.param("filter_z", filter_z_, 4.);
    nh_.param("mod", mod_, 200);

    sub_static_cloud_ =
        nh_.subscribe("cloud_in", 10, &NormalExtractor::callback, this);
    sub_dynamic_cloud_ =
        nh_.subscribe("sensor_cloud_in", 10, &NormalExtractor::callback, this);

    pub_ = nh_.advertise<geometry_msgs::PoseArray>("normals", 10);
    pub_visualize_ =
        nh_.advertise<visualization_msgs::MarkerArray>("normals_visualize", 10);

    pclEstimator_ =
        std::shared_ptr<PCLEstimator>(new PCLEstimator(resolution_pcl_));

    treeEstimator_ = std::shared_ptr<OcTreeEstimator>(
        new OcTreeEstimator(resolution_tree_, depth_));

    filter_ = std::shared_ptr<Filter>(new Filter());

    cloudPtr_ = PointCloudXYZ::Ptr(new PointCloudXYZ());
}

void NormalExtractor::callback(sensor_msgs::PointCloud2ConstPtr ptr) {
    // ROS_INFO("in call back");
    // PointCloudXYZ::Ptr cloudPtr_(new PointCloudXYZ());
    pcl::fromROSMsg(*ptr, *cloudPtr_);

    filter_->setPass("z", -10, filter_z_);
    filter_->filterPass(cloudPtr_);
    filter_->filterSor(cloudPtr_);

    std::vector<std::pair<Vec3f, Quaternion>> normals;

    switch (mode_) {
    case 0:
        normals = pclEstimator_->estimate(cloudPtr_);
        break;
    case 1:
        normals = treeEstimator_->estimate(cloudPtr_);
        break;
    default:
        ROS_INFO("the mode is not supported, please choose mode 0 or 2!");
    }

    publishPoseArray(normals);

    if (visualize_) {
        publishMarkerArray(normals);
    }
    // ROS_INFO("finish call back");
}

void NormalExtractor::publishPoseArray(
    const std::vector<std::pair<Vec3f, Quaternion>> &normals) {
    geometry_msgs::PoseArray pa;

    pa.header.frame_id = frame_id_;
    pa.header.stamp = ros::Time();

    for (auto n : normals) {
        geometry_msgs::Pose p;

        auto pos = n.first;
        auto ori = n.second;

        p.position.x = pos(0);
        p.position.y = pos(1);
        p.position.z = pos(2);

        p.orientation.w = ori.w();
        p.orientation.x = ori.x();
        p.orientation.y = ori.y();
        p.orientation.z = ori.z();

        pa.poses.push_back(p);
    }

    pub_.publish(pa);
    pa.poses.clear();
}

void NormalExtractor::publishMarkerArray(
    const std::vector<std::pair<Vec3f, Quaternion>> &normals) {

    visualization_msgs::MarkerArray ma;

    std_msgs::ColorRGBA color;
    color.r = 255.;
    color.g = 0.;
    color.b = 0.;
    color.a = 1.;

    geometry_msgs::Vector3 scale;
    scale.x = 0.5;
    scale.y = 0.05;
    scale.z = 0.05;

    unsigned long i = 0;

    int mod;

    for (auto n : normals) {

        if (i % mod_ == 0) {

            auto pos = n.first;
            auto ori = n.second;

            string name_space = "normal_" + std::to_string(i);

            geometry_msgs::Point position;
            position.x = pos(0);
            position.y = pos(1);
            position.z = pos(2);

            geometry_msgs::Quaternion orientation;
            orientation.w = ori.w();
            orientation.x = ori.x();
            orientation.y = ori.y();
            orientation.z = ori.z();

            auto m = createArrowMarker(frame_id_, name_space, duration_, color,
                                       scale, position, orientation);

            ma.markers.push_back(m);
        }

        i++;
    }

    pub_visualize_.publish(ma);
    ma.markers.clear();
}

visualization_msgs::Marker NormalExtractor::createArrowMarker(
    const string &frame_id, const string &name_space, double duration,
    const std_msgs::ColorRGBA &color, const geometry_msgs::Vector3 &scale,
    const geometry_msgs::Point &position,
    const geometry_msgs::Quaternion &orientation) {
    visualization_msgs::Marker arrow;
    // header
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = ros::Time();

    // type
    arrow.ns = name_space;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(duration);

    // geometry
    arrow.color = color;
    arrow.scale = scale;

    // pose
    arrow.pose.position = position;
    arrow.pose.orientation = orientation;

    return arrow;
}

void NormalExtractor::dynCallback(normal_extractor::NEXTServerConfig &config,
                                  uint32_t level) {
    auto dyn_frame_id = config.frame_id;
    auto dyn_filter_z = config.filter_z;
    auto dyn_mode = config.mode;

    auto dyn_resolution_pcl = config.resolution_pcl;
    auto dyn_resolution_tree = config.resolution_tree;
    auto dyn_tree_depth = config.tree_depth;

    auto dyn_visualize = config.visualize;
    auto dyn_mod = config.mod;
    auto dyn_duration = config.duration;

    frame_id_ = dyn_frame_id;
    filter_z_ = dyn_filter_z;

    mode_ = dyn_mode;

    if (dyn_resolution_pcl != resolution_pcl_) {
        resolution_pcl_ = dyn_resolution_pcl;
        pclEstimator_->setResolution(resolution_pcl_);
    }

    if (dyn_resolution_tree != resolution_tree_) {
        resolution_tree_ = dyn_resolution_tree;
        treeEstimator_ = std::shared_ptr<OcTreeEstimator>(
            new OcTreeEstimator(resolution_tree_, depth_));
    }

    treeEstimator_->setDepth(dyn_tree_depth);

    visualize_ = dyn_visualize;
    mod_ = dyn_mod;
    duration_ = dyn_duration;

    //
    if (!cloudPtr_->empty()) {
        filter_->setPass("z", -10, filter_z_);
        filter_->filterPass(cloudPtr_);
        filter_->filterSor(cloudPtr_);

        std::vector<std::pair<Vec3f, Quaternion>> normals;

        switch (mode_) {
        case 0:
            normals = pclEstimator_->estimate(cloudPtr_);
            break;
        case 1:
            normals = treeEstimator_->estimate(cloudPtr_);
            break;
        default:
            ROS_INFO("the mode is not supported");
        }
        std::cout << normals.size() << std::endl;
        publishPoseArray(normals);

        if (visualize_) {
            publishMarkerArray(normals);
        }
    }
}
} // namespace Planning3D

#include "estimator.h"

namespace Planning3D {

OcTreeEstimator::OcTreeEstimator(double resolution, int depth)
    : tree_(resolution), depth_(depth) {}

std::vector<std::pair<Vec3f, Quaternion>>
OcTreeEstimator::estimate(PointCloudXYZ::Ptr src) {
    constructTree(src);

    return getNormal();
}

void OcTreeEstimator::constructTree(PointCloudXYZ::Ptr src) {
    for (auto p : src->points)
        tree_.updateNode(octomap::point3d(p.x, p.y, p.z), true);

    for (auto p : src->points)
        tree_.integrateNodeInformation(p.x, p.y, p.z);

    tree_.updateInnerOccupancy();
}

std::vector<std::pair<Vec3f, Quaternion>> OcTreeEstimator::getNormal() {
    std::vector<std::pair<Vec3f, Quaternion>> res;

    for (auto it = tree_.begin_tree(); it != tree_.end_tree(); ++it) {
        if (it.getDepth() == depth_) {
            auto info = it->getInformation();
            if (info.normal_ != Vec3f::Zero()) {
                auto p = info.accPoint_ / info.count_;
                auto normal = info.normal_;

                Vec3f xAxis = Vec3f(1.0, 0., 0.);
                Quaternion q = Quaternion::FromTwoVectors(xAxis, normal);

                res.push_back(std::make_pair(p, q));
            }
        }
    }

    return res;
}

std::vector<std::pair<Vec3f, Quaternion>>
PCLEstimator::estimate(PointCloudXYZ::Ptr src) {
    PointCloudPN::Ptr normalPtr(new PointCloudPN());

    pcl::NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud(src);

    pcl::search::KdTree<PointXYZ>::Ptr tree(
        new pcl::search::KdTree<PointXYZ>());
    ne.setSearchMethod(tree);

    PointCloudNormal::Ptr n(new PointCloudNormal());

    ne.setRadiusSearch(resolution_);
    ne.compute(*n);

    pcl::concatenateFields(*src, *n, *normalPtr);

    std::vector<std::pair<Vec3f, Quaternion>> res;

    for (auto pn : normalPtr->points) {
        Vec3f p(pn.x, pn.y, pn.z);

        Vec3f xAxis(1., 0., 0.);
        Quaternion q = Quaternion::FromTwoVectors(
            xAxis, Vec3f(pn.normal_x, pn.normal_y, pn.normal_z));

        res.push_back(std::make_pair(p, q));
    }

    return res;
}

} // namespace Planning3D

#include "filter.h"

namespace Planning3D {
Filter::Filter() {
    minCrop_ = Eigen::Vector4f(-1., -3., -5., 1.);
    maxCrop_ = Eigen::Vector4f(10., 3., 1., 1.);
    crop_.setMin(minCrop_);
    crop_.setMax(maxCrop_);

    direction_ = "z";
    minPass_ = -3.;
    maxPass_ = 1.;
    negative_ = false;
    pass_.setFilterFieldName(direction_);
    pass_.setFilterLimits(minPass_, maxPass_);
    pass_.setFilterLimitsNegative(negative_);

    k_ = 20;
    deviation_ = 1.;
    sor_.setMeanK(k_);
    sor_.setStddevMulThresh(deviation_);

    leafSize_ << 0.1, 0.1, 0.1;
    voxel_.setLeafSize(leafSize_(0), leafSize_(1), leafSize_(2));
}

Filter::~Filter() {}

void Filter::filterCrop(PointCloudXYZ::Ptr pCloud) {
    crop_.setInputCloud(pCloud);
    crop_.filter(*pCloud);
}

void Filter::filterPass(PointCloudXYZ::Ptr pCloud) {
    pass_.setInputCloud(pCloud);
    pass_.filter(*pCloud);
}

void Filter::filterSor(PointCloudXYZ::Ptr pCloud) {
    sor_.setInputCloud(pCloud);
    sor_.filter(*pCloud);
}

void Filter::filterCrop(PointCloudXYZ::Ptr pCloud, const Vec4f &min,
                        const Vec4f &max) {
    setCrop(min, max);
    filterCrop(pCloud);
}

void Filter::filterPass(PointCloudXYZ::Ptr pCloud, string direction, double min,
                        double max) {
    setPass(direction, min, max);
    filterPass(pCloud);
}

void Filter::filterSor(PointCloudXYZ::Ptr pCloud, int k, double deviation) {
    setSor(k, deviation);
    filterSor(pCloud);
}

void Filter::setCrop(const Vec4f &min, const Vec4f &max) {
    minCrop_ = min;
    maxCrop_ = max;
    crop_.setMin(min);
    crop_.setMax(max);
}

void Filter::setPass(string direction, double min, double max) {
    direction_ = direction;
    minPass_ = min;
    maxPass_ = max;
    pass_.setFilterFieldName(direction_);
    pass_.setFilterLimits(minPass_, maxPass_);
}

void Filter::setSor(int k, double deviation) {
    k_ = k;
    deviation_ = deviation;
    sor_.setMeanK(k_);
    sor_.setStddevMulThresh(deviation_);
}

void Filter::setVoxel(double x, double y, double z) {
    leafSize_ << x, y, z;
    voxel_.setLeafSize(leafSize_(0), leafSize_(1), leafSize_(2));
}

void Filter::filterVoxel(PointCloudXYZ::Ptr pCloud) {
    voxel_.setInputCloud(pCloud);
    voxel_.filter(*pCloud);
}

void Filter::filterVoxel(PointCloudXYZ::Ptr pCloud, double x, double y,
                         double z) {
    setVoxel(x, y, z);
    filterVoxel(pCloud);
}
void Filter::removeSupport(PointCloudXYZ::Ptr pCloud) {

    // remove support of sensor
    double dis = 0.55;
    pcl::ConditionOr<PointXYZ>::Ptr range_cond(
        new pcl::ConditionOr<PointXYZ>());

    range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(
        new pcl::FieldComparison<PointXYZ>("z", pcl::ComparisonOps::LT, -dis)));
    range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(
        new pcl::FieldComparison<PointXYZ>("z", pcl::ComparisonOps::GT, dis)));
    range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(
        new pcl::FieldComparison<PointXYZ>("y", pcl::ComparisonOps::LT, -dis)));
    range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(
        new pcl::FieldComparison<PointXYZ>("y", pcl::ComparisonOps::GT, dis)));
    range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(
        new pcl::FieldComparison<PointXYZ>("x", pcl::ComparisonOps::LT, -dis)));
    range_cond->addComparison(pcl::FieldComparison<PointXYZ>::ConstPtr(
        new pcl::FieldComparison<PointXYZ>("x", pcl::ComparisonOps::GT, dis)));

    removal_.setCondition(range_cond);

    removal_.setInputCloud(pCloud);
    removal_.filter(*pCloud);
}
} // namespace Planning3D
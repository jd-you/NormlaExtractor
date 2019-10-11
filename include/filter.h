#ifndef __FILTER_H__
#define __FILTER_H__

#include "types.h"

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace Planning3D {

/**
 * @brief class with different pcl standart filters
 *
 */
class Filter {
  public:
    Filter();
    ~Filter();

    void setCrop(const Vec4f &min, const Vec4f &max);
    void filterCrop(PointCloudXYZ::Ptr pCloud);
    void filterCrop(PointCloudXYZ::Ptr pCloud, const Vec4f &min,
                    const Vec4f &max);

    void setPass(string direction, double min, double max);
    void filterPass(PointCloudXYZ::Ptr pCloud);
    void filterPass(PointCloudXYZ::Ptr pCloud, string direction, double min,
                    double max);

    void setSor(int k, double deviation);
    void filterSor(PointCloudXYZ::Ptr pCloud);
    void filterSor(PointCloudXYZ::Ptr pCloud, int k, double deviation);

    void setVoxel(double x, double y, double z);
    void filterVoxel(PointCloudXYZ::Ptr pCloud);
    void filterVoxel(PointCloudXYZ::Ptr pCloud, double x, double y, double z);

    void removeSupport(PointCloudXYZ::Ptr pCloud);

  private:
    pcl::CropBox<PointXYZ> crop_;
    Vec4f minCrop_;
    Vec4f maxCrop_;

    pcl::PassThrough<PointXYZ> pass_;
    bool negative_;
    string direction_;
    double minPass_;
    double maxPass_;

    pcl::StatisticalOutlierRemoval<PointXYZ> sor_;
    int k_;
    double deviation_;

    pcl::VoxelGrid<PointXYZ> voxel_;
    Vec3f leafSize_;

    pcl::ConditionalRemoval<PointXYZ> removal_;
};
} // namespace Planning3D

#endif // __FILTER_H__
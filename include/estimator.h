#ifndef __ESTIMATOR_H__
#define __ESTIMATOR_H__
#include <vector>

#include "types.h"

#include "normal_octree/normal_octree.h"
#include "pcl/features/normal_3d.h"

namespace Planning3D {
class Estimator {
  public:
    Estimator() {}

    ~Estimator() = default;

    virtual std::vector<std::pair<Vec3f, Quaternion>>
    estimate(PointCloudXYZ::Ptr src) = 0;
};

class PCLEstimator : public Estimator {
  public:
    PCLEstimator(double resolution) : resolution_(resolution) {}

    ~PCLEstimator() = default;

    virtual std::vector<std::pair<Vec3f, Quaternion>>
    estimate(PointCloudXYZ::Ptr src);

    inline void setResolution(double resolution) { resolution_ = resolution; }

  private:
    double resolution_;
};

class OcTreeEstimator : public Estimator {
  public:
    OcTreeEstimator(double resolution, int depth);

    ~OcTreeEstimator() = default;

    virtual std::vector<std::pair<Vec3f, Quaternion>>
    estimate(PointCloudXYZ::Ptr src);

    inline void setDepth(int depth) { depth_ = depth; }

  private:
    int depth_; // normal estimation in which depth

    std::vector<std::pair<Vec3f, Quaternion>> getNormal();
    void constructTree(PointCloudXYZ::Ptr src);

    NormalOcTree tree_;
};
} // namespace Planning3D
#endif // __ESTIMATOR_H__
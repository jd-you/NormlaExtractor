#ifndef __NORMAL_OCTREE_NODE_H__
#define __NORMAL_OCTREE_NODE_H__
#include <Eigen/Eigenvalues>
#include <octomap/ColorOcTree.h>

#include "types.h"

namespace Planning3D {
class NormalOcTree;

class NormalPCA {
  public:
    NormalPCA() {}
    Vec3f estimate(const Mat3f &covariance);
};

class NormalOcTreeNode : public octomap::ColorOcTreeNode {
  public:
    friend class NormalOcTree;

    /**
     * @brief extended information stored in the octree node
     *
     */
    struct Information {

        Information()
            : accPoint_(Vec3f::Zero()), accPointSquare_(Mat3f::Zero()),
              count_(0), normal_(Vec3f::Zero()) {}

        Information(const Vec3f &accPoint, const Mat3f &accPointSquare,
                    unsigned int count, const Vec3f &normal)
            : accPoint_(accPoint), accPointSquare_(accPointSquare),
              count_(count), normal_(normal) {}

        Information(const Information &other)
            : accPoint_(other.accPoint_),
              accPointSquare_(other.accPointSquare_), count_(other.count_),
              normal_(other.normal_) {}

        inline bool operator==(const Information &other) const {
            return normal_.transpose() * other.normal_ > 0.992;
        }

        inline bool operator!=(const Information &other) const {
            return normal_.transpose() * other.normal_ > 0.992;
        }

        Vec3f accPoint_;
        Mat3f accPointSquare_;
        unsigned int count_;

        Vec3f normal_;
    };

    NormalOcTreeNode();
    NormalOcTreeNode(const NormalOcTreeNode &other);

    bool operator==(const NormalOcTreeNode &other) const;

    void copyData(const NormalOcTreeNode &from);

    // interface
    inline Information getInformation() const { return info_; }
    inline Information &getInformation() { return info_; }

    inline void setInformation(const Information &info) { this->info_ = info; }

    inline void setInformation(const Vec3f &accPoint,
                               const Mat3f &accPointSquare, unsigned int count,
                               const Vec3f &normal = Vec3f::Zero()) {
        info_ = Information(accPoint, accPointSquare, count, normal);
    }
    inline void setInformationAccPoint(const Vec3f &accPoint) {
        info_.accPoint_ = accPoint;
    }
    inline void setInformationAccPointSquare(const Mat3f &accPointSquare) {
        info_.accPointSquare_ = accPointSquare;
    }
    inline void setInformationCount(unsigned int count) {
        info_.count_ = count;
    }
    inline void setInformationNormal(const Vec3f &normal) {
        info_.normal_ = normal;
    }
    inline bool isInformationSet() const { return (info_.count_ != 0); }

    //
    void estimate();
    void updateInformationChildren();
    Information getChildInformation() const;

  private:
    static NormalPCA pca_;
    Information info_;
};
} // namespace Planning3D
#endif // __NORMAL_OCTREE_NODE_H__
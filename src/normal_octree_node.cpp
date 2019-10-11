#include "normal_octree/normal_octree_node.h"

namespace Planning3D {
NormalPCA NormalOcTreeNode::pca_ = NormalPCA();

Vec3f NormalPCA::estimate(const Mat3f &covariance) {
    Eigen::EigenSolver<Mat3f> eigen_solver(covariance);

    Mat3f eigenVectors = eigen_solver.pseudoEigenvectors();
    Mat3f eigenValues = eigen_solver.pseudoEigenvalueMatrix();

    int minCol = 0;
    minCol = eigenValues(minCol, minCol) < eigenValues(1, 1) ? minCol : 1;
    minCol = eigenValues(minCol, minCol) < eigenValues(2, 2) ? minCol : 2;
    // TODO check另外两个特征值很近
    Vec3f normal = eigenVectors.col(minCol);

    return normal;
}

NormalOcTreeNode::NormalOcTreeNode()
    : octomap::ColorOcTreeNode(), info_(Information()) {}

NormalOcTreeNode::NormalOcTreeNode(const NormalOcTreeNode &other)
    : octomap::ColorOcTreeNode(other), info_(other.info_) {}

bool NormalOcTreeNode::operator==(const NormalOcTreeNode &other) const {
    return (value == other.value && info_ == other.info_);
}

void NormalOcTreeNode::copyData(const NormalOcTreeNode &from) {
    ColorOcTreeNode::copyData(from);
    this->info_ = from.info_;
}

void NormalOcTreeNode::estimate() {
    Mat3f covariance;
    covariance = info_.accPointSquare_ / info_.count_ -
                 info_.accPoint_ * info_.accPoint_.transpose() /
                     (info_.count_ * info_.count_);
    info_.normal_ = pca_.estimate(covariance);
}

void NormalOcTreeNode::updateInformationChildren() {
    info_ = getChildInformation();
}

NormalOcTreeNode::Information NormalOcTreeNode::getChildInformation() const {
    // pca_.reset();

    Information info;
    if (children != NULL) {
        for (int i = 0; i < 8; ++i) {
            NormalOcTreeNode *child =
                static_cast<NormalOcTreeNode *>(children[i]);

            if (child != NULL && child->isInformationSet()) {
                auto infoChild = child->getInformation();

                info.accPoint_ += infoChild.accPoint_;
                info.accPointSquare_ += infoChild.accPointSquare_;
                info.count_ += infoChild.count_;
            }
        }

        if (info.count_ < 3)
            return info;

        Mat3f covariance;
        covariance = info.accPointSquare_ / info.count_ -
                     info.accPoint_ * info.accPoint_.transpose() /
                         (info.count_ * info.count_);
        info.normal_ = pca_.estimate(covariance);
        return info;
    }
    return info;
}
} // namespace Planning3D

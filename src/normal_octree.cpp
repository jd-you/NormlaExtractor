#include "normal_octree/normal_octree.h"

namespace Planning3D {

NormalOcTree::StaticMemberInitializer NormalOcTree::NormalOcTreeMemberInit =
    NormalOcTree::StaticMemberInitializer();

NormalOcTree::NormalOcTree(double resolution)
    : octomap::OccupancyOcTreeBase<NormalOcTreeNode>(resolution) {
    NormalOcTreeMemberInit.ensureLinking();
}

NormalOcTree::~NormalOcTree() {}

bool NormalOcTree::pruneNode(NormalOcTreeNode *node) { return false; }

bool NormalOcTree::isNodeCollapsible(const NormalOcTreeNode *node) const {
    return false;
}

NormalOcTreeNode *
NormalOcTree::integrateNodeInformation(const octomap::OcTreeKey &key,
                                       const Vec3f &point) {
    NormalOcTreeNode *n = search(key);

    if (n) {

        auto &info = n->getInformation();

        info.accPoint_ += point;
        info.accPointSquare_ += point * point.transpose();
        info.count_++;

        if (info.count_ > 2)
            n->estimate();
        else
            info.normal_ = Vec3f::Zero();
    }

    return n;
}

void NormalOcTree::updateInnerOccupancyRecurs(NormalOcTreeNode *node,
                                              unsigned int depth) {
    if (nodeHasChildren(node)) {
        if (depth < this->tree_depth) {
            for (unsigned int i = 0; i < 8; ++i) {
                if (nodeChildExists(node, i)) {
                    updateInnerOccupancyRecurs(getNodeChild(node, i),
                                               depth + 1);
                }
            }
        }
        node->updateOccupancyChildren();
        node->updateInformationChildren();
    }
}

void NormalOcTree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
}

NormalOcTreeNode *NormalOcTree::integrateNodeInformation(float x, float y,
                                                         float z) {
    octomap::OcTreeKey key;
    if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key))
        return NULL;
    Vec3f center(x, y, z);
    return integrateNodeInformation(key, center);
}

} // namespace Planning3D
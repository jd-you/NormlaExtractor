#ifndef __NORMAL_OCTREE_H__
#define __NORMAL_OCTREE_H__

#include <iostream>
#include <octomap/OccupancyOcTreeBase.h>

#include "normal_octree_node.h"
#include "types.h"

namespace Planning3D {
class NormalOcTree : public octomap::OccupancyOcTreeBase<NormalOcTreeNode> {
  public:
    NormalOcTree(double resolution);
    ~NormalOcTree();

    NormalOcTree *create() const { return new NormalOcTree(resolution); }

    std::string getTreeType() const { return "ColorOcTree"; }

    virtual bool pruneNode(NormalOcTreeNode *node);
    virtual bool isNodeCollapsible(const NormalOcTreeNode *node) const;

    NormalOcTreeNode *integrateNodeInformation(const octomap::OcTreeKey &key,
                                               const Vec3f &center);

    NormalOcTreeNode *integrateNodeInformation(float x, float y, float z);

    void updateInnerOccupancy();

  private:
    // depth是开始的层数
    void updateInnerOccupancyRecurs(NormalOcTreeNode *node, unsigned int depth);

    class StaticMemberInitializer {
      public:
        StaticMemberInitializer() {
            NormalOcTree *tree = new NormalOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

        void ensureLinking(){};
    };

    static StaticMemberInitializer NormalOcTreeMemberInit;
};
} // namespace Planning3D

#endif // __NORMAL_OCTREE_H__
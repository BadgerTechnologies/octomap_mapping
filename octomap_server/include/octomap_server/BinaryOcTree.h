#ifndef OCTOMAP_SERVER_BINARY_OCTREE_HH
#define OCTOMAP_SERVER_BINARY_OCTREE_HH

#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeDataNode.h>

namespace octomap_server {

/**
 * An Octree-node which stores a binary value per node.
 * This value is used to represent occupancy for a sensor update cycle.
 */
class BinaryOcTreeNode : public octomap::OcTreeDataNode<bool> {

public:

  // Binary nodes default to false on construction
  BinaryOcTreeNode() : octomap::OcTreeDataNode<bool>(false) {}
  ~BinaryOcTreeNode() {}
};

/**
 * An AbstractOcTree which stores a binary value per node.
 */
class BinaryOcTree : public octomap::OcTreeBase <BinaryOcTreeNode> {

public:
  /// Default constructor, sets resolution of leafs
  BinaryOcTree(double resolution) : octomap::OcTreeBase<BinaryOcTreeNode>(resolution) {}

  // Returns true if a node was inserted, false if the node already existed
  bool insert(const octomap::OcTreeKey& k, bool value = false) {
    bool inserted = false;

    if (root == NULL) {
      inserted = true;
      root = new BinaryOcTreeNode();
    }
    BinaryOcTreeNode* curNode (root);

    // follow or construct nodes down to last level...
    for (int i=(tree_depth-1); i>=0; i--) {

      unsigned int pos = computeChildIdx(k, i);

      // requested node does not exist
      if (!nodeChildExists(curNode, pos)) {
        inserted = true;
        createNodeChild(curNode, pos);
      }
      // descend down one level in the tree
      curNode = getNodeChild(curNode, pos);
    }

    // only set to true if it was false.
    // do not allow setting false if true has already been set.
    if (!curNode->getValue() && value)
    {
      curNode->setValue(true);
    }
    return inserted;
  }
};

} // end namespace octomap_server

#endif

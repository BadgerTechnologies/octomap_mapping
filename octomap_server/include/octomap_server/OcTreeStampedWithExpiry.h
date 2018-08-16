#ifndef OCTOMAP_OCTREE_STAMPED_NONLINEAR_DECAY_H
#define OCTOMAP_OCTREE_STAMPED_NONLINEAR_DECAY_H

#include <ctime>
#include <algorithm>
#include <stdlib.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap_server {

// node definition
class OcTreeNodeStampedWithExpiry : public octomap::OcTreeNode {

  public:
    // Class-wide Parameters.

    OcTreeNodeStampedWithExpiry() : octomap::OcTreeNode(), stamp(0), expiry(0) {}

    OcTreeNodeStampedWithExpiry(const OcTreeNodeROSStamped& rhs) :
        octomap::OcTreeNode(rhs),
        stamp(rhs.stamp),
        expiry(rhs.expiry),
    {
    }

    bool operator==(const OcTreeNodeStampedWithExpiry& rhs) const
    {
      // No need to compare expiry, as it is a function of stamp and value
      return (rhs.value == value && rhs.stamp == stamp);
    }

    void copyData(const OcTreeNodeStampedWithExpiry& from)
    {
      octomap::OcTreeNode::copyData(from);
      stamp = from.stamp;
      expiry = from.expiry;
    }

    // timestamp
    inline time_t getTimestamp() const { return stamp; }
    inline void setTimestamp(time_t new_stamp) { stamp = new_stamp; }

    // expiry
    inline time_t getExpiry() const { return expiry; }
    inline time_t setExpiry(time_t new_expiry) const { expiry = new_expiry; }

    // update occupancy and timesteps of inner nodes
    inline void updateOccupancyChildren()
    {
      octomap::OcTreeNode::updateOccupancyChildren();

      time_t min_stamp = std::numeric_limits<time_t>::max();
      time_t min_expiry = std::numeric_limits<time_t>::max();
      bool have_children = false;

      if (children != NULL){
        for (unsigned int i=0; i<8; i++) {
          if (children[i] != NULL) {
            OcTreeNodeStampedWithExpiry node * = static_cast<OcTreeNodeStampedWithExpiry*>(children[i]);
            min_stamp = std::min(min_stamp, node->stamp);
            min_expiry = std::min(min_expiry, node->expiry);
            have_children = true;
          }
        }
      }

      // only update if we have children
      if (have_children)
      {
        stamp = min_stamp;
        expiry = min_expiry;
      }
    }

  protected:
    // There is no need for more than second accuracy, so only store the
    // seconds component of the timestamp.
    time_t stamp, expiry;
};


// tree definition
class OcTreeStampedWithExpiry : public OccupancyOcTreeBase <OcTreeNodeStampedWithExpiry>
{

  public:
    /// Default constructor, sets resolution of leafs
    OcTreeStampedWithExpiry(double resolution);

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTreeStampedWithExpiry* create() const {return new OcTreeStamped(resolution); }

    std::string getTreeType() const {return "OcTreeStampedWithExpiry";}

    //! \return Time of last update
    time_t getLastUpdateTime();

    // Remove all expired nodes.
    // This also calculates and stores any missing expiration times in the tree.
    // This function should be called periodically.
    void expireNodes();

    virtual void updateNodeLogOdds(OcTreeNodeStampedWithExpiry* node, const float& update) const;
    void integrateMissNoTime(OcTreeNodeStampedWithExpiry* node) const;

  protected:
    // Quadratic delta-t expiration coefficients. The input is the number of
    // times a particular mode was marked from the default value (which would
    // be the curreng logodds divided prob_hit_log).
    float a_coeff = 1.0 / 50.0;
    // Assume b_coeff is always zero
    float c_coeff = 15.0;
    // Used as the new value for updated nodes.
    // Only updated when calling expireNodes. This keeps our idea of time at
    // the resolution of our expiration rate check, which allows us to easily
    // prune nodes as their timestamps will only change when this time is
    // updated.
    time_t last_expire_time;

    // Return true if this node has expired.
    // Assume node is valid.
    bool expireNodeRecurs(OcTreeNodeStampedWithExpiry* node);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        OcTreeStampedWithExpiry* tree = new OcTreeStampedWithExpiry(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {};
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeStampedWithExpiryMemberInit;
};

} // end namespace

#endif

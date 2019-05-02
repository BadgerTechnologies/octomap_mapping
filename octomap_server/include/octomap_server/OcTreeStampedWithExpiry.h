#ifndef OCTOMAP_OCTREE_STAMPED_NONLINEAR_DECAY_H
#define OCTOMAP_OCTREE_STAMPED_NONLINEAR_DECAY_H

#include <ctime>
#include <algorithm>
#include <stdlib.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

namespace octomap_server {

// node definition
class OcTreeNodeStampedWithExpiry : public octomap::OcTreeNode
{

  public:
    // Class-wide Parameters.

    OcTreeNodeStampedWithExpiry() : octomap::OcTreeNode(), stamp(0), expiry(0), ema(-1.0) {}

    OcTreeNodeStampedWithExpiry(const OcTreeNodeStampedWithExpiry& rhs) :
        octomap::OcTreeNode(rhs),
        stamp(rhs.stamp),
        expiry(rhs.expiry),
        ema(rhs.ema),
        emvar(rhs.emvar)
    {
    }

    bool operator==(const OcTreeNodeStampedWithExpiry& rhs) const
    {
      // No need to compare expiry, as it is a function of stamp and value
      // XXX fix this to not have magic numbers
      return (rhs.value == value && rhs.stamp == stamp && fabs(rhs.ema - ema)<1e-6 && fabs(rhs.emvar - emvar)<1e-6 );
    }

    void copyData(const OcTreeNodeStampedWithExpiry& from)
    {
      octomap::OcTreeNode::copyData(from);
      stamp = from.stamp;
      expiry = from.expiry;
      ema = from.ema;
      emvar = from.emvar;
    }

    // Caller must first have checked that all children are individually
    // deleted if necessary!
    void deleteNodeChildren()
    {
       delete[] children;
       children = NULL;
    }

    // timestamp
    inline time_t getTimestamp() const { return stamp; }
    inline void setTimestamp(time_t new_stamp) { stamp = new_stamp; }

    // expiry
    inline time_t getExpiry() const { return expiry; }
    inline time_t setExpiry(time_t new_expiry) { expiry = new_expiry; }

    // update occupancy and timesteps of inner nodes
    inline void updateOccupancyChildren()
    {
      octomap::OcTreeNode::updateOccupancyChildren();

      time_t min_stamp = std::numeric_limits<time_t>::max();
      time_t min_expiry = std::numeric_limits<time_t>::max();
      int num_children = 0;
      float ema_sum = 0.0;
      float emvar_sum = 0.0;

      if (children != NULL) {
        for (unsigned int i=0; i<8; i++) {
          if (children[i] != NULL) {
            OcTreeNodeStampedWithExpiry *node = static_cast<OcTreeNodeStampedWithExpiry*>(children[i]);
            min_stamp = std::min(min_stamp, node->stamp);
            min_expiry = std::min(min_expiry, node->expiry);
            num_children++;
            ema_sum += node->ema;
            emvar_sum += node->emvar;
          }
        }
      }

      // only update if we have children
      if (num_children > 0) {
        stamp = min_stamp;
        expiry = min_expiry;
        // XXX determine if this is necessary
        // just set statistics to average of children
        ema = ema_sum / num_children;
        emvar = emvar_sum / num_children;
      }
    }

    // Get average occupancy. 1.0 is occupied, 0.0 is free.
    float getAverage() const {return ema;}
    void resetAverage() {ema = -1.0;}

    // Get occupancy variance.
    float getVariance() const {return emvar;}

    inline void updateEMA(float x_i, const float ema_alpha)
    {
      // initialize when ema is negative (sentinel)
      if (ema < 0.0)
      {
        ema = x_i;
        emvar = 0.0;
        ROS_INFO_THROTTLE(1.0, "updateEMA: sentinel");
      }
      else
      {
        const float delta = x_i - ema;
        ema += ema_alpha * delta;
#if 0
        emvar = (1.0 - ema_alpha) * emvar + ema_alpha * delta * delta;
        ROS_INFO_THROTTLE(1.0, "updateEMA: x_i: %f ema_alpha: %f delta: %f ema: %f, emvar: %f", x_i, ema_alpha, delta, ema, emvar);
        if (x_i > 0)
        {
          ema += ema_alpha;
          if (ema > 1.0) ema = 1.0;
        }
        else
        {
          ema -= ema_alpha;
          if (ema < 0.0) ema = 0.0;
        }
#endif
        // clamp ema so we can prune tree and also take the quick way in updateNode
        if (ema < .001)
        {
          ema = 0.0;
        }
        else if (ema > .999)
        {
          ema = 1.0;
        }
      }
    }

  protected:
    // There is no need for more than second accuracy, so only store the
    // seconds component of the timestamp.
    time_t stamp, expiry;

    // Occupancy EMA, EMVar tracking.
    // This is used by the motion tracker, for instance
    // XXX this should be a sub-class
    float ema;
    float emvar;

};


// Tree definition
class OcTreeStampedWithExpiry : public octomap::OccupancyOcTreeBase <OcTreeNodeStampedWithExpiry>
{

  public:
    // Default constructor, sets resolution.
    // Be sure to call expireNodes() after construction to initialize the
    // expiration time. This can not be done in the default constructor
    // because it is called before ros::Time::now() can be accessed.
    OcTreeStampedWithExpiry(double resolution);

    // virtual constructor: creates a new object of same type
    // (Covariant return type requires an up-to-date compiler)
    OcTreeStampedWithExpiry* create() const {return new OcTreeStampedWithExpiry(resolution); }

    std::string getTreeType() const {return "OcTreeStampedWithExpiry";}

    // Time of last update
    time_t getLastUpdateTime() const {return last_expire_time;}

    // Remove all expired nodes.
    // This also calculates and stores any missing expiration times in the tree.
    // This function should be called periodically.
    void expireNodes();

    virtual OcTreeNodeStampedWithExpiry* updateNode(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval = false);
    virtual OcTreeNodeStampedWithExpiry* updateNode(const octomap::OcTreeKey& key, bool occupied, bool lazy_eval = false) {
      return updateNode(key, occupied ? prob_hit_log : prob_miss_log, lazy_eval);
    }

    virtual void updateNodeLogOdds(OcTreeNodeStampedWithExpiry* node, const float& update) const;

    virtual void expandNode(NodeType* node);
    virtual bool pruneNode(NodeType* node);

    bool getSizeChanged() {return size_changed;}
    void setSizeChanged(bool new_value) {size_changed = new_value;}

    time_t getMaxExpiryDelta() const {
      return a_coeff_log_odds * clamping_thres_max * clamping_thres_max + c_coeff;
    }

    void setEMAAlpha(float alpha) {
      ema_alpha = alpha;
    }

  protected:
    // Quadratic delta-t expiration coefficients. The input is the number of
    // times a particular mode was marked from the default value (which would
    // be the curreng logodds divided prob_hit_log).
    double a_coeff, a_coeff_log_odds;
    // Assume b_coeff is always zero
    double c_coeff;
    double quadratic_start, quadratic_start_log_odds;
    // Assume free space we just use a flat timeout for
    double c_coeff_free;
    // Used as the new value for updated nodes.
    // Only updated when calling expireNodes. This keeps our idea of time at
    // the resolution of our expiration rate check, which allows us to easily
    // prune nodes as their timestamps will only change when this time is
    // updated.
    time_t last_expire_time;
    int expire_count;

    float ema_alpha;

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

#include <ros/time.h>
#include "octomap_server/OcTreeNodeStampedWithExpiry.h"

namespace octomap_server {

OcTreeStampedWithExpiry::OcTreeStampedWithExpiry(double resolution)
  : OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry>(resolution)
{
  last_expire_time = ros::Time::now.sec;
  ocTreeStampedWithExpiryMemberInit.ensureLinking();
}

void OcTreeStampedWithExpiry::expireNodes()
{
  last_expire_time = ros::Time::now().sec;

  // pre-compute a_coeff in terms of log-odds instead of number of observations
  a_coeff_log_odds = a_coeff * (1.0 / prob_hit_log) * (1.0 / prob_hit_log);

  if (tree->root != NULL)
  {
    expireNodeRecurs(tree->root);
  }
}


void OcTreeStampedWithExpiry::expireNodeRecurs(OcTreeNodeStampedWithExpiry* node)
{
  // We can prune our search tree using the stored expiry.
  // If we encounter an expiry of zero, that indicates a deferred calculation.
  // Calculate the expiration of such nodes as they are encountered, being
  // sure to update the inner nodes when necessary.

  // For now, only expire occupied nodes
  if (node->isNodeOccupied())
  {
    time_t expiry = node->getExpiry();
    // For inner nodes, expiry is the minimum of all child nodes expiry's.
    // For nodes which have not yet had expiry calculated, expiry will be zero
    // If this node (or any child) has expired or this node (or any child) has
    // not had expiry set, decend in the search.
    if (expiry <= last_expire_time)
    {
      // This is an inner node
      if (nodeHasChildren(node))
      {
        // Update all children first
        for (unsigned int i=0; i<8; i++)
        {
          if (tree->nodeChildExists(node, i))
          {
            if (expireNodeRecurs(tree->getNodeChild(node, i)))
            {
              // Delete the child node
              deleteNodeChild(node, i);
            }
          }
        }
        // If we have no more children left, this inner node has expired too
        if (!nodeHasChildren(node))
        {
          // Curiously, deleteNodeChild does not clean up the dynamic array
          // for the children pointers when the child count drops to zero.
          // pruneNode does this, but is not what we want (we want to drop the
          // expired data, not merge it up the tree!). So prevent leaking
          // memory be ensuring this inner node deletes the children pointer
          // storage before the caller deletes us!
          delete[] node->children;
          node->children = NULL;
          return true;
        }
        // Update the inner node's expiry to track the min of all children
        node->updateOccupancyChildren();
      }
      else
      {
        // Leaf, update expiry if 0
        if (expiry == 0)
        {
          expiry = node->getTimestamp() + c_coeff;
          expiry += a_coeff_log_odds * value * value;
          node->setExpiry(expiry);
        }
        if (expiry <= last_expire_time)
        {
          // We have expired!
          return true;
        }
      }
    }
  }

  return false;
}

void OcTreeStampedWithExpiry::updateNodeLogOdds(OcTreeNodeStampedWithExpiry* node, const float& update) const
{
  // Update value based on expiry if present
  // This will rarely be present, only if we haven't seen this node recently
  time_t expiry = node->getExpiry();
  // For now, only decay occupied nodes
  if (expiry != 0 && node->isNodeOccupied())
  {
    time_t orig_delta_t = expiry - node->getTimestamp();
    time_t curr_delta_t = expiry - now;
    if (curr_delta_t <= 0)
    {
      // Its already expired, set it back to the background value prior to update
      node->setLogOdds(occ_prob_thres_log);
    }
    else
    {
      // Decay the value towards the background by an amount proportional to the remaining time
      double decay_factor = ((double)curr_delta_t)/((double)orig_delta_t);
      double logodds_delta = node->getLogOdds() - occ_prob_thres_log;
      node->setLogOdds(occ_prob_thres_log + logodds_delta * decay_factor);
    }
  }
  OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry>::updateNodeLogOdds(node, update);
  node->setTimestamp(last_expire_time);
  // Because we will very likely observe the same space multiple times, we do
  // not want to update expiry just to have to update it again on the next
  // sensor cycle. Instead, set it to zero and re-calculate it when it is
  // needed.
  node->setExpiry(0);
}

void OcTreeStampedWithExpiry::integrateMissNoTime(OcTreeNodeStampedWithExpiry* node) const{
OccupancyOcTreeBase<OcTreeNodeStampedWithExpiry>::updateNodeLogOdds(node, prob_miss_log);
}

OcTreeStampedWithExpiry::StaticMemberInitializer OcTreeStampedWithExpiry::ocTreeStampedWithExpiryMemberInit;

} // end namespace

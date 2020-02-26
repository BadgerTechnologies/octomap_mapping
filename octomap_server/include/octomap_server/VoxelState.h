#ifndef OCTOMAP_VOXEL_STATE_H
#define OCTOMAP_VOXEL_STATE_H

namespace octomap_server {

using VoxelState = uint8_t;

namespace voxel_state {

// Make the voxel states unique bits for speed of down sampling in the array
// implementation.
static constexpr VoxelState UNKNOWN = 0;
static constexpr VoxelState FREE = 1;
static constexpr VoxelState OCCUPIED = 2;
static constexpr VoxelState INNER = 4;
// MAX can be used to size a counter container indexed by VoxelState
static constexpr VoxelState MAX = 5;

}  // namespace VoxelState

}  // namespace octomap_server

#endif  // OCTOMAP_VOXEL_STATE_H

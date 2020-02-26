#include <octomap_server/SensorUpdateKeyMapArrayImpl.h>
#include <cstring>

namespace octomap_server {

SensorUpdateKeyMapArrayImpl::SensorUpdateKeyMapArrayImpl(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key)
{
  setBounds(min_key, max_key);
}

#if 0
void SensorUpdateKeyMapArrayImpl::apply(OcTreeT* tree) const
{
  octomap::OcTreeKey index;
  // put min and dims on stack so compiler may optimize w/ register
  const octomap::OcTreeKey min(min_key_);
  const octomap::OcTreeKey dims(dims_);
  const VoxelState* const grid(grid_.data());
  for (index.k[2]=0; index.k[2]<dims.k[2]; ++index.k[2])
  {
    const unsigned int z_skip = index.k[2] * skip_;
    for (index.k[1]=0; index.k[1]<dims.k[1]; ++index.k[1])
    {
      const unsigned int y_skip = z_skip + index.k[1] * dims[0];
      for (index.k[0]=0; index.k[0]<dims.k[0]; ++index.k[0])
      {
        const VoxelState voxel_state = grid[y_skip + index.k[0]];
        if (voxel_state != voxel_state::UNKNOWN)
        {
          octomap::OcTreeKey key;
          key.k[0] = min.k[0] + index.k[0];
          key.k[1] = min.k[1] + index.k[1];
          key.k[2] = min.k[2] + index.k[2];
          tree->updateNode(key, voxel_state == voxel_state::OCCUPIED);
        }
      }
    }
  }
}
#endif

void SensorUpdateKeyMapArrayImpl::clear()
{
  std::memset(grid_.data(), voxel_state::UNKNOWN, grid_.size() * sizeof(VoxelState));
}

void SensorUpdateKeyMapArrayImpl::setBounds(const octomap::OcTreeKey& min_key, const octomap::OcTreeKey& max_key)
{
  // Truncate minimum and maximum to level about our level to line our data up
  // with the level above.
  min_key_ = octomap::computeIndexKey(level_ + 1, min_key);
  max_key_ = octomap::computeIndexKey(level_ + 1, max_key);
  calculateDims();
}

void SensorUpdateKeyMapArrayImpl::calculateDims()
{
  if (level_ < depth_)
  {
    // Because max is inclusive, add one more index for the last element, as
    // it was truncated to the next-to-last index at the level.
    max_key_[0] += (1 << level_);
    max_key_[1] += (1 << level_);
    max_key_[2] += (1 << level_);
    dims_[0] = ((max_key_[0] - min_key_[0]) >> level_) + 1;
    dims_[1] = ((max_key_[1] - min_key_[1]) >> level_) + 1;
    dims_[2] = ((max_key_[2] - min_key_[2]) >> level_) + 1;
    // The last bit of dims_ should never be set when not at the root
    assert((dims_[0] & 0x01) == 0x01);
    assert((dims_[1] & 0x01) == 0x01);
    assert((dims_[2] & 0x01) == 0x01);
  }
  else
  {
    // Nothing to adjust here, all the bit shifts are invalid, so simply set
    // the dimensions to unity for the root node.
    dims_[0] = 1;
    dims_[1] = 1;
    dims_[2] = 1;
  }
  skip_ = dims_[0] * dims_[1];
  grid_.resize(dims_[0] * dims_[1] * dims_[2]);
}

bool SensorUpdateKeyMapArrayImpl::insertFree(const octomap::OcTreeKey& key)
{
  return insertImpl<voxel_state::FREE>(key);
}

bool SensorUpdateKeyMapArrayImpl::insertFreeCells(const octomap::OcTreeKey *free_cells, size_t free_cells_count)
{
  if (free_cells_count == 0)
    return false;
  VoxelState* grid = grid_.data();
  while (free_cells_count > 0)
  {
    VoxelState* grid_loc = grid + calculateIndex(*free_cells);
    if (*grid_loc == voxel_state::UNKNOWN)
    {
      *grid_loc = voxel_state::FREE;
    }
    ++free_cells;
    --free_cells_count;
  }
  return true;
}

bool SensorUpdateKeyMapArrayImpl::insertOccupied(const octomap::OcTreeKey& key)
{
  return insertImpl<voxel_state::OCCUPIED>(key);
}

void SensorUpdateKeyMapArrayImpl::insertInner(const octomap::OcTreeKey& key)
{
  insertImpl<voxel_state::INNER>(key);
}

VoxelState SensorUpdateKeyMapArrayImpl::find(const octomap::OcTreeKey& key) const
{
  return gridRef(key);
}

void SensorUpdateKeyMapArrayImpl::downSample(const octomap::OcTreeSpace& tree, SensorUpdateKeyMapImpl* output_map) const
{
  SensorUpdateKeyMapImpl::downSample(tree, output_map);
  SensorUpdateKeyMapArrayImpl* output_array = dynamic_cast<SensorUpdateKeyMapArrayImpl*>(output_map);

  // The output map is not an array. This should not happen, as the array
  // implementation will be choosen on each successive level.
  assert(output_array != nullptr);

  downSample(tree, output_array);
}

void SensorUpdateKeyMapArrayImpl::downSample(const octomap::OcTreeSpace& tree, SensorUpdateKeyMapArrayImpl* output_array) const
{
  // First, ensure target array bounds are set
  output_array->setBounds(min_key_, max_key_);
  unsigned int target_level = output_array->level_;
  octomap::OcTreeKey index;
  // put dims on stack so compiler may optimize w/ register
  const octomap::OcTreeKey dims(dims_);
  const VoxelState* const grid = grid_.data();
  VoxelState* out_grid = output_array->grid_.data();
  for (index.k[2]=0; index.k[2]<dims.k[2]; index.k[2] += 2)
  {
    const unsigned int z_skip = index.k[2] * skip_;
    for (index.k[1]=0; index.k[1]<dims.k[1]; index.k[1] += 2)
    {
      const unsigned int y_skip = index.k[1] * dims[0];
      const VoxelState* rows[4];
      rows[0] = &grid[z_skip + y_skip];
      rows[1] = &grid[z_skip + y_skip + dims[0]];
      rows[2] = &grid[z_skip + skip_ + y_skip];
      rows[3] = &grid[z_skip + skip_ + y_skip + dims[0]];
      octomap::OcTreeKey current_key;
      current_key[0] = min_key_[0] + index[0];
      current_key[1] = min_key_[1] + index[1];
      current_key[2] = min_key_[2] + index[2];
      // The current_key is the corner of the octant we are working on.
      // Since we make our array line up to the next level, the current_key is
      // also the target index key (key from the corner, NOT center!)
      VoxelState* out_row = &output_array->gridRef(current_key);
      for (index.k[0]=0; index.k[0]<dims.k[0]; index.k[0] += 2)
      {
        const VoxelState voxel_state_or
            = rows[0][index.k[0]]
            | rows[0][index.k[0]+1]
            | rows[1][index.k[0]]
            | rows[1][index.k[0]+1]
            | rows[2][index.k[0]]
            | rows[2][index.k[0]+1]
            | rows[3][index.k[0]]
            | rows[3][index.k[0]+1];
        const VoxelState voxel_state_and
            = rows[0][index.k[0]]
            & rows[0][index.k[0]+1]
            & rows[1][index.k[0]]
            & rows[1][index.k[0]+1]
            & rows[2][index.k[0]]
            & rows[2][index.k[0]+1]
            & rows[3][index.k[0]]
            & rows[3][index.k[0]+1];
        // Set the inner bit if any bits were present in the or.
        // Do this without a branch to speed up this inner loop.
        // This technique works because VoxelState's are each in different
        // bits. The FREE or OCCUPIED bits should be set past level 0
        // iff all 8 entries in the octant are all FREE or OCCUPIED. This is
        // covered by the logical and of each of the octant entries. The INNER
        // bit should get set if there are any children of any sort in the
        // octant. This gets handled by the or test and shift. Note the logic
        // works fine if all the octant entries were INNER, the inner bit is
        // set for both the and and the or.
        *out_row = voxel_state_and | ((voxel_state_or != 0) << 2);
        ++out_row;
        rows[0] += 2;
        rows[1] += 2;
        rows[2] += 2;
        rows[3] += 2;
      }
    }
  }
}

} // end namespace octomap_server

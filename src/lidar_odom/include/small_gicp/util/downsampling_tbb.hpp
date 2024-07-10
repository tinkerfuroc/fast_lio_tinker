// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <atomic>
#include <memory>
#include <iostream>

#include <tbb/tbb.h>
#include <small_gicp/points/traits.hpp>
#include <small_gicp/util/fast_floor.hpp>
#include <small_gicp/util/vector3i_hash.hpp>

namespace small_gicp {

/// @brief Voxel grid downsampling with TBB backend.
/// @note  Discretized voxel coords must be in 21bit range [-1048576, 1048575].
/// @param points     Input points
/// @param leaf_size  Downsampling resolution
/// @return           Downsampled points
template <typename InputPointCloud, typename OutputPointCloud = InputPointCloud>
std::shared_ptr<OutputPointCloud> voxelgrid_sampling_tbb(const InputPointCloud& points, double leaf_size) {
  if (traits::size(points) == 0) {
    return std::make_shared<OutputPointCloud>();
  }

  const double inv_leaf_size = 1.0 / leaf_size;
  const int coord_bit_size = 21;                       // Bits to represent each voxel coordinate (pack 21x3 = 63bits in 64bit int)
  const size_t coord_bit_mask = (1 << 21) - 1;         // Bit mask
  const int coord_offset = 1 << (coord_bit_size - 1);  // Coordinate offset to make values positive

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt(traits::size(points));
  tbb::parallel_for(tbb::blocked_range<size_t>(0, traits::size(points), 64), [&](const tbb::blocked_range<size_t>& range) {
    for (size_t i = range.begin(); i != range.end(); i++) {
      const Eigen::Array4i coord = fast_floor(traits::point(points, i) * inv_leaf_size) + coord_offset;
      if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
        std::cerr << "warning: voxel coord is out of range!!" << std::endl;
        coord_pt[i] = {0, i};
        continue;
      }

      // Compute voxel coord bits (0|1bit, z|21bit, y|21bit, x|21bit)
      const std::uint64_t bits =                                 //
        ((coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |  //
        ((coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |  //
        ((coord[2] & coord_bit_mask) << (coord_bit_size * 2));
      coord_pt[i] = {bits, i};
    }
  });

  // Sort by voxel coords
  tbb::parallel_sort(coord_pt, [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  auto downsampled = std::make_shared<OutputPointCloud>();
  traits::resize(*downsampled, traits::size(points));

  // Take block-wise sum
  const int block_size = 2048;
  std::atomic_uint64_t num_points = 0;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, traits::size(points), block_size), [&](const tbb::blocked_range<size_t>& range) {
    std::vector<Eigen::Vector4d> sub_points;
    sub_points.reserve(block_size);

    Eigen::Vector4d sum_pt = traits::point(points, coord_pt[range.begin()].second);
    for (size_t i = range.begin() + 1; i != range.end(); i++) {
      if (coord_pt[i - 1].first != coord_pt[i].first) {
        sub_points.emplace_back(sum_pt / sum_pt.w());
        sum_pt.setZero();
      }
      sum_pt += traits::point(points, coord_pt[i].second);
    }
    sub_points.emplace_back(sum_pt / sum_pt.w());

    const size_t point_index_begin = num_points.fetch_add(sub_points.size());
    for (size_t i = 0; i < sub_points.size(); i++) {
      traits::set_point(*downsampled, point_index_begin + i, sub_points[i]);
    }
  });

  traits::resize(*downsampled, num_points);

  return downsampled;
}

}  // namespace small_gicp

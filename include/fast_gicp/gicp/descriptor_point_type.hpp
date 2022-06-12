#ifndef CUSTOM_POINTCLOUD_POINT_TYPES_H
#define CUSTOM_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace descriptor_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    uint16_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud


POINT_CLOUD_REGISTER_POINT_STRUCT(descriptor_pointcloud::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint16_t, ring, ring))

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

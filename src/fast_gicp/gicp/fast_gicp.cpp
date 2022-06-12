#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/impl/fast_gicp_impl.hpp>

template class fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::FastGICP<pcl::PointNormal, pcl::PointNormal>;
template class fast_gicp::FastGICP<descriptor_pointcloud::PointXYZIRL, descriptor_pointcloud::PointXYZIRL>;

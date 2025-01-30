#pragma once
#include <vector>


#ifdef USE_BONXAI
#include <bonxai/bonxai.hpp>
#include <bonxai/probabilistic_map.hpp>
using Point3DType = Eigen::Vector3d;
using CoordType = Bonxai::CoordT;
using MapType = Bonxai::ProbabilisticMap;
using NodeType = Bonxai::CellT;
using RayType = std::vector<Bonxai::CoordT>;
#else

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
using Point3DType = octomap::point3d;
using CoordType = octomap::OcTreeKey;
using MapType = octomap::OcTree;
using NodeType = octomap::OcTreeNode*;
using RayType = octomap::KeyRay;
#endif
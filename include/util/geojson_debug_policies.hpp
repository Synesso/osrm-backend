#ifndef OSRM_GEOJSON_DEBUG_POLICIES
#define OSRM_GEOJSON_DEBUG_POLICIES

#include <vector>

#include "extractor/query_node.hpp"
#include "util/coordinate.hpp"
#include "util/json_container.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

namespace osrm
{
namespace util
{

struct NodeIdVectorToLineString
{
    NodeIdVectorToLineString(const std::vector<extractor::QueryNode> &node_coordinates);

    // converts a vector of node ids into a linestring geojson feature
    util::json::Object operator()(const std::vector<NodeID> &node_ids) const;

    const std::vector<extractor::QueryNode> &node_coordinates;
};

struct CoordinateVectorToLineString
{
    // converts a vector of node ids into a linestring geojson feature
    util::json::Object operator()(const std::vector<util::Coordinate> &coordinates) const;
};

struct NodeIdVectorToMultiPoint
{
    NodeIdVectorToMultiPoint(const std::vector<extractor::QueryNode> &node_coordinates);

    // converts a vector of node ids into a linestring geojson feature
    util::json::Object operator()(const std::vector<NodeID> &node_ids) const;

    const std::vector<extractor::QueryNode> &node_coordinates;
};

struct CoordinateVectorToMultiPoint
{
    // converts a vector of node ids into a linestring geojson feature
    util::json::Object operator()(const std::vector<util::Coordinate> &coordinates) const;
};

} /* namespace util */
} /* namespace osrm */

#endif /* OSRM_GEOJSON_DEBUG_POLICIES */

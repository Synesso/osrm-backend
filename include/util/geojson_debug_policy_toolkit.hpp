#ifndef OSRM_GEOJSON_DEBUG_POLICY_TOOLKIT_HPP
#define OSRM_GEOJSON_DEBUG_POLICY_TOOLKIT_HPP

#include "util/json_container.hpp"

#include <algorithm>

namespace osrm
{
namespace util
{
struct CoordinateToJsonArray
{
    util::json::Array operator()(const util::Coordinate coordinate)
    {
        util::json::Array json_coordinate;
        json_coordinate.values.push_back(static_cast<double>(toFloating(coordinate.lon)));
        json_coordinate.values.push_back(static_cast<double>(toFloating(coordinate.lat)));
        return json_coordinate;
    }
};

struct NodeIdToCoordinate
{
    NodeIdToCoordinate(const std::vector<extractor::QueryNode> &node_coordinates)
        : node_coordinates(node_coordinates)
    {
    }

    const std::vector<extractor::QueryNode> &node_coordinates;

    util::json::Array operator()(const NodeID nid)
    {
        auto coordinate = node_coordinates[nid];
        CoordinateToJsonArray converter;
        return converter(coordinate);
    }
};

inline util::json::Object makeFeature(std::string type, util::json::Array coordinates)
{
    util::json::Object result;
    util::json::Object properties;
    result.values["type"] = "Feature";
    result.values["properties"] = properties;
    util::json::Object geometry;
    geometry.values["type"] = std::move(type);
    geometry.values["properties"] = properties;
    geometry.values["coordinates"] = std::move(coordinates);
    result.values["geometry"] = geometry;

    return result;
}

inline util::json::Array makeJsonArray(const std::vector<util::Coordinate> &input_coordinates)
{
    util::json::Array coordinates;
    std::transform(input_coordinates.begin(),
                   input_coordinates.end(),
                   std::back_inserter(coordinates.values),
                   CoordinateToJsonArray());
    return coordinates;
}
} // namespace util
} // namespace osrm

#endif /* OSRM_GEOJSON_DEBUG_POLICY_TOOLKIT_HPP */

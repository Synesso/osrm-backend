#ifndef OSRM_EXTRACTOR_GUIDANCE_NODE_BASED_GRAPH_WALKER
#define OSRM_EXTRACTOR_GUIDANCE_NODE_BASED_GRAPH_WALKER

#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/intersection_generator.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "util/coordinate.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>
#include <boost/optional.hpp>
#include <utility>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// forward declaration to allow interaction between the intersection generator and the graph hopper
class IntersectionGenerator;

/*
 * The graph hopper is a utility that lets you find certain intersections with a node based graph,
 * accumulating information along the way
 */
class NodeBasedGraphWalker
{
  public:
    NodeBasedGraphWalker(const util::NodeBasedDynamicGraph &node_based_graph,
                         const IntersectionGenerator &intersection_generator);

    // the returned node-id, edge-id are either the last ones used, just prior accumulator
    // terminating or empty if the traversal ran into a dead end
    template <class accumulator_type, class selector_type>
    boost::optional<std::pair<NodeID, EdgeID>> TraverseRoad(NodeID starting_at_node_id,
                                                            EdgeID following_edge_id,
                                                            accumulator_type &accumulator,
                                                            const selector_type &selector);

  private:
    const util::NodeBasedDynamicGraph &node_based_graph;
    const IntersectionGenerator &intersection_generator;
};

// Accumulate all coordinates following a road until we
struct LengthLimitedCoordinateAccumulator
{
    LengthLimitedCoordinateAccumulator(
        const extractor::guidance::CoordinateExtractor &coordinate_extractor,
        const util::NodeBasedDynamicGraph &node_based_graph,
        const double max_length);

    // true if the path has traversed enough distance
    bool terminate();

    // update the accumulator
    void update(const NodeID from_node,
                const EdgeID via_edge,
                const NodeID to_node);

    const extractor::guidance::CoordinateExtractor &coordinate_extractor;
    const util::NodeBasedDynamicGraph &node_based_graph;
    const double max_length;
    double accumulated_length;
    std::vector<util::Coordinate> coordinates;
};

// The FollowRoadNameSelector tries to follow a given name along a route. We offer methods to skip
// over bridges/similar situations if desired, following narrow turns
struct SelectRoadByNameOnlyChoiceAndStraightness
{
    SelectRoadByNameOnlyChoiceAndStraightness(const NameID desired_name_id,
                                              const bool requires_entry);

    // we expect the node we are coming form, the edge we reach the intersection over, the
    // intersection itself as well as the graph to
    boost::optional<EdgeID> operator()(const NodeID nid,
                                       const EdgeID via_edge_id,
                                       const Intersection &intersection,
                                       const util::NodeBasedDynamicGraph &node_based_graph) const;

    const NameID desired_name_id;
    const bool requires_entry;
};

template <class accumulator_type, class selector_type>
boost::optional<std::pair<NodeID, EdgeID>>
NodeBasedGraphWalker::TraverseRoad(NodeID current_node_id,
                                   EdgeID current_edge_id,
                                   accumulator_type &accumulator,
                                   const selector_type &selector)
{
    // since graph hopping is used in many ways, we don't generate an adjusted intersection
    // (otherwise we could end up in infinite recursion if we call the graph hopper during the
    // adjustment itself). Relying only on `GetConnectedRoads` (which itself does no graph hopping),
    // we prevent this from happening.
    const auto stop_node_id = current_node_id;
    // we wan't to put out the last valid entries. To do so, we need to update within the following
    // while loop.
    for (std::size_t safety_hop_limit = 0; safety_hop_limit < 1000; ++safety_hop_limit)
    {
        accumulator.update(
            current_node_id, current_edge_id, node_based_graph.GetTarget(current_edge_id));

        // we have looped back to our initial intersection
        if (node_based_graph.GetTarget(current_edge_id) == stop_node_id)
            return {};

        // look at the next intersection
        const auto next_intersection =
            intersection_generator.GetConnectedRoads(current_node_id, current_edge_id);

        // don't follow u-turns or go past our initial intersection
        if (next_intersection.size() <= 1)
            return {};

        auto next_edge_id =
            selector(current_node_id, current_edge_id, next_intersection, node_based_graph);

        if (!next_edge_id)
            return {};

        if (accumulator.terminate())
            return {std::make_pair(current_node_id, current_edge_id)};

        current_node_id = node_based_graph.GetTarget(current_edge_id);
        current_edge_id = *next_edge_id;
    }

    BOOST_ASSERT(
        "Reached safety hop limit. Graph hopper seems to have been caught in an endless loop");
    return {};
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /* OSRM_EXTRACTOR_GUIDANCE_NODE_BASED_GRAPH_WALKER */

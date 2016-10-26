#include "extractor/guidance/sliproad_handler.hpp"
#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/intersection_scenario_three_way.hpp"
#include "extractor/guidance/toolkit.hpp"

#include "util/guidance/toolkit.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <utility>

#include <boost/assert.hpp>

using EdgeData = osrm::util::NodeBasedDynamicGraph::EdgeData;
using osrm::util::guidance::getTurnDirection;
using osrm::util::guidance::angularDeviation;

namespace osrm
{
namespace extractor
{
namespace guidance
{

SliproadHandler::SliproadHandler(const IntersectionGenerator &intersection_generator,
                                 const util::NodeBasedDynamicGraph &node_based_graph,
                                 const std::vector<QueryNode> &node_info_list,
                                 const util::NameTable &name_table,
                                 const SuffixTable &street_name_suffix_table)
    : IntersectionHandler(node_based_graph,
                          node_info_list,
                          name_table,
                          street_name_suffix_table,
                          intersection_generator)
{
}

// The intersection has to connect a Sliproad, see the example scenario below:
// Intersection at `d`: Sliproad `bd` connecting `cd` and the road starting at `d`.
bool SliproadHandler::canProcess(const NodeID /*nid*/,
                                 const EdgeID /*via_eid*/,
                                 const Intersection &intersection) const
{
    return intersection.size() > 2;
}

// Detect sliproad b-d in the following example:
//
//       .
//       e
//       .
//       .
// a ... b .... c .
//       `      .
//         `    .
//           `  .
//              d
//              .
//
// ^ a nid
//    ^ ab source_edge_id
//       ^ b intersection
Intersection SliproadHandler::
operator()(const NodeID /*nid*/, const EdgeID source_edge_id, Intersection intersection) const
{
    BOOST_ASSERT(intersection.size() > 2);

    // Potential splitting / start of a Sliproad (b)
    auto intersection_node_id = node_based_graph.GetTarget(source_edge_id);

    // Road index prefering non-sliproads (bc)
    auto obvious = getObviousIndexWithSliproads(source_edge_id, intersection, intersection_node_id);

    if (!obvious)
        return intersection;

    // Potential non-sliproad road (bc), leading to the intersection (c) the Sliproad (bd) shortcuts
    const auto &next_road = intersection[*obvious];
    const auto next_road_edge_data = node_based_graph.GetEdgeData(next_road.turn.eid);

    const auto source_edge_data = node_based_graph.GetEdgeData(source_edge_id);

    // The road leading to the intersection (bc) has to continue from our source
    {
        auto same_road_category =
            next_road_edge_data.road_classification == source_edge_data.road_classification;
        auto same_name = next_road_edge_data.name_id != EMPTY_NAMEID &&
                         next_road_edge_data.name_id == source_edge_data.name_id;

        if (!same_road_category || !same_name || !next_road.entry_allowed)
            return intersection;
    };

    // Link-check for (bc) and later on (cd) which both are getting shortcutted by Sliproad
    const auto is_potential_link = [this, next_road](const ConnectedRoad &road) {
        // Prevent from starting in or going onto a roundabout
        auto is_roundabout = node_based_graph.GetEdgeData(road.turn.eid).roundabout;
        auto onto_rounadbout = hasRoundaboutType(road.turn.instruction);

        // Narrow turn angle for road (bd) and guard against data issues (roads on top of each
        // other)
        auto is_narrow = angularDeviation(road.turn.angle, STRAIGHT_ANGLE) <= 2 * NARROW_TURN_ANGLE;
        auto not_same_angle = angularDeviation(next_road.turn.angle, road.turn.angle) >
                              std::numeric_limits<double>::epsilon();

        auto roundabout = is_roundabout || onto_rounadbout;

        return !roundabout && road.entry_allowed && is_narrow && not_same_angle;
    };

    if (!std::any_of(begin(intersection), end(intersection), is_potential_link))
        return intersection;

    // If the intersection is too far away, don't bother continuing
    if (getLengthToIntersection(intersection_node_id, next_road.turn.eid) > MAX_SLIPROAD_THRESHOLD)
        return intersection;

    // Try to find the intersection at (c) which the Sliproad shortcuts
    const auto next = getNextIntersection(intersection_node_id, next_road);

    if (!next)
        return intersection;

    // If we are at a traffic loop at the end of a road, don't consider it a sliproad
    if (intersection_node_id == next->node)
        return intersection;

    std::unordered_set<NameID> target_road_names;

    for (const auto &road : next->intersection)
    {
        const auto &target_data = node_based_graph.GetEdgeData(road.turn.eid);
        target_road_names.insert(target_data.name_id);
    }

    // Check all roads for Sliproads and assign appropriate TurnType
    for (auto &road : intersection)
    {
        if (!is_potential_link(road))
            continue;

        EdgeID candidate_in = road.turn.eid;

        const auto target_intersection = [&](NodeID node) {
            auto intersection = intersection_generator(node, candidate_in);
            // skip over traffic lights
            if (intersection.size() == 2)
            {
                node = node_based_graph.GetTarget(candidate_in);
                candidate_in = intersection[1].turn.eid;
                intersection = intersection_generator(node, candidate_in);
            }
            return intersection;
        }(intersection_node_id);

        // If the sliproad candidate is a through street, we cannot handle it as a sliproad.
        if (isThroughStreet(road.turn.eid, target_intersection))
            continue;

        for (const auto &candidate_road : target_intersection)
        {
            const auto &candidate_data = node_based_graph.GetEdgeData(candidate_road.turn.eid);

            // Name mismatch
            if (target_road_names.count(candidate_data.name_id) == 0)
                continue;

            if (node_based_graph.GetTarget(candidate_road.turn.eid) == next->node)
            {
                road.turn.instruction.type = TurnType::Sliproad;
                break;
            }
            else
            {
                const auto skip_traffic_light_intersection = intersection_generator(
                    node_based_graph.GetTarget(candidate_in), candidate_road.turn.eid);
                if (skip_traffic_light_intersection.size() == 2 &&
                    node_based_graph.GetTarget(skip_traffic_light_intersection[1].turn.eid) ==
                        next->node)
                {

                    road.turn.instruction.type = TurnType::Sliproad;
                    break;
                }
            }
        }
    }

    if (next_road.turn.instruction.type == TurnType::Fork)
    {
        const auto &next_data = node_based_graph.GetEdgeData(next_road.turn.eid);
        if (next_data.name_id == source_edge_data.name_id)
        {
            if (angularDeviation(next_road.turn.angle, STRAIGHT_ANGLE) < 5)
                intersection[*obvious].turn.instruction.type = TurnType::Suppressed;
            else
                intersection[*obvious].turn.instruction.type = TurnType::Continue;
            intersection[*obvious].turn.instruction.direction_modifier =
                getTurnDirection(intersection[*obvious].turn.angle);
        }
        else if (next_data.name_id != EMPTY_NAMEID)
        {
            intersection[*obvious].turn.instruction.type = TurnType::NewName;
            intersection[*obvious].turn.instruction.direction_modifier =
                getTurnDirection(intersection[*obvious].turn.angle);
        }
        else
        {
            intersection[*obvious].turn.instruction.type = TurnType::Suppressed;
        }
    }

    return intersection;
}

// Implementation details

// Skips over `tl` traffic light and returns
//
//  a ... tl ... b .. c
//               .
//               .
//               d
//
//  ^ at
//     ^ road
boost::optional<SliproadHandler::IntersectionAndNode>
SliproadHandler::getNextIntersection(const NodeID at, const ConnectedRoad &road) const
{
    auto intersection = intersection_generator(at, road.turn.eid);
    auto in_edge = road.turn.eid;
    auto intersection_node = node_based_graph.GetTarget(in_edge);

    // To prevent ending up in an endless loop, we remember all visited nodes. This is
    // necessary, since merging of roads can actually create enterable loops of degree two
    std::unordered_set<NodeID> visited_nodes;

    auto node = at;
    while (intersection.size() == 2 && visited_nodes.count(node) == 0)
    {
        visited_nodes.insert(node);
        node = node_based_graph.GetTarget(in_edge);

        // We ended up in a loop without exit
        if (node == at)
        {
            return boost::none;
        }

        in_edge = intersection[1].turn.eid;
        intersection = intersection_generator(node, in_edge);
        intersection_node = node_based_graph.GetTarget(in_edge);
    }

    if (intersection.size() <= 2)
    {
        return boost::none;
    }

    return boost::make_optional(IntersectionAndNode{intersection, intersection_node});
}

boost::optional<std::size_t> SliproadHandler::getObviousIndexWithSliproads(
    const EdgeID from, const Intersection &intersection, const NodeID at) const
{
    BOOST_ASSERT(from != SPECIAL_EDGEID);
    BOOST_ASSERT(at != SPECIAL_NODEID);

    // If a turn is obvious without taking Sliproads into account use this
    const auto index = findObviousTurn(from, intersection);

    if (index != 0)
        return boost::make_optional(index);

    // Otherwise check if the road is forking into two and one of them is a Sliproad;
    // then the non-Sliproad is the obvious one.
    if (intersection.size() != 3)
        return boost::none;

    const auto forking = intersection[1].turn.instruction.type == TurnType::Fork &&
                         intersection[2].turn.instruction.type == TurnType::Fork;

    if (!forking)
        return boost::none;

    const auto first = getNextIntersection(at, intersection[1]);
    const auto second = getNextIntersection(at, intersection[2]);

    if (!first || !second)
        return boost::none;

    // In case of loops at the end of the road, we will arrive back at the intersection
    // itself. If that is the case, the road is obviously not a sliproad.
    if (canBeTargetOfSliproad(first->intersection) && at != second->node)
        return boost::make_optional(std::size_t{2});

    if (canBeTargetOfSliproad(second->intersection) && at != first->node)
        return boost::make_optional(std::size_t{1});

    return boost::none;
}

int SliproadHandler::getLengthToIntersection(const NodeID start, const EdgeID onto) const
{
    BOOST_ASSERT(start != SPECIAL_NODEID);
    BOOST_ASSERT(onto != SPECIAL_EDGEID);

    using namespace util::coordinate_calculation;

    auto extractor = intersection_generator.GetCoordinateExtractor();
    auto coordinates = extractor.GetForwardCoordinatesAlongRoad(start, onto);
    return getLength(coordinates, &haversineDistance);
}

bool SliproadHandler::isThroughStreet(const EdgeID from, const Intersection &intersection) const
{
    BOOST_ASSERT(from != SPECIAL_EDGEID);
    BOOST_ASSERT(!intersection.empty());

    const auto edge_name_id = node_based_graph.GetEdgeData(from).name_id;

    auto first = begin(intersection) + 1; // Skip UTurn road
    auto last = end(intersection);

    auto same_name = [&](auto road) {
        return node_based_graph.GetEdgeData(road.turn.eid).name_id == edge_name_id;
    };

    return std::find_if(first, last, same_name) != last;
}

bool SliproadHandler::canBeTargetOfSliproad(const Intersection &intersection)
{
    // Example to handle:
    //       .
    // a . . b .
    //  `    .
    //    `  .
    //       c    < intersection
    //       .
    //

    // One outgoing two incoming
    if (intersection.size() != 3)
        return false;

    // For (c) to be target of a Sliproad (ab) and (bc) have to be directed
    auto backwards = intersection[0].entry_allowed;
    auto undirected_link = intersection[1].entry_allowed && intersection[2].entry_allowed;

    if (backwards || undirected_link)
        return false;

    return true;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

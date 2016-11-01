#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/toolkit.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

ConnectedRoad::ConnectedRoad(const TurnOperation turn, const bool entry_allowed)
    : entry_allowed(entry_allowed), turn(turn)
{
}

std::string toString(const ConnectedRoad &road)
{
    std::string result = "[connection] ";
    result += std::to_string(road.turn.eid);
    result += " allows entry: ";
    result += std::to_string(road.entry_allowed);
    result += " angle: ";
    result += std::to_string(road.turn.angle);
    result += " bearing: ";
    result += std::to_string(road.turn.bearing);
    result += " instruction: ";
    result += std::to_string(static_cast<std::int32_t>(road.turn.instruction.type)) + " " +
              std::to_string(static_cast<std::int32_t>(road.turn.instruction.direction_modifier)) +
              " " + std::to_string(static_cast<std::int32_t>(road.turn.lane_data_id));
    return result;
}

Intersection::iterator findClosestTurn(Intersection &intersection, const double angle)
{
    return std::min_element(intersection.begin(),
                            intersection.end(),
                            [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                return angularDeviation(lhs.turn.angle, angle) <
                                       angularDeviation(rhs.turn.angle, angle);
                            });
}
Intersection::const_iterator findClosestTurn(const Intersection &intersection, const double angle)
{
    return std::min_element(intersection.cbegin(),
                            intersection.cend(),
                            [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
                                return angularDeviation(lhs.turn.angle, angle) <
                                       angularDeviation(rhs.turn.angle, angle);
                            });
}

Intersection::Base::iterator Intersection::findClosestTurn(double angle)
{
    return std::min_element(
        this->begin(), this->end(), [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
            return util::guidance::angularDeviation(lhs.turn.angle, angle) <
                   util::guidance::angularDeviation(rhs.turn.angle, angle);
        });
}

Intersection::Base::const_iterator Intersection::findClosestTurn(double angle) const
{
    return std::min_element(
        this->begin(), this->end(), [angle](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
            return util::guidance::angularDeviation(lhs.turn.angle, angle) <
                   util::guidance::angularDeviation(rhs.turn.angle, angle);
        });
}

std::uint8_t Intersection::getLaneCount(const util::NodeBasedDynamicGraph &node_based_graph) const
{
    const auto extract_lanes = [&node_based_graph](const ConnectedRoad &road) {
        return node_based_graph.GetEdgeData(road.turn.eid).road_classification.GetNumberOfLanes();
    };
    return extract_lanes(*std::max_element(
        this->begin(),
        this->end(),
        [&node_based_graph, extract_lanes](const ConnectedRoad &lhs, const ConnectedRoad &rhs) {
            return extract_lanes(lhs) < extract_lanes(rhs);
        }));
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

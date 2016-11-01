#ifndef POLYLINECOMPRESSOR_H_
#define POLYLINECOMPRESSOR_H_

#include "util/coordinate.hpp"

#include <string>
#include <vector>

namespace osrm
{
namespace engine
{

using CoordVectorForwardIter = std::vector<util::Coordinate>::const_iterator;
// Encodes geometry into polyline format.
// See: https://developers.google.com/maps/documentation/utilities/polylinealgorithm
std::string encodePolyline(CoordVectorForwardIter begin, CoordVectorForwardIter end, double precision = 1e5);

// Decodes geometry from polyline format
// See: https://developers.google.com/maps/documentation/utilities/polylinealgorithm
std::vector<util::Coordinate> decodePolyline(const std::string &polyline, double precision = 1e5);
}
}

#endif /* POLYLINECOMPRESSOR_H_ */

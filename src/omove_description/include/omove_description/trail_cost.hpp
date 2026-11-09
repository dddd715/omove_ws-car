#pragma once
#include <algorithm>
#include <cmath>
namespace omove_description {
inline unsigned char trailPenalty(double distance, double age, double radius,
  double grace, double ttl, double peak)
{
  if (age<grace || age>=ttl || distance>=radius) {return 0;}
  return static_cast<unsigned char>(std::lround(
    peak*(1.-distance/radius)*(ttl-age)/(ttl-grace)));
}
inline unsigned char addTrailCost(unsigned char original, unsigned char penalty)
{
  // Preserve unknown, lethal and inscribed obstacle costs exactly.
  if (original>=253) {return original;}
  return static_cast<unsigned char>(std::min(252, int(original)+int(penalty)));
}
}

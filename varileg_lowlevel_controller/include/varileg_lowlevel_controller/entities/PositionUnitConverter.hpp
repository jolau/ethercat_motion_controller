//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP

class PositionUnitConverter {
 public:
  PositionUnitConverter() = default;
  PositionUnitConverter(int conversionFactor) : conversionFactor_(conversionFactor) {}
  float toRad(const int &inc) {
    return ((float) inc) / conversionFactor_;
  };
  int toInc(const float &rad) {
    return (int) rad * conversionFactor_;
  };
 private:
  int conversionFactor_;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP

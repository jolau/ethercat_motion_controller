//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP

class PositionUnitConverter {
 public:
  PositionUnitConverter() = default;
  PositionUnitConverter(double conversionFactor) : conversionFactor_(conversionFactor) {}
  double toRad(const int &inc) {
    return ((float) inc) / conversionFactor_;
  };
  int toInc(const double &rad) {
    return rad * conversionFactor_;
  };
 private:
  double conversionFactor_;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP

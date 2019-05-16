//
// Created by jolau on 07.04.19.
//

#ifndef VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP
#define VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP

/**
 * Converts between radian and increments (encoder ticks)
 */
class PositionUnitConverter {
 public:
  PositionUnitConverter() = default;
  PositionUnitConverter(double conversionFactor) : conversionFactor_(conversionFactor) {}
  /**
   * Convert increments (encoder ticks) to radian. Conversion: rad = inc / #conversionFactor_
   * @param inc position in increments (encoder ticks)
   * @return position in radian
   */
  double toRad(const int &inc) {
    return ((float) inc) / conversionFactor_;
  };

  /**
   * Convert radian to increments (encoder ticks). Conversion: inc = rad * #conversionFactor_
   * @param rad position in radian
   * @return position in increments (encoder ticks)
   */
  int toInc(const double &rad) {
    return rad * conversionFactor_;
  };

  double getConversionFactor(){
    return conversionFactor_;
  }

 private:
  double conversionFactor_;
};

#endif //VARILEG_LOWLEVEL_CONTROLLER_POSITIONUNITCONVERTER_HPP

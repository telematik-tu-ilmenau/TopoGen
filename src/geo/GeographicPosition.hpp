#ifndef GEOGRAPHICPOSITION_HPP
#define GEOGRAPHICPOSITION_HPP

#include "util/Util.hpp"
#include <cassert>
#include <utility>

typedef std::pair<double, double> GeographicPositionTuple;

class GeographicPosition {
   public:
    GeographicPosition() : _latitude(0), _longitude(0), _valid(false) {}

    GeographicPosition(double latitude, double longitude) : _latitude(latitude), _longitude(longitude), _valid(true) {
        assert(Util::checkBounds(latitude, longitude));
    }

    virtual void setLat(double val) { _latitude = val; }

    virtual void setLon(double val) { _longitude = val; }

    double lat() { return _latitude; }

    double lon() { return _longitude; }

    void setInvalid() { _valid = false; }

    void setValid() { _valid = true; }

    bool isValid() { return _valid; }

    virtual ~GeographicPosition() {};

   protected:
    double _latitude;
    double _longitude;
    bool _valid;
};

#endif  // GEOGRAPHICPOSITION_HPP

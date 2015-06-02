#ifndef GEOGRAPHICPOSITION_HPP
#define GEOGRAPHICPOSITION_HPP

#include <cassert>
#include <utility>

typedef std::pair<double,double> GeographicPositionTuple;

template <typename T>
class GeographicPosition {
public:
    GeographicPosition() : _latitude(0), _longitude(0), _valid(false) {}

    GeographicPosition(T latitude, T longitude) : _latitude(latitude), _longitude(longitude), _valid(true) {
        assert(-90.0 <= latitude && latitude <= 90.0);
        assert(-180.0 <= longitude && longitude <= 180.0);
    }

    virtual void setLat(T val) {
        _latitude = val;
    }

    virtual void setLon(T val) {
        _longitude = val;
    }

    T lat() {
        return _latitude;
    }

    T lon() {
        return _longitude;
    }

    void setInvalid() {
        _valid = false;
    }

    void setValid() {
        _valid = true;
    }

    bool isValid() {
        return _valid;
    }

protected:
    T _latitude;
    T _longitude;
    bool _valid;
};

#endif // GEOGRAPHICPOSITION_HPP


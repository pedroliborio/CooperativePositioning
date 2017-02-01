/*
 * GPS.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_GPS_GPS_H_
#define LOCALIZATION_GPS_GPS_H_


#include <Types.h>
#include <fstream>

namespace Localization {

class GPS {
private:
    LonLat gpsOutGeoPos, gpsRecGeoPos; //GPS outage and recovery positions...
    Coord gpsOutUTMPos, gpsRecUTMPos;
    double errorGPSOut, errorGPSRec; //Errors at points of outage and recovery
public:
    GPS();
    GPS(std::string outagesFile);
    virtual ~GPS();
    void GetOutageInformation(std::string outagesFile);

    double getErrorGpsOut() const {
        return errorGPSOut;
    }

    void setErrorGpsOut(double errorGpsOut) {
        errorGPSOut = errorGpsOut;
    }

    double getErrorGpsRec() const {
        return errorGPSRec;
    }

    void setErrorGpsRec(double errorGpsRec) {
        errorGPSRec = errorGpsRec;
    }

    const LonLat& getGpsOutGeoPos() const {
        return gpsOutGeoPos;
    }

    void setGpsOutGeoPos(const LonLat& gpsOutGeoPos) {
        this->gpsOutGeoPos = gpsOutGeoPos;
    }

    const Coord& getGpsOutUtmPos() const {
        return gpsOutUTMPos;
    }

    void setGpsOutUtmPos(const Coord& gpsOutUtmPos) {
        gpsOutUTMPos = gpsOutUtmPos;
    }

    const LonLat& getGpsRecGeoPos() const {
        return gpsRecGeoPos;
    }

    void setGpsRecGeoPos(const LonLat& gpsRecGeoPos) {
        this->gpsRecGeoPos = gpsRecGeoPos;
    }

    const Coord& getGpsRecUtmPos() const {
        return gpsRecUTMPos;
    }

    void setGpsRecUtmPos(const Coord& gpsRecUtmPos) {
        gpsRecUTMPos = gpsRecUtmPos;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_GPS_GPS_H_ */

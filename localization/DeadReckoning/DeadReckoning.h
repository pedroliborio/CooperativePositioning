/*
 * DeadReckoning.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_DEADRECKONING_DEADRECKONING_H_
#define LOCALIZATION_DEADRECKONING_DEADRECKONING_H_

#include <Types.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>
#include <limits.h>
#include <iomanip>

using namespace GeographicLib;

namespace Localization {

class DeadReckoning {
protected:
    //For a frequency of 10Hz
    //FIXME After exchange for an automatic frequency given by Update Interval parameter from omnetpp.ini
    const double ANGLE_RANDOM_WALK_NOISE = 0.063245553;// 0.02°/s/sqrt(Hz)
    const double OFFSET =  0.1;// 1°/s (0.1 in 10Hz)
    const double NON_LINEARITY = 0.1; //1/°s (0.1 in 10Hz)
    //FIXME Put sources of error of odometer
private:
    LonLat lastKnowPosGeo;
    Coord lastKnowPosUTM;
    double angle;
    double errorGeo;
    double errorUTM;
    double timeDR;
    //double angularError; // constant bias

    //This simulate one of teh error sources in a gyroscope and is added to the angle
    //given by sumo in UTM coordinates and Geographiclib in lat lon corrdinates
    double sigma_theta;
public:
    DeadReckoning();
    DeadReckoning(LonLat lastGPSPos);
    virtual ~DeadReckoning();
    void setGeoPos(LonLat *lastSUMOPos, LonLat *atualSUMOPos);
    void setUTMPos(Coord utmCoord);
    void setErrorLonLat(LonLat *atualSUMOPos);
    void setErrorUTMPos(Coord *atualSUMOPos);


    double getErrorGeo() const {
        return errorGeo;
    }

    double getErrorUtm() const {
        return errorUTM;
    }

    const LonLat& getLastKnowPosGeo() const {
        return lastKnowPosGeo;
    }

    const Coord& getLastKnowPosUtm() const {
        return lastKnowPosUTM;
    }

    void setLastKnowPosGeo(const LonLat& lastKnowPosGeo) {
        this->lastKnowPosGeo = lastKnowPosGeo;
    }

    void setLastKnowPosUtm(const Coord& lastKnowPosUtm) {
        lastKnowPosUTM = lastKnowPosUtm;
    }

    void setErrorGeo(double errorGeo) {
        this->errorGeo = errorGeo;
    }

    void setErrorUtm(double errorUtm) {
        errorUTM = errorUtm;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_DEADRECKONING_DEADRECKONING_H_ */

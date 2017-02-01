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

using namespace GeographicLib;

namespace Localization {

class DeadReckoning {
private:
    LonLat lastKnowPosGeo;
    Coord lastKnowPosUTM;
    double errorGeo;
    double errorUTM;
public:
    DeadReckoning();
    DeadReckoning(LonLat lastGPSPos);
    virtual ~DeadReckoning();
    void setGeoPos(LonLat *lastSUMOPos, LonLat *atualSUMOPos);
    void setUTMPos(Coord utmCoord);
    void setErrorLonLat(LonLat *atualSUMOPos);
    void setErrorUTM(Coord *atualSUMOPos);


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
};

} /* namespace Localization */

#endif /* LOCALIZATION_DEADRECKONING_DEADRECKONING_H_ */

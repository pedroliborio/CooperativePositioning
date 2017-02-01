/*
 * Projection.h
 *
 *  Created on: Jan 17, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_PROJECTIONS_PROJECTION_H_
#define LOCALIZATION_PROJECTIONS_PROJECTION_H_

#include <proj_api.h>
#include <Projection.h>
#include <Types.h>
#include <fstream>
#include <iomanip>
#include <limits>

namespace Localization {

class Projection {
private:
    Coord utmCoord;
    LonLat geoCoord;
    projXY netOffset;
    projPJ pj_utm;

public:
    Projection();
    Projection(std::string fileProjName);
    virtual ~Projection();
    void FromUTMToLonLat(void);
    void FromLonLatToUTM(void);

    const LonLat& getGeoCoord() const {
        return geoCoord;
    }

    void setGeoCoord(const LonLat& geoCoord) {
        this->geoCoord = geoCoord;
    }

    const Coord& getUtmCoord() const {
        return utmCoord;
    }

    void setUtmCoord(const Coord& utmCoord) {
        this->utmCoord = utmCoord;
    }

    const projXY& getNetOffset() const {
        return netOffset;
    }

    void setNetOffset(const projXY& netOffset)
    {
        this->netOffset = netOffset;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_PROJECTIONS_PROJECTION_H_ */

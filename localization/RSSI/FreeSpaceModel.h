/*
 * FreeSpaceModel.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_FREESPACEMODEL_H_
#define LOCALIZATION_RSSI_FREESPACEMODEL_H_

#include<cmath>

namespace Localization {

class FreeSpaceModel {
private:
    double rssi;
    double distance;
public:
    FreeSpaceModel();
    virtual ~FreeSpaceModel();
    void setRSSI(double distance, double pTx, double alpha, double lambda);
    void setDistance(double rssi, double pTx, double alpha, double lambda);

    double getDistance() const {
        return distance;
    }

    double getRSSI() const {
        return rssi;
    }

};

} /* namespace Localization */

#endif /* LOCALIZATION_RSSI_FREESPACEMODEL_H_ */

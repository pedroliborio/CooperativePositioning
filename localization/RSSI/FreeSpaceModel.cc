/*
 * FreeSpaceModel.cc
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <RSSI/FreeSpaceModel.h>

namespace Localization {

FreeSpaceModel::FreeSpaceModel() {
    // TODO Auto-generated constructor stub

}

FreeSpaceModel::~FreeSpaceModel() {
    // TODO Auto-generated destructor stub
}

void FreeSpaceModel::setRSSI(double distance, double pTx, double alpha, double lambda){
    rssi = 10*log10(pTx) - 10 * log10((16* M_PI * M_PI * pow(distance, alpha)) / pow(lambda,alpha));
    //dBmW
}

void FreeSpaceModel::setDistance(double rssi, double pTx, double alpha, double lambda){
    distance = (lambda / (pow((4 * M_PI),(2/alpha)) ) ) * (pow(pTx, (1/alpha))) * (pow(10,( - (rssi / (10*alpha) ) )));
    //meters
}

} /* namespace Localization */

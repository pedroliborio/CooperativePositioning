/*
 * Outage.cc
 *
 *  Created on: Jan 23, 2017
 *      Author: liborio
 */

#include <Outage/Outage.h>

namespace Localization {

Outage::Outage() {
    // TODO Auto-generated constructor stub


}

Outage::Outage(Coord outPos, Coord recPos) {
    outagePos = outPos;
    recoverPos = recPos;
    inOutage = false;
    inRecover = false;
}

Outage::~Outage() {
    // TODO Auto-generated destructor stub
}

void Outage::ControlOutage(Coord *sumoPos){
    double dist;
    if(inOutage == false && inRecover==false ){
        dist = outagePos.sqrdist(*sumoPos);
        if(dist > distOutage){
            inOutage = true;
        }
        else{
            distOutage = dist;
        }
    }
    else{
        if(inOutage == true && inRecover==false ){
            dist = recoverPos.sqrdist(*sumoPos);
            if(dist > distRecover){
                inOutage = false;
                inRecover = true;
            }
            else{
                distRecover = dist;
            }
        }
    }
}

} /* namespace Localization */

//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __COOPERATIVEPOSITIONING_RSU_H_
#define __COOPERATIVEPOSITIONING_RSU_H_

//#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include <communication/LocAppCom.h>

/**
 * Small RSU Demo using 11p
 */
class RSU : public LocAppCom {
    public:
        virtual void initialize(int stage);
    private:
        std::list<BasicSafetyMessage*> listFWDBeacons;
    protected:
        /** @brief handle self messages */
        virtual void handleSelfMsg(cMessage* msg);
        virtual void onBSM(BasicSafetyMessage* bsm);
        bool BeaconIsDuplicated(BasicSafetyMessage* bsm);
        void PutBeaconInformation(BasicSafetyMessage* bsm);
        void AddBeaconToForward(BasicSafetyMessage* fwdBSM);
        BaseMobility *baseMob;
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;
        //void DeleteOldBeaconToForward();
};

#endif

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

#include <Outages.h>

Define_Module(Outages);

void Outages::initialize(int stage)
{
    // TODO - Generated method body
    indexEntranceExit = 1;
    indexExitEntrance = 1;

    sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
    sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);
    sendFWDBeaconEvt = new cMessage("fwd beacon evt", SEND_FWDBEACON_EVT);


}

/*void Outages::handleSelfMsg(cMessage* msg){
    BaseWaveApplLayer::handleSelfMsg(msg);
}*/



OutCoord Outages::getOutage(std::string routeFile){

    //std::cout << routeFile << endl;
    //std::cout << indexEntranceExit << endl;
    //std::cout << indexExitEntrance << endl;

    OutCoord coords;
    if(routeFile == "DMATEntranceExit"){
        coords.outage.x = dmatEntranceExit[indexEntranceExit-1][0];
        coords.outage.y = dmatEntranceExit[indexEntranceExit-1][1];
        coords.recover.x = dmatEntranceExit[indexEntranceExit][0];
        coords.recover.y = dmatEntranceExit[indexEntranceExit][1];
        indexEntranceExit+=2;
        if(indexEntranceExit > (DMAT_SIZE)){
            indexEntranceExit = 1;
        }
        return coords;
    }

    if(routeFile == "DMATExitEntrance"){
        coords.outage.x = dmatExitEntrance[indexExitEntrance-1][0];
        coords.outage.y = dmatExitEntrance[indexExitEntrance-1][1];
        coords.recover.x = dmatExitEntrance[indexExitEntrance][0];
        coords.recover.y = dmatExitEntrance[indexExitEntrance][1];
        indexExitEntrance+=2;
        if(indexExitEntrance > (DMAT_SIZE)){
            indexExitEntrance = 1;
        }
        return coords;
    }

    if(routeFile == "DPTEntranceExit"){
        coords.outage.x = dptEntranceExit[indexEntranceExit-1][0];
        coords.outage.y = dptEntranceExit[indexEntranceExit-1][1];
        coords.recover.x = dptEntranceExit[indexEntranceExit][0];
        coords.recover.y = dptEntranceExit[indexEntranceExit][1];
        indexEntranceExit+=2;
        if(indexEntranceExit > (DPT_SIZE)){
            indexEntranceExit = 1;
        }
        return coords;
    }

    if(routeFile == "DPTExitEntrance"){
        coords.outage.x = dptExitEntrance[indexExitEntrance-1][0];
        coords.outage.y = dptExitEntrance[indexExitEntrance-1][1];
        coords.recover.x = dptExitEntrance[indexExitEntrance][0];
        coords.recover.y = dptExitEntrance[indexExitEntrance][1];
        indexExitEntrance+=2;
        if(indexExitEntrance > (DPT_SIZE)){
            indexExitEntrance = 1;
        }
        return coords;
    }

    return coords;
}

Outages::~Outages() {
    //statistics recording goes here

}

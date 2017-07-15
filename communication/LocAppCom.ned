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

package com.liborio.cooperativepositioning.communication;

import org.car2x.veins.base.modules.IBaseApplLayer;

//simple LocAppCom like IBaseApplLayer
//{
//    @class(LocAppCom);
//    @display("i=block/wrxtx");
//    string appName = default("My first Veins App!");

//
// WAVE application layer base class.
//
// @author David Eckhoff
//
// @see BaseWaveApplLayer
// @see Nic80211p
// @see Mac1609_4
// @see PhyLayer80211p
//
@class(LocAppCom);
@display("i=block/wrxtx");
simple LocAppCom like IBaseApplLayer
{
    parameters:
        bool debug = default(false); //whether to print debug output

        int headerLength = default(88bit) @unit(bit); //header length of the application

        bool sendBeacons = default(true); //tell the applayer to periodically send beacons
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconPriority = default(3); //the access category of the beacon messages
        double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages

        int dataLengthBits = default(1024bit) @unit(bit); //the length of a data packet
        bool dataOnSch = default(false); //tells the applayer whether to use a service channel for datapackets or the control channel
        int dataPriority = default(3); //the default access category for data packets

        bool avoidBeaconSynchronization = default(true); //don't start beaconing directly after node was created but delay to avoid artifical synchronization

        bool sendWSA = default(false);
        int wsaLengthBits = default(250bit) @unit(bit);
        double wsaInterval =  default(1s) @unit(s);

        bool communicateWhileParked = default(false); //whether the car should communicate when it is parked in SUMO
        
        bool forwardBeacons = default(false); //forward beacons?
        int numHops = default(1); //default number of hops is a single hop approach
        double forwardInterval = default(1s) @unit(s);

    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
    
}

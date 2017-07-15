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

#include "RSU.h"
Define_Module(RSU);

void RSU::initialize(int stage) {
    BaseApplLayer::initialize(stage);
    if (stage==0) {

        //initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = NULL;
            mobility = NULL;
            traciVehicle = NULL;
        }

        //Getting pointer to access RSU coordinates
        baseMob = FindModule<BaseMobility*>::findSubModule(getParentModule());


        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
                getParentModule());
        assert(mac);

        myId = getParentModule()->getId();

        //Single or Multihop
        //multihop = par("multihop").boolValue();

        //read parameters
        headerLength = par("headerLength").longValue();
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits").longValue();
        beaconPriority = par("beaconPriority").longValue();
        beaconInterval =  par("beaconInterval").doubleValue();

        forwardBeacons = par("forwardBeacons").boolValue();
        forwardInterval = par("forwardInterval").doubleValue();
        numHops = par("numHops").longValue();

        dataLengthBits = par("dataLengthBits").longValue();
        dataOnSch = par("dataOnSch").boolValue();
        dataPriority = par("dataPriority").longValue();

        wsaInterval = par("wsaInterval").doubleValue();
        communicateWhileParked = par("communicateWhileParked").boolValue();
        currentOfferedServiceId = -1;

        isParked = false;


        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);
        sendFWDBeaconEvt = new cMessage("fwd beacon evt", SEND_FWDBEACON_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        generatedFWDBSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;
        receivedFWDBSMs = 0;

    }
    else if (stage == 1) {
        //simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            std::cerr << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if ( beaconInterval.raw() % (mac->getSwitchingInterval().raw()*2)) {
                    std::cerr << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2*mac->getSwitchingInterval() << "). "
                            << "This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, type_CCH);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }

            //FIXME Added By me initializing beriodic forwarding...
            if(forwardBeacons){
                scheduleAt(computeAsynchronousSendingTime(forwardInterval, type_CCH),sendFWDBeaconEvt);
            }
        }
    }
}

void RSU::handleSelfMsg(cMessage *msg){
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            BasicSafetyMessage* bsm = new BasicSafetyMessage();

            //RSU put their own fixed position.
            PutBeaconInformation(bsm);
            //Put veins control parameters
            populateWSM(bsm);
            //send it to mac layer...
            sendDown(bsm);
            //schedule next periodic beacon event...
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
            break;
        }
        case SEND_WSA_EVT:   {
            WaveServiceAdvertisment* wsa = new WaveServiceAdvertisment();
            populateWSM(wsa);
            sendDown(wsa);
            scheduleAt(simTime() + wsaInterval, sendWSAEvt);
            break;
        }
        case SEND_FWDBEACON_EVT:{
            if(forwardBeacons){
                //FIXME Maybe is good only forward beacons from vehicles...
                //std::cout << "Evento de forwarding" << endl;
                //Verificar se tem beacons com TTL expirados e deletar.
                //DeleteOldBeaconToForward();
                //Verificar se tem beacon na lista e fazer forwarding e depois deletar.
                if(!this->listFWDBeacons.empty()){
                    //envia um beacon e o retira da lista de beacons para envio
                    sendDown(this->listFWDBeacons.front());
                    //std::cout << "forwarding beacon: " << listFWDBeacons.front()->getPersistentID() << endl;
                    this->listFWDBeacons.pop_front();
                }
                //agenda proximo envio periodico
                scheduleAt(simTime()+ forwardInterval, sendFWDBeaconEvt);
            }
            break;
        }
        default: {
            if (msg)
                DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }

}

void RSU::onBSM(BasicSafetyMessage* bsm){

    if (BeaconIsDuplicated(bsm) || BeaconHaveMyId(bsm) || (!BeaconIsAlive(bsm)) || bsm->getInOutage() ){

            //std::cout <<"do nothing..." << endl;
            //delete(bsm);
            return;
    }
    else{
        if(forwardBeacons){
           BasicSafetyMessage *fwdBSM = bsm->dup();
           fwdBSM->setPersistentID(bsm->getPersistentID());
           fwdBSM->setHops(bsm->getHops()+1);
           //std::cout << bsm->getPersistentID() << "<- BSM, FWDBSM ->"<< fwdBSM->getPersistentID() << endl;

           //Adiciona o beacon para lista de beacons para encaminhamento...
           AddBeaconToForward(fwdBSM);
       }
    }
}

bool RSU::BeaconIsDuplicated(BasicSafetyMessage* bsm){
    for(std::list<BasicSafetyMessage*>::iterator it=this->listFWDBeacons.begin(); it!= this->listFWDBeacons.end(); ++it){
        if((*it)->getPersistentID() == bsm->getPersistentID()){
            //std::cout <<"beacon duplicated:"<<" list: "<< (*it)->getPersistentID() << " beacon: " << bsm->getPersistentID() << endl;
            return true;
        }
    }
    return false;
}

void RSU::PutBeaconInformation(BasicSafetyMessage *bsm){

    //Put unique persistent ID for control of FORWARDING purposes
    bsm->setPersistentID(bsm->getId());

    bsm->setIsRSUBSM(true);

    bsm->setHops(0);

    //Put Fixed RSU position on beacon and disseminate it...
    //This position have omnet coordinates. If a vehicle receive,
    //it will convert to TraCI coordinates before process beacon information...
    bsm->setSenderRealPos(baseMob->getCurrentPosition());
}

void RSU::AddBeaconToForward(BasicSafetyMessage* fwdBSM){
    //std::cout << myId <<" putting beacon on list: " << fwdBSM->getPersistentID() <<' '<< fwdBSM->getSenderAddress()  <<' '<< fwdBSM->getTimestamp()<< endl;
    this->listFWDBeacons.push_back(fwdBSM);
}

/*void RSU::DeleteOldBeaconToForward(){
    for(std::list<BasicSafetyMessage*>::iterator it=listFWDBeacons.begin(); it!= listFWDBeacons.end();){

        if( (simTime() - (*it)->getTimestamp()) > 0.5){
      //      std::cout <<" deleting old beacon"<< (*it)->getPersistentID() <<endl;
            delete *it;
            it = this->listFWDBeacons.erase(it);
        }
        else{
            ++it;
        }
    }
}*/

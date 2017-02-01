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

#include "LocAppCom.h"

const simsignalwrap_t LocAppCom::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

Define_Module(LocAppCom);

//TraCIConnection* conn = FindModule<TraCIConnection*>::findSubModule(getParentModule)



void LocAppCom::initialize(int stage){
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        mobility = TraCIMobilityAccess().get(getParentModule());
        //connection = FindModule<TraCIConnection*>::findSubModule(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        timeSeed = time(0);

        //Initialize Projection...
        //size of (EntranceExit or ExitEntrance ) == 12
        projection = new Projection( traciVehicle->getRouteId().substr( 0,(traciVehicle->getRouteId().size() - 12) ) );

        //Initialize SUMO Positions tracker
        lastSUMOUTMPos = traci->getTraCIXY(mobility->getCurrentPosition());
        atualSUMOUTMPos = lastSUMOUTMPos;
        projection->setUtmCoord(lastSUMOUTMPos);
        projection->FromUTMToLonLat();
        lastSUMOGeoPos = projection->getGeoCoord();
        atualSUMOGeoPos = lastSUMOGeoPos;

        //Using the route of vehicle to get the information on dataset
        //The name of the route is the same of the equivalent outages dataset.
        gpsModule = new GPS(traciVehicle->getRouteId());

        projection->setGeoCoord(gpsModule->getGpsOutGeoPos());
        projection->FromLonLatToUTM();
        gpsModule->setGpsOutUtmPos(projection->getUtmCoord());
        projection->setGeoCoord(gpsModule->getGpsRecGeoPos());
        projection->FromLonLatToUTM();
        gpsModule->setGpsRecUtmPos(projection->getUtmCoord());

        /*std::cout << "GPS: "
        <<" - "<< myId
        <<" - "<< gpsModule->getGpsOutUtmPos()
        <<" - "<< std::setprecision(10) << gpsModule->getGpsOutGeoPos().lon <<", " << std::setprecision(10) << gpsModule->getGpsOutGeoPos().lat
        <<" - "<< gpsModule->getGpsRecUtmPos()
        <<" - "<< std::setprecision(10) << gpsModule->getGpsRecGeoPos().lon <<", " << std::setprecision(10) << gpsModule->getGpsRecGeoPos().lat
        <<" - "<< std::setprecision(10) << gpsModule->getErrorGpsOut()
        <<" - "<< std::setprecision(10) << gpsModule->getErrorGpsRec()
        << "\n\n";*/

        outageModule = new Outage(gpsModule->getGpsOutUtmPos(), gpsModule->getGpsRecUtmPos());

        /*std::cout << "Outage: "
                    <<" - "<< myId
                    <<" - "<< outageModule->getDistOutage()
                    <<" - "<< outageModule->getDistRecover()
                    <<" - "<< outageModule->getOutagePos()
                    <<" - "<< outageModule->getRecoverPos()
                    << "\n\n";*/

        //GDR Module
        drModule =  new DeadReckoning(gpsModule->getGpsOutGeoPos());
        projection->setGeoCoord(drModule->getLastKnowPosGeo());
        projection->FromLonLatToUTM();
        drModule->setUTMPos(projection->getUtmCoord());

        //Filters
        filter = new Filters();

        //Multilateration Module
        multilateration = new Multilateration();

        //Phy Models for RSSI
        fsModel = new FreeSpaceModel();
        trgiModel = new TwoRayInterferenceModel();

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

    }

}

void LocAppCom::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            WaveShortMessage* wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
            Coord currentPos = traci->getTraCIXY(mobility->getCurrentPosition());

            lastSUMOUTMPos = atualSUMOUTMPos;
            lastSUMOGeoPos = atualSUMOGeoPos;
            atualSUMOUTMPos = currentPos;
            projection->setUtmCoord(atualSUMOUTMPos);
            projection->FromUTMToLonLat();
            atualSUMOGeoPos = projection->getGeoCoord();

            //Real Position
            wsm->setSenderRealPos(atualSUMOUTMPos);
            outageModule->ControlOutage(&atualSUMOUTMPos);

            //antes da queda
            if(!outageModule->isInOutage() && !outageModule->isInRecover()){
                wsm->setInOutage(false);
                wsm->setSenderGPSPos(gpsModule->getGpsOutUtmPos());
                wsm->setErrorGPS(gpsModule->getErrorGpsOut());
                wsm->setSenderDRPos(drModule->getLastKnowPosUtm());
                wsm->setErrorDR(drModule->getErrorUtm());

                /*std::cout << "Before Outage: "
                <<" - "<< myId
                <<" - "<< outageModule->getDistOutage()
                <<" - "<< outageModule->getDistRecover()
                <<" - "<< outageModule->getOutagePos()
                <<" - "<< outageModule->getRecoverPos()
                <<" - "<< outageModule->isInOutage()
                <<" - "<< outageModule->isInRecover()
                << "\n\n";*/
            }
            else{
                //em queda
                if(outageModule->isInOutage() && !outageModule->isInRecover()){
                    /*std::cout << "In Outage: "
                    <<" - "<< myId
                    <<" - "<< outageModule->getDistOutage()
                    <<" - "<< outageModule->getDistRecover()
                    <<" - "<< outageModule->getOutagePos()
                    <<" - "<< outageModule->getRecoverPos()
                    <<" - "<< outageModule->isInOutage()
                    <<" - "<< outageModule->isInRecover()
                    << "\n\n";*/
                    wsm->setInOutage(true);
                    drModule->setGeoPos(&lastSUMOGeoPos, &atualSUMOGeoPos);
                    drModule->setErrorLonLat(&atualSUMOGeoPos);
                    projection->setGeoCoord(drModule->getLastKnowPosGeo());
                    projection->FromLonLatToUTM();
                    drModule->setUTMPos(projection->getUtmCoord());
                    drModule->setErrorUTM(&atualSUMOUTMPos);
                    wsm->setSenderDRPos(drModule->getLastKnowPosUtm());
                    wsm->setErrorDR(drModule->getErrorUtm());
                }
                else{
                    /*std::cout << "After Outage: "
                    <<" - "<< myId
                    <<" - "<< outageModule->getDistOutage()
                    <<" - "<< outageModule->getDistRecover()
                    <<" - "<< outageModule->getOutagePos()
                    <<" - "<< outageModule->getRecoverPos()
                    <<" - "<< outageModule->isInOutage()
                    <<" - "<< outageModule->isInRecover()
                    << "\n\n";*/
                    //apos a queda
                    wsm->setInOutage(false);
                    wsm->setSenderGPSPos(gpsModule->getGpsRecUtmPos());
                    wsm->setErrorGPS(gpsModule->getErrorGpsRec());
                    wsm->setSenderDRPos(drModule->getLastKnowPosUtm());
                    wsm->setErrorDR(drModule->getErrorUtm());
                }
            }

            //}
            sendWSM(wsm);
            //Draw annotation
            //findHost()->getDisplayString().updateWith("r=16,blue");
            //annotations->scheduleErase(1,annotations->drawLine(wsm->getSenderPos(), mobility->getCurrentPosition(),"green"));
            //Schedule nest time to  send beacon
            scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
            break;
        }
        default: {
            if (msg)
                DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

void  LocAppCom::onBeacon(WaveShortMessage* wsm){


    //Draw annotation"
    //findHost()->getDisplayString().updateWith("r=16,blue");
    //annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));
    //annotations->scheduleErase(1,annotations->drawLine(wsm->getSenderPos(), mobility->getCurrentPosition(),"blue"));

    //FIXME It's necessary actualize the tracking of the ego vehicle before
    //continue the process same as handleselfmessage.
    Coord currentPos = traci->getTraCIXY(mobility->getCurrentPosition());
    //make the multilateration

    /*std::cout <<"VEHICLE"<< myId << "\n\n";
    std::cout <<"POS 1"<< currentPos << "\n\n";
    std::cout <<"POS 2"<< atualSUMOUTMPos << "\n\n";*/

    AnchorNode anchorNode;
    //If the anchorNode already exists it will be get to be update
    //otherwise the new values will gathered and push to the list
    getAnchorNode(wsm->getSenderAddress(), &anchorNode);
    anchorNode.vehID = wsm->getSenderAddress();
    anchorNode.timestamp = wsm->getTimestamp();
    anchorNode.inOutage = wsm->getInOutage();

    anchorNode.realPos = wsm->getSenderPos();
    anchorNode.realDist = anchorNode.realPos.distance(currentPos);

    //TODO Talvez aplicar o RSSI direto nas distancias DR ou sobre o erro?
    anchorNode.deadReckPos = wsm->getSenderDRPos();
    anchorNode.errorDR = wsm->getErrorDR();
    anchorNode.deadReckDist = anchorNode.deadReckPos.distance(currentPos);

    anchorNode.gpsPos = wsm->getSenderGPSPos();
    anchorNode.errorGPS = wsm->getErrorGPS();
    anchorNode.gpsDist = anchorNode.gpsPos.distance(currentPos);

    //TODO Mecanismo para minimizar o erro ou seja utlizar nós anchoras com erro minimo
    //So utilizar na mutilateração nós ancoras em queda.

    //Calculating RSSI using Real Distances
    fsModel->setRSSI(anchorNode.realDist, this->pTx, this->alpha, this->lambda);
    anchorNode.realRSSIFS = fsModel->getRSSI();
    fsModel->setDistance(anchorNode.realRSSIFS, this->pTx, this->alpha, this->lambda);
    anchorNode.realRSSIDistFS = fsModel->getDistance();

    trgiModel->setRSSI(anchorNode.realDist, this->pTx, this->lambda, this->ht, this->hr, this->epsilonR);
    anchorNode.realRSSITRGI = trgiModel->getRSSI();
    trgiModel->setDistance(anchorNode.realRSSITRGI,anchorNode.realDist,this->pTx,this->lambda, this->ht,this->hr, this->epsilonR);
    anchorNode.realRSSIDistTRGI = trgiModel->getDistance();

    //Calculating RSSI using Dead Reckoning
    fsModel->setRSSI(anchorNode.deadReckDist, this->pTx, this->alpha, this->lambda);
    anchorNode.drRSSIFS = fsModel->getRSSI();
    fsModel->setDistance(anchorNode.drRSSIFS, this->pTx, this->alpha, this->lambda);
    anchorNode.drRSSIDistFS = fsModel->getDistance();

    trgiModel->setRSSI(anchorNode.deadReckDist, this->pTx, this->lambda, this->ht, this->hr, this->epsilonR);
    anchorNode.drRSSITRGI = trgiModel->getRSSI();
    trgiModel->setDistance(anchorNode.drRSSITRGI,anchorNode.deadReckDist,this->pTx,this->lambda, this->ht,this->hr, this->epsilonR);
    anchorNode.drRSSIDistTRGI = trgiModel->getDistance();

    //Avg Filter applied in distance measurements
    //Above Real Dists
    anchorNode.k++;//Increment k_th iteration
    filter->setAverageFilter(anchorNode.k, anchorNode.realRSSIDistAvgFilterFS, anchorNode.realRSSIDistFS);
    anchorNode.realRSSIDistAvgFilterFS = filter->getAvgFilter();
    filter->setAverageFilter(anchorNode.k, anchorNode.realRSSIDistAvgFilterTRGI, anchorNode.realRSSIDistTRGI);
    anchorNode.realRSSIDistAvgFilterTRGI = filter->getAvgFilter();
    //Above DR dists
    filter->setAverageFilter(anchorNode.k, anchorNode.drRSSIDistAvgFilterFS, anchorNode.drRSSIDistFS);
    anchorNode.drRSSIDistAvgFilterFS = filter->getAvgFilter();
    filter->setAverageFilter(anchorNode.k, anchorNode.drRSSIDistAvgFilterTRGI, anchorNode.drRSSIDistTRGI);
    anchorNode.drRSSIDistAvgFilterTRGI = filter->getAvgFilter();

    //Update new values at the list
    //Somente utilizamos a multilateração com a posição dos nós em queda
    //if(anchorNode.inOutage){
    UpdateNeighborList(&anchorNode);
    //}
    //PrintAnchorNode(&anchorNode);

    //FIXME IMplement a mechanism to discard a beacon after some round
    //TODO Timestamp for compute the ttl of the beacon and use it for discard after some time
    //TODO Discard anchor node information with timestamp > than a determined threshold (maybe 100ms)...
    //If there are 4 or more anchor nodes call multilateration method
    if(anchorNodes.size() > 3){
        //TODO Call Multilateration Method
        multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->REAL_DIST);
        coopPosReal = multilateration->getEstPosition();
        coopPosReal.z = mobility->getCurrentPosition().z;

        std::cout <<"atualSUMOPos "<< atualSUMOUTMPos<< endl;
        std::cout <<"curPos "<< currentPos<< endl;
        std::cout <<"CoopPos "<< coopPosReal << endl;

        exit(0);

        multilateration->DoMultilateration(&anchorNodes,multilateration->DR_POS, multilateration->DR_DIST);
        coopPosDR = multilateration->getEstPosition();
        coopPosDR.z = mobility->getCurrentPosition().z;

        multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->FS_DIST);
        coopPosRSSIFS = multilateration->getEstPosition();
        coopPosRSSIFS.z = mobility->getCurrentPosition().z;

        multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->TRGI_DIST);
        coopPosRSSITRGI = multilateration->getEstPosition();
        coopPosRSSITRGI.z = mobility->getCurrentPosition().z;

    }
    else{
        coopPosRSSIFS.x = coopPosRSSIFS.y = coopPosRSSIFS.z = .0;
        coopPosRSSITRGI.x = coopPosRSSITRGI.y = coopPosRSSITRGI.z =  .0;
        coopPosDR.x = coopPosDR.y = coopPosDR.z = .0;
    }

    /****************Log File with results of CP Approach
    **vehID (Neighbor) | Timestamp | MyRealPosition (SUMO) |
    **Neighbor Position (SUMO) |  Real Distance | Est. RSSI Dist FS | RSSI FS |
    **Est. RSSI Dist TRGI | RSSI TRGI | My Estimated Position (Via CP FSpace) | My Estimated Position (Via CP TRGI) |
    * */
    //FIXME ONLY FOR DEBUG OF POSITION AND PROJECTIONS
    //std::pair<double,double> coordTraCI = traci->getTraCIXY(mobility->getCurrentPosition());
    //std::cout << coordTraCI.first << ' '<< coordTraCI.second << endl;
    //std::pair<double,double> lonlat = traci->getLonLat(mobility->getCurrentPosition());
    std::fstream beaconLogFile(std::to_string(myId)+'-'+std::to_string(timeSeed)+".txt", std::fstream::app);
    beaconLogFile << anchorNode.vehID
            <<'\t'<< anchorNode.timestamp
            <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.x
            <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.y
            <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.z
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().x
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().y
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().z
            <<'\t'<< std::setprecision(10) << outageModule->isInOutage()
            <<'\t'<< std::setprecision(10) << anchorNode.inOutage
            <<'\t'<< std::setprecision(10) << anchorNode.realPos.x
            <<'\t'<< std::setprecision(10) << anchorNode.realPos.y
            <<'\t'<< std::setprecision(10) << anchorNode.realPos.z
            <<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.x
            <<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.y
            <<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.z
            <<'\t'<< std::setprecision(10) << anchorNode.realDist
            <<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistFS
            <<'\t'<< std::setprecision(10) << anchorNode.realRSSIFS
            <<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistTRGI
            <<'\t'<< std::setprecision(10) << anchorNode.realRSSITRGI
            <<'\t'<< std::setprecision(10) << coopPosReal.x
            <<'\t'<< std::setprecision(10) << coopPosReal.y
            <<'\t'<< std::setprecision(10) << coopPosReal.z
            <<'\t'<< std::setprecision(10) << coopPosDR.x
            <<'\t'<< std::setprecision(10) << coopPosDR.y
            <<'\t'<< std::setprecision(10) << coopPosDR.z
            <<'\t'<< std::setprecision(10) << coopPosRSSIFS.x
            <<'\t'<< std::setprecision(10) << coopPosRSSIFS.y
            <<'\t'<< std::setprecision(10) << coopPosRSSIFS.z
            <<'\t'<< std::setprecision(10) << coopPosRSSITRGI.x
            <<'\t'<< std::setprecision(10) << coopPosRSSITRGI.y
            <<'\t'<< std::setprecision(10) << coopPosRSSITRGI.z
            << endl;
    beaconLogFile.close();

    //The begin of Cooperative Positioning Approach
}

//Update the position of a neighbor vehicle in the list
void LocAppCom::UpdateNeighborList(AnchorNode *anchorNode){
    //Verify if anchor node already exists...
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        if(it->vehID == anchorNode->vehID){
            it->vehID = anchorNode->vehID;
            it->timestamp = anchorNode->timestamp;
            it->realPos = anchorNode->realPos;
            it->deadReckPos = anchorNode->deadReckPos;
            it->gpsPos = anchorNode->gpsPos;
            it->realDist = anchorNode->realDist;
            it->deadReckDist = anchorNode->deadReckDist;
            it->gpsDist = anchorNode->gpsDist;
            it->errorGPS = anchorNode->errorGPS;
            it->errorDR = anchorNode->errorDR;
            it->realRSSIDistFS = anchorNode->realRSSIDistFS;
            it->realRSSIDistTRGI = anchorNode->realRSSIDistTRGI;
            it->realRSSIFS = anchorNode->realRSSIFS;
            it->realRSSITRGI = anchorNode->realRSSITRGI;
            it->drRSSIDistFS = anchorNode->drRSSIDistFS;
            it->drRSSIDistTRGI = anchorNode->drRSSIDistTRGI;
            it->drRSSIFS = anchorNode->drRSSIFS;
            it->drRSSITRGI = anchorNode->drRSSITRGI;
            it->realRSSIDistAvgFilterFS = anchorNode->realRSSIDistAvgFilterFS;
            it->realRSSIDistAvgFilterTRGI = anchorNode->realRSSIDistAvgFilterTRGI;
            it->drRSSIDistAvgFilterFS = anchorNode->drRSSIDistAvgFilterFS;
            it->drRSSIDistAvgFilterTRGI = anchorNode->drRSSIDistAvgFilterTRGI;
            it->k = anchorNode->k;
            it->inOutage = anchorNode->inOutage;
            return;
        }
    }
    anchorNodes.push_back(*anchorNode);
}


void LocAppCom::PrintAnchorNode(AnchorNode *anchorNode){
    std::cout
    <<'\t'<<anchorNode->vehID
    <<'\t'<<anchorNode->timestamp
    <<'\t'<<anchorNode->realPos
    <<'\t'<<anchorNode->deadReckPos
    <<'\t'<<anchorNode->gpsPos
    <<'\t'<<anchorNode->realDist
    <<'\t'<<anchorNode->deadReckDist
    <<'\t'<<anchorNode->gpsDist
    <<'\t'<<anchorNode->errorGPS
    <<'\t'<<anchorNode->errorDR
    <<'\t'<<anchorNode->realRSSIDistFS
    <<'\t'<<anchorNode->realRSSIDistTRGI
    <<'\t'<<anchorNode->realRSSIFS
    <<'\t'<<anchorNode->realRSSITRGI
    <<'\t'<<anchorNode->drRSSIDistFS
    <<'\t'<<anchorNode->drRSSIDistTRGI
    <<'\t'<<anchorNode->drRSSIFS
    <<'\t'<<anchorNode->drRSSITRGI
    <<'\t'<<anchorNode->realRSSIDistAvgFilterFS
    <<'\t'<<anchorNode->realRSSIDistAvgFilterTRGI
    <<'\t'<<anchorNode->drRSSIDistAvgFilterFS
    <<'\t'<<anchorNode->drRSSIDistAvgFilterTRGI
    <<'\t'<<anchorNode->k
    <<'\t'<<anchorNode->inOutage
    << std::endl;
}


/*
 *Return size of the list of anchor nodes if the id not exists
 *or return the index in the list if the id of an existing anchor node exists
*/
void LocAppCom::getAnchorNode(int id, AnchorNode *anchorNode){
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        if(it->vehID == id){
            anchorNode->vehID = it->vehID;
            anchorNode->timestamp = it->timestamp;
            anchorNode->realPos = it->realPos;
            anchorNode->deadReckPos = it->deadReckPos;
            anchorNode->gpsPos = it->gpsPos;
            anchorNode->realDist = it->realDist;
            anchorNode->deadReckDist = it->deadReckDist;
            anchorNode->gpsDist = it->gpsDist;
            anchorNode->errorGPS = it->errorGPS;
            anchorNode->errorDR = it->errorDR;
            anchorNode->realRSSIDistFS = it->realRSSIDistFS;
            anchorNode->realRSSIDistTRGI = it->realRSSIDistTRGI;
            anchorNode->realRSSIFS = it->realRSSIFS;
            anchorNode->realRSSITRGI = it->realRSSITRGI;
            anchorNode->drRSSIDistFS = it->drRSSIDistFS;
            anchorNode->drRSSIDistTRGI = it->drRSSIDistTRGI;
            anchorNode->drRSSIFS = it->drRSSIFS;
            anchorNode->drRSSITRGI = it->drRSSITRGI;
            anchorNode->realRSSIDistAvgFilterFS = it->realRSSIDistAvgFilterFS;
            anchorNode->realRSSIDistAvgFilterTRGI = it->realRSSIDistAvgFilterTRGI;
            anchorNode->drRSSIDistAvgFilterFS = it->drRSSIDistAvgFilterFS;
            anchorNode->drRSSIDistAvgFilterTRGI = it->drRSSIDistAvgFilterTRGI;
            anchorNode->k = it->k;
            anchorNode->inOutage = it->inOutage;
            break;
        }
    }

}


//We not using data messages in our approach :)
void LocAppCom::onData(WaveShortMessage* wsm){

}


void LocAppCom::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details){
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}




void LocAppCom::finish(){
    BaseWaveApplLayer::finish();

}






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

void LocAppCom::initialize(int stage){
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        Coord coord;//Vehicle's position in SUMO coordinates
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        timeSeed = time(0);

        //Initialize Projections Module...
        //size of (EntranceExit or ExitEntrance ) == 12
        projection = new Projection( traciVehicle->getRouteId().substr( 0,(traciVehicle->getRouteId().size() - 12) ) );

        //Convert from OMNET to TRACI/SUMO
        coord = traci->getTraCIXY(mobility->getCurrentPosition());

        //Initialize Outage Module
        outageModule =  new Outage(traciVehicle->getRouteId());
        //std::cout <<"Outage OUT: "<< outageModule->getOutagePos() << endl;
        //std::cout <<"Outage REC: "<< outageModule->getRecoverPos() << endl;

        //Initializing GPS Module
        gpsModule = new GPS(traciVehicle->getRouteId());
        gpsModule->CompPosition(&coord);

        //std::cout <<"GPS: "<< gpsModule->getPosition() << endl;
        //std::cout <<"GPS: "<< std::setprecision(10)<< gpsModule->getError() << endl;

        //Initializing DR Module
        projection->setUtmCoord(gpsModule->getPosition());
        projection->FromUTMToLonLat();
        drModule = new DeadReckoning(projection->getGeoCoord());

        //Initializing MapMatching Module
        mapMatching = new MapMatching(traciVehicle->getRouteId());
        mapMatching->DoMapMatching(traciVehicle->getRoadId(),gpsModule->getPosition());

        //std::cout <<"MM: "<< mapMatching->getMatchPoint() << endl;
        //std::cout <<"MM: "<< std::setprecision(10) << mapMatching->getDistGpsmm() << endl;

        //Initialize SUMO Positions tracker
        lastSUMOUTMPos = coord;
        atualSUMOUTMPos = lastSUMOUTMPos;
        /*projection->setUtmCoord(lastSUMOUTMPos);
        projection->FromUTMToLonLat();
        lastSUMOGeoPos = projection->getGeoCoord();
        atualSUMOGeoPos = lastSUMOGeoPos;*/

        //Using the route of vehicle to get the information on dataset
        //The name of the route is the same of the equivalent outages dataset.
        /*gpsModule = new GPS(traciVehicle->getRouteId());

        projection->setGeoCoord(gpsModule->getGpsOutGeoPos());
        projection->FromLonLatToUTM();
        gpsModule->setGpsOutUtmPos(projection->getUtmCoord());
        projection->setGeoCoord(gpsModule->getGpsRecGeoPos());
        projection->FromLonLatToUTM();
        gpsModule->setGpsRecUtmPos(projection->getUtmCoord());*/


        /*std::cout << "GPS: "
        <<" - "<< myId
        <<" - "<< gpsModule->getGpsOutUtmPos()
        <<" - "<< std::setprecision(10) << gpsModule->getGpsOutGeoPos().lon <<", " << std::setprecision(10) << gpsModule->getGpsOutGeoPos().lat
        <<" - "<< gpsModule->getGpsRecUtmPos()
        <<" - "<< std::setprecision(10) << gpsModule->getGpsRecGeoPos().lon <<", " << std::setprecision(10) << gpsModule->getGpsRecGeoPos().lat
        <<" - "<< std::setprecision(10) << gpsModule->getErrorGpsOut()
        <<" - "<< std::setprecision(10) << gpsModule->getErrorGpsRec()
        << "\n\n";*/

        //outageModule = new Outage(gpsModule->getGpsOutUtmPos(), gpsModule->getGpsRecUtmPos());

        /*std::cout << "Outage: "
                    <<" - "<< myId
                    <<" - "<< outageModule->getDistOutage()
                    <<" - "<< outageModule->getDistRecover()
                    <<" - "<< outageModule->getOutagePos()
                    <<" - "<< outageModule->getRecoverPos()
                    << "\n\n";*/

        //GDR Module
        /*drModule =  new DeadReckoning(gpsModule->getGpsOutGeoPos());
        projection->setGeoCoord(drModule->getLastKnowPosGeo());
        projection->FromLonLatToUTM();
        drModule->setUTMPos(projection->getUtmCoord());*/

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
    //TODO Verify if last and atual positions is diferents if not fix it and make GDR work
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            //std::cout << myId<<" Send Beacon" << endl;
            WaveShortMessage* wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);

            //Convert from OMNET to TRACI/SUMO
            Coord coord = traci->getTraCIXY(mobility->getCurrentPosition());

            lastSUMOUTMPos = atualSUMOUTMPos;
            atualSUMOUTMPos = coord;

            //std::cout <<myId <<" Last:"<< lastSUMOUTMPos << endl;
            //std::cout <<myId <<" Actual:"<< atualSUMOUTMPos << endl;


            //Real Position
            wsm->setSenderRealPos(atualSUMOUTMPos);
            //Detect if in a outage stage...
            outageModule->ControlOutage(&atualSUMOUTMPos);

            //antes da queda
            if(!outageModule->isInOutage() && !outageModule->isInRecover()){

                //Put in WSM that this vehicle isn't in outage stage
                wsm->setInOutage(false);

                //Compute GPS Position and Error
                gpsModule->CompPosition(&atualSUMOUTMPos);

                wsm->setSenderGPSPos(gpsModule->getPosition());
                wsm->setErrorGPS(gpsModule->getError());

                //Only Update Dead Reckoning Module with last GPS Position
                projection->setUtmCoord(gpsModule->getPosition());
                projection->FromUTMToLonLat();
                drModule->setLastKnowPosGeo(projection->getGeoCoord());
                drModule->setErrorUtm(gpsModule->getError());
                drModule->setErrorGeo(gpsModule->getError());

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
                    //Put in WSM that this vehicle is in outage stage

                    /*if(atualSUMOUTMPos == lastSUMOUTMPos){
                        std::cout <<"Last-Atual: "<< myId << endl;
                        std::cout <<atualSUMOUTMPos << endl;
                        std::cout <<lastSUMOUTMPos << endl;

                    }*/

                    wsm->setInOutage(true);

                    //Convert from UTM to Lat Lon Coordinates from SUMO positions (last and actual) for use in GDR
                    projection->setUtmCoord(lastSUMOUTMPos);
                    projection->FromUTMToLonLat();
                    lastSUMOGeoPos = projection->getGeoCoord();
                    projection->setUtmCoord(atualSUMOUTMPos);
                    projection->FromUTMToLonLat();
                    atualSUMOGeoPos = projection->getGeoCoord();

                    //Compute GDR position.
                    drModule->setGeoPos(&lastSUMOGeoPos, &atualSUMOGeoPos);
                    //Convert from Lon Lat to UTM coordinates
                    projection->setGeoCoord(drModule->getLastKnowPosGeo());
                    projection->FromLonLatToUTM();
                    //Update in UTM Coordinates in DR Module
                    drModule->setUTMPos(projection->getUtmCoord());
                    drModule->setErrorUTMPos(&atualSUMOUTMPos);
                    wsm->setSenderDRPos(drModule->getLastKnowPosUtm());
                    wsm->setErrorDR(drModule->getErrorUtm());

                    //UPDATE GPS error considering last position before outage
                    gpsModule->CompError(&atualSUMOUTMPos);
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
                    //Put in WSM that this vehicle isn't in outage stage anymore
                    wsm->setInOutage(false);
                    gpsModule->CompPosition(&atualSUMOUTMPos);
                    wsm->setSenderGPSPos(gpsModule->getPosition());
                    wsm->setErrorGPS(gpsModule->getError());
                }
            }
            //FIXME Only for debug
            std::cout
            << std::setprecision(10) << lastSUMOUTMPos.x
            <<'\t'<< std::setprecision(10) << lastSUMOUTMPos.y
            <<'\t'<< std::setprecision(10) << lastSUMOUTMPos.z
            <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.x
            <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.y
            <<'\t'<< std::setprecision(10) << atualSUMOUTMPos.z
            <<'\t'<< std::setprecision(10) << gpsModule->getPosition().x
            <<'\t'<< std::setprecision(10) << gpsModule->getPosition().y
            <<'\t'<< std::setprecision(10) << gpsModule->getPosition().z
            <<'\t'<< std::setprecision(10) << gpsModule->getError()
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().x
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().y
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().z
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosGeo().lat
            <<'\t'<< std::setprecision(10) << drModule->getLastKnowPosGeo().lon
            <<'\t'<< std::setprecision(10) << drModule->getAngle()
            <<'\t'<< std::setprecision(10) << std::showpoint << drModule->getArw()
            <<'\t'<< std::setprecision(10) << std::showpoint << drModule->getOffset()
            <<'\t'<< std::setprecision(10) << std::showpoint << drModule->getNonLinearity()
            <<'\t'<< std::setprecision(10) << drModule->getError()
            <<'\t'<< std::setprecision(10) << drModule->getErrorUtm()
            <<'\t'<< std::setprecision(10) << outageModule->isInOutage()
            <<'\t'<< std::setprecision(10) << wsm->getTimestamp()
            << endl;
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
    //Coord currentPos = traci->getTraCIXY(mobility->getCurrentPosition());
    Coord coord = traci->getTraCIXY(mobility->getCurrentPosition());
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

    anchorNode.realPos = wsm->getSenderRealPos();
    anchorNode.realDist = anchorNode.realPos.distance(coord);

    //TODO Talvez aplicar o RSSI direto nas distancias DR ou sobre o erro?
    anchorNode.deadReckPos = wsm->getSenderDRPos();
    anchorNode.errorDR = wsm->getErrorDR();
    anchorNode.deadReckDist = anchorNode.deadReckPos.distance(coord);

    anchorNode.gpsPos = wsm->getSenderGPSPos();
    anchorNode.errorGPS = wsm->getErrorGPS();
    anchorNode.gpsDist = anchorNode.gpsPos.distance(coord);

    //TODO Mecanismo para minimizar o erro ou seja utlizar nós anchoras com erro minimo

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
        std::cout <<"curPos "<< coord << endl;
        std::cout <<"CoopPos "<< coopPosReal << endl;

        //exit(0);

        /*multilateration->DoMultilateration(&anchorNodes,multilateration->DR_POS, multilateration->DR_DIST);
        coopPosDR = multilateration->getEstPosition();
        coopPosDR.z = mobility->getCurrentPosition().z;*/

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
    beaconLogFile
                /*00*/<< anchorNode.vehID
                /*01*/<<'\t'<< anchorNode.timestamp
                /*02*/<<'\t'<< std::setprecision(10) << atualSUMOUTMPos.x
                /*03*/<<'\t'<< std::setprecision(10) << atualSUMOUTMPos.y
                /*04*/<<'\t'<< std::setprecision(10) << atualSUMOUTMPos.z
                /*05*/<<'\t'<< std::setprecision(10) << gpsModule->getPosition().x
                /*06*/<<'\t'<< std::setprecision(10) << gpsModule->getPosition().y
                /*07*/<<'\t'<< std::setprecision(10) << gpsModule->getPosition().z
                /*08*/<<'\t'<< std::setprecision(10) << gpsModule->getError()
                /*09*/<<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().x
                /*10*/<<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().y
                /*11*/<<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().z
                /*12*/<<'\t'<< std::setprecision(10) << drModule->getErrorUtm()
                /*13*/<<'\t'<< std::setprecision(10) << outageModule->isInOutage()
                /*14*/<<'\t'<< std::setprecision(10) << anchorNode.inOutage
                /*15*/<<'\t'<< std::setprecision(10) << anchorNode.realPos.x
                /*16*/<<'\t'<< std::setprecision(10) << anchorNode.realPos.y
                /*17*/<<'\t'<< std::setprecision(10) << anchorNode.realPos.z
                /*18*/<<'\t'<< std::setprecision(10) << anchorNode.gpsPos.x
                /*19*/<<'\t'<< std::setprecision(10) << anchorNode.gpsPos.y
                /*20*/<<'\t'<< std::setprecision(10) << anchorNode.gpsPos.z
                /*21*/<<'\t'<< std::setprecision(10) << anchorNode.errorGPS
                /*22*/<<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.x
                /*23*/<<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.y
                /*24*/<<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.z
                /*25*/<<'\t'<< std::setprecision(10) << anchorNode.errorDR
                /*26*/<<'\t'<< std::setprecision(10) << anchorNode.realDist
                /*27*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistFS
                /*28*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSIFS
                /*29*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistTRGI
                /*30*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSITRGI
                /*31*/<<'\t'<< std::setprecision(10) << coopPosReal.x
                /*32*/<<'\t'<< std::setprecision(10) << coopPosReal.y
                /*33*/<<'\t'<< std::setprecision(10) << coopPosReal.z
                /*34*/<<'\t'<< std::setprecision(10) << coopPosDR.x
                /*35*/<<'\t'<< std::setprecision(10) << coopPosDR.y
                /*36*/<<'\t'<< std::setprecision(10) << coopPosDR.z
                /*37*/<<'\t'<< std::setprecision(10) << coopPosRSSIFS.x
                /*38*/<<'\t'<< std::setprecision(10) << coopPosRSSIFS.y
                /*39*/<<'\t'<< std::setprecision(10) << coopPosRSSIFS.z
                /*40*/<<'\t'<< std::setprecision(10) << coopPosRSSITRGI.x
                /*41*/<<'\t'<< std::setprecision(10) << coopPosRSSITRGI.y
                /*42*/<<'\t'<< std::setprecision(10) << coopPosRSSITRGI.z
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

void LocAppCom::RecognizeEdges(void){
    //FIXME Support Method that uses traciVehicle to recognize edges ids and generate files
    //These files will be used to generate the mechanism of outages and after the MM and GPS error...
    std::cout <<"Current Road:" << endl;
    std::cout << traciVehicle->getRoadId() << endl;
    std::cout << traciVehicle->getRouteId() << endl;
    std::cout <<"road ids:" << endl;
    std::list<std::string> roadIds = traciVehicle->getPlannedRoadIds();
    /*for(std::list<std::string>::iterator it=roadIds.begin(); it!= roadIds.end(); ++it){
        std::cout << *it << endl;
    }*/
    std::string path = "../localization/Graphs/"+traciVehicle->getRouteId()+".txt";
    std::cout << path;
    std::fstream fileEdges(path.c_str(), std::fstream::out);
    if (!fileEdges){
        std::cout << "para o baba" << endl;
        exit(0);
    }
    for(std::list<std::string>::iterator it=roadIds.begin(); it!= roadIds.end(); ++it){
        fileEdges << *it << endl;
    }

    if(myId > 1){
        exit(0);
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
    if(myId == 0){
        exit(0);
    }
    //FIXME Probably when reach the total of outages (last vehicles pass)
    //call some method to finish the simulation
    //FIXME another way repeat outages in dataset up to a determined time

}






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
    //std::cout << "HSM: "<< myId << '\t'<< simTime() << "\n";

    //TODO Verify if last and atual positions is diferents if not fix it and make GDR work
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {

            /*
             * *BEGIN OF UPDATE OF SELF POSITIONING (GPS and DR)
             */

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
                drModule->setLastKnowPosUtm(gpsModule->getPosition());
                drModule->setErrorUtm(gpsModule->getError());
                drModule->setErrorGeo(gpsModule->getError());
            }
            else{
                //em queda
                if(outageModule->isInOutage() && !outageModule->isInRecover()){
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

                    wsm->setSenderGPSPos(gpsModule->getPosition());
                    wsm->setErrorGPS(gpsModule->getError());
                }
                else{
                    //Put in WSM that this vehicle isn't in outage stage anymore
                    wsm->setInOutage(false);
                    gpsModule->CompPosition(&atualSUMOUTMPos);
                    drModule->setLastKnowPosUtm(gpsModule->getPosition());
                    drModule->setErrorUtm(gpsModule->getError());
                    wsm->setSenderGPSPos(gpsModule->getPosition());
                    wsm->setErrorGPS(gpsModule->getError());
                    wsm->setSenderDRPos(gpsModule->getPosition());
                    wsm->setErrorDR(gpsModule->getError());
                }
            }
            sendWSM(wsm);

            /*
             * *END OF UPDATE OF SELF POSITIONING (GPS and DR)
             */

            /*
             * *BEGIN OF UPDATE CP POSITIONING (RSSI)
             */

            DiscardOldBeacons();
            //TODO Begins the Multilateration process
            if(anchorNodes.size() > 3){
                 //TODO Call Multilateration Method
                 double localResidual;
                if(multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->FS_DIST)){
                    coopPosReal = multilateration->getEstPosition();
                    coopPosReal.z = atualSUMOUTMPos.z;
                    errorCPReal = coopPosReal.distance(atualSUMOUTMPos);
                }
                //
                 this->residual = SetResidual();
                 SortByResidual();

                 list<AnchorNode> tempList;
                 //get backup of list
                 tempList = anchorNodes;

                 while(tempList.size() > 3){
                     //remove o primeiro elemento (de maior residual)
                     if(myId==0){
                         std::cout<<"\nAntes\n";
                         PrintNeighborList();
                     }
                     anchorNodes.pop_front();
                     if(myId==0){
                         std::cout<<"\nDepois\n";
                         PrintNeighborList();
                     }
                     //FIXME Verificar isso com calma
                     if(!(multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->FS_DIST)) ){
                         //Apos retirar uma posição o sisema ficou sem solução entao nao altera
                         anchorNodes = tempList; //devolve o beacon que foi retirado
                         break;
                     }

                     localResidual = SetResidual();
                     SortByResidual();
                     if(myId==0){
                      std::cout << "residual" << std::setprecision(10)<< localResidual <<" "<< std::setprecision(10)<<this->residual<< endl;
                      std::cout << "SIZE:" << anchorNodes.size() << endl;
                     }
                     if(localResidual < this->residual){
                         this->residual = localResidual;
                         coopPosReal = multilateration->getEstPosition();
                         coopPosReal.z = atualSUMOUTMPos.z;
                         errorCPReal = coopPosReal.distance(atualSUMOUTMPos);
                         tempList = anchorNodes;
                     }
                     else{
                         if(myId==0){
                             std::cout <<"ENOUGH!\n";
                         }
                         anchorNodes = tempList;
                         break;
                     }

                 }//end while
                 tempList.clear();//
                 //std::cout <<"atualSUMOPos "<< atualSUMOUTMPos<< endl;
                 //std::cout <<"curPos "<< coord << endl;
                 //std::cout <<"CoopPos "<< coopPosReal << endl;
                 //exit(0);

                 /*multilateration->DoMultilateration(&anchorNodes,multilateration->DR_POS, multilateration->DR_DIST);
                 coopPosDR = multilateration->getEstPosition();
                 coopPosDR.z = mobility->getCurrentPosition().z;

                 multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->FS_DIST);
                 coopPosRSSIFS = multilateration->getEstPosition();
                 coopPosRSSIFS.z = mobility->getCurrentPosition().z;

                 multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->TRGI_DIST);
                 coopPosRSSITRGI = multilateration->getEstPosition();
                 coopPosRSSITRGI.z = mobility->getCurrentPosition().z;
                 //TODO THIS is for eliminate all beacons in every multilateration
                 //anchorNodes.clear();
                 if(myId==0){
                     //After Remove Residuals and MULt
                     std::cout<<"BestList:\n";
                     PrintNeighborList();
                 }*/
             }// total of anchor nodes for one fresh multilateration
            /*
             * *END OF UPDATE CP POSITIONING (RSSI)
             */

            //TODO MAKE LOGO FILE
            std::fstream beaconLogFile(std::to_string(myId)+'-'+std::to_string(timeSeed)+".txt", std::fstream::app);
            beaconLogFile
            << std::setprecision(10) << simTime()
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
            <<'\t'<< std::setprecision(10) << drModule->getErrorUtm()
            <<'\t'<< std::setprecision(10) << outageModule->isInOutage()
            <<'\t'<< std::setprecision(10) << coopPosReal.x
            <<'\t'<< std::setprecision(10) << coopPosReal.y
            <<'\t'<< std::setprecision(10) << coopPosReal.z
            <<'\t'<< std::setprecision(10) << errorCPReal
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

            string path = "rv/files/";
            path+= std::to_string(simTime().dbl());
            if(myId==0){
                std::fstream rangeVectorFile(path, std::fstream::out);
                for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
                    rangeVectorFile
                    << atualSUMOUTMPos.x
                    <<"\t"<< atualSUMOUTMPos.y
                    <<"\t"<< atualSUMOUTMPos.z
                    <<"\t"<< atualSUMOUTMPos.distance(it->realPos)
                    <<"\t"<< it->myPosition.x
                    <<"\t"<< it->myPosition.y
                    <<"\t"<< it->myPosition.z
                    <<"\t"<< it->vehID
                    <<'\t'<< it->timestamp
                    <<'\t'<< it->realPos.x
                    <<'\t'<< it->realPos.y
                    <<'\t'<< it->realPos.z
                    <<'\t'<< it->realDist
                    <<'\t'<< it->realRSSIDistTRGI
                    <<'\t'<< coopPosReal.x
                    <<'\t'<< coopPosReal.y
                    <<'\t'<< coopPosReal.z
                    << std::endl;
                }
                rangeVectorFile.close();
            }


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
    //std::cout << "Onb: "<< myId <<'\t'<< simTime() << "\n";
    //Draw annotation"
    //findHost()->getDisplayString().updateWith("r=16,blue");
    //annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));
    //annotations->scheduleErase(1,annotations->drawLine(wsm->getSenderPos(), mobility->getCurrentPosition(),"blue"));

    //FIXME It's necessary update the tracking of the ego vehicle before
    /*Coord coord = traci->getTraCIXY(mobility->getCurrentPosition());
    if((coord.x != atualSUMOUTMPos.x) && (coord.y != atualSUMOUTMPos.y)){
    //UPDATE Ad Hoc Positioning Information
        lastSUMOUTMPos = atualSUMOUTMPos;
        atualSUMOUTMPos = coord;

        outageModule->ControlOutage(&atualSUMOUTMPos);

        //antes da queda
        if(!outageModule->isInOutage() && !outageModule->isInRecover()){

            //Compute GPS Position and Error
            gpsModule->CompPosition(&atualSUMOUTMPos);

            //Only Update Dead Reckoning Module with last GPS Position
            projection->setUtmCoord(gpsModule->getPosition());
            projection->FromUTMToLonLat();
            drModule->setLastKnowPosGeo(projection->getGeoCoord());
            drModule->setErrorUtm(gpsModule->getError());
            drModule->setErrorGeo(gpsModule->getError());
        }
        else{
            //em queda
            if(outageModule->isInOutage() && !outageModule->isInRecover()){
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

                //UPDATE GPS error considering last position before outage
                gpsModule->CompError(&atualSUMOUTMPos);
            }
            else{
                //Put in WSM that this vehicle isn't in outage stage anymore
                gpsModule->CompPosition(&atualSUMOUTMPos);
            }
        }
    }*/

    /*std::cout <<"VEHICLE"<< myId << "\n";
    std::cout <<"POS 1"<< coord << "\n";
    std::cout <<"POS 2"<< atualSUMOUTMPos << "\n\n";*/

    //If the anchorNode already exists it will be get to be update
    //otherwise the new values will gathered and push to the list
    AnchorNode anchorNode;
    getAnchorNode(wsm->getSenderAddress(), &anchorNode);
    anchorNode.vehID = wsm->getSenderAddress();
    anchorNode.timestamp = wsm->getTimestamp();
    anchorNode.inOutage = wsm->getInOutage();

    //FIXME Here the distance need to be calculated with my best estimation
    // This can be CP, DR or GPS position
    // The hipotese is that as the DR will increase the error and up some threshold
    // The CP will be best to use

    anchorNode.myPosition = atualSUMOUTMPos;

    anchorNode.realPos = wsm->getSenderRealPos();
    anchorNode.realDist = anchorNode.realPos.distance(atualSUMOUTMPos);

    anchorNode.deadReckPos = wsm->getSenderDRPos();
    anchorNode.errorDR = wsm->getErrorDR();
    anchorNode.deadReckDist = anchorNode.deadReckPos.distance(atualSUMOUTMPos);

    anchorNode.gpsPos = wsm->getSenderGPSPos();
    anchorNode.errorGPS = wsm->getErrorGPS();
    anchorNode.gpsDist = anchorNode.gpsPos.distance(atualSUMOUTMPos);


    //Calculating RSSI using Real Distances
    fsModel->setRSSI(anchorNode.realDist, this->pTx, this->alpha, this->lambda);
    anchorNode.realRSSIFS = fsModel->getRSSI();
    fsModel->setDistance(anchorNode.realRSSIFS, this->pTx, this->alpha, this->lambda);
    anchorNode.realRSSIDistFS = fsModel->getDistance() + (RNGCONTEXT normal( 0, (fsModel->getDistance()*0.01) ) );

    /*if(myId==0){
        std::cout <<"RSSI DIST FS: " << std::setprecision(10) << fsModel->getDistance() <<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistFS << endl;
    }*/

    /*trgiModel->setRSSI(anchorNode.realDist, this->pTx, this->lambda, this->ht, this->hr, this->epsilonR);
    anchorNode.realRSSITRGI = trgiModel->getRSSI();
    trgiModel->setDistance(anchorNode.realRSSITRGI,anchorNode.realDist,this->pTx,this->lambda, this->ht,this->hr, this->epsilonR);
    anchorNode.realRSSIDistTRGI = trgiModel->getDistance(); //+ (RNGCONTEXT normal( 0,(trgiModel->getDistance()*0.1) ) ); //10% de erro*/


    /*Calculating RSSI using Dead Reckoning
    fsModel->setRSSI(anchorNode.deadReckDist, this->pTx, this->alpha, this->lambda);
    anchorNode.drRSSIFS = fsModel->getRSSI();
    fsModel->setDistance(anchorNode.drRSSIFS, this->pTx, this->alpha, this->lambda);
    anchorNode.drRSSIDistFS = fsModel->getDistance();*/

    /*trgiModel->setRSSI(anchorNode.deadReckDist, this->pTx, this->lambda, this->ht, this->hr, this->epsilonR);
    anchorNode.drRSSITRGI = trgiModel->getRSSI();
    trgiModel->setDistance(anchorNode.drRSSITRGI,anchorNode.deadReckDist,this->pTx,this->lambda, this->ht,this->hr, this->epsilonR);
    anchorNode.drRSSIDistTRGI = trgiModel->getDistance();*/

    //FIXME Avg Filter applied in distance measurements
    //Above Real Dists
    /*anchorNode.k++;//Increment k_th iteration
    filter->setAverageFilter(anchorNode.k, anchorNode.realRSSIDistAvgFilterFS, anchorNode.realRSSIDistFS);
    anchorNode.realRSSIDistAvgFilterFS = filter->getAvgFilter();
    filter->setAverageFilter(anchorNode.k, anchorNode.realRSSIDistAvgFilterTRGI, anchorNode.realRSSIDistTRGI);
    anchorNode.realRSSIDistAvgFilterTRGI = filter->getAvgFilter();
    //Above DR dists
    filter->setAverageFilter(anchorNode.k, anchorNode.drRSSIDistAvgFilterFS, anchorNode.drRSSIDistFS);
    anchorNode.drRSSIDistAvgFilterFS = filter->getAvgFilter();
    filter->setAverageFilter(anchorNode.k, anchorNode.drRSSIDistAvgFilterTRGI, anchorNode.drRSSIDistTRGI);
    anchorNode.drRSSIDistAvgFilterTRGI = filter->getAvgFilter();*/

    //Verify if it is a good beacon
    if(IsAGoodBeacon(&anchorNode)){
        //Update new values at the list
        UpdateNeighborList(&anchorNode);
        //PutInNeighborList(&anchorNode);
    }

    //FIXME IMplement a mechanism to discard a beacon after some round
    //TODO Timestamp for compute the ttl of the beacon and use it for discard after some time
    //TODO Discard anchor node information with timestamp > than a determined threshold (maybe 100ms)...
    //If there are 4 or more anchor nodes call multilateration method

   /*if(anchorNodes.size() > 3){
        //TODO Call Multilateration Method
        double localResidual;
        multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->REAL_DIST);
        coopPosReal = multilateration->getEstPosition();
        coopPosReal.z = atualSUMOUTMPos.z;
        errorCPReal = coopPosReal.distance(atualSUMOUTMPos);

        this->residual = SetResidual();
        SortByResidual();

        list<AnchorNode> tempList;
        //get backup of list
        tempList = anchorNodes;

        while(tempList.size() > 3){
            //remove o primeiro elemento (de maior residual)
            if(myId==0){
                std::cout<<"\nAntes\n";
                PrintNeighborList();
            }
            anchorNodes.pop_front();
            if(myId==0){
                std::cout<<"\nDepois\n";
                PrintNeighborList();
            }
            multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->REAL_DIST);
            localResidual = SetResidual();
            SortByResidual();
            if(localResidual < this->residual){
                std::cout << "removing residual\n";
                this->residual = localResidual;
                coopPosReal = multilateration->getEstPosition();
                coopPosReal.z = atualSUMOUTMPos.z;
                errorCPReal = coopPosReal.distance(atualSUMOUTMPos);
                tempList = anchorNodes;
            }
            else{
                std::cout <<"ENOUGH!\n";
                anchorNodes = tempList;
                break;
            }

        }//end while
        //std::cout <<"atualSUMOPos "<< atualSUMOUTMPos<< endl;
        //std::cout <<"curPos "<< coord << endl;
        //std::cout <<"CoopPos "<< coopPosReal << endl;
        //exit(0);

        multilateration->DoMultilateration(&anchorNodes,multilateration->DR_POS, multilateration->DR_DIST);
        coopPosDR = multilateration->getEstPosition();
        coopPosDR.z = mobility->getCurrentPosition().z;

        multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->FS_DIST);
        coopPosRSSIFS = multilateration->getEstPosition();
        coopPosRSSIFS.z = mobility->getCurrentPosition().z;

        multilateration->DoMultilateration(&anchorNodes,multilateration->REAL_POS, multilateration->TRGI_DIST);
        coopPosRSSITRGI = multilateration->getEstPosition();
        coopPosRSSITRGI.z = mobility->getCurrentPosition().z;
        //TODO THIS is for eliminate all beacons in every multilateration
        //anchorNodes.clear();
        if(myId==0){
            //After Remove Residuals and MULt
            std::cout<<"BestList:\n";
            PrintNeighborList();
        }
    }// total of anchor nodes for one fresh multilateration
    else{
        coopPosRSSIFS.x = coopPosRSSIFS.y = coopPosRSSIFS.z = .0;
        coopPosRSSITRGI.x = coopPosRSSITRGI.y = coopPosRSSITRGI.z =  .0;
        coopPosDR.x = coopPosDR.y = coopPosDR.z = .0;
    }*/

    /****************Log File with results of CP Approach
    **vehID (Neighbor) | Timestamp | MyRealPosition (SUMO) |
    **Neighbor Position (SUMO) |  Real Distance | Est. RSSI Dist FS | RSSI FS |
    **Est. RSSI Dist TRGI | RSSI TRGI | My Estimated Position (Via CP FSpace) | My Estimated Position (Via CP TRGI) |
    * */
    //FIXME ONLY FOR DEBUG OF POSITION AND PROJECTIONS
    //std::pair<double,double> coordTraCI = traci->getTraCIXY(mobility->getCurrentPosition());
    //std::cout << coordTraCI.first << ' '<< coordTraCI.second << endl;
    //std::pair<double,double> lonlat = traci->getLonLat(mobility->getCurrentPosition());
    //std::fstream beaconLogFile(std::to_string(myId)+'-'+std::to_string(timeSeed)+".txt", std::fstream::app);
    /*if ( beaconLogFile.peek() == std::ifstream::traits_type::eof() )
    {
       //Put Header
    }*/
//    beaconLogFile
//                << std::setprecision(10) << simTime()
//                /*00*/<<'\t'<< anchorNode.vehID
//                /*01*/<<'\t'<< anchorNode.timestamp
//                /*02*/<<'\t'<< std::setprecision(10) << atualSUMOUTMPos.x
//                /*03*/<<'\t'<< std::setprecision(10) << atualSUMOUTMPos.y
//                /*04*/<<'\t'<< std::setprecision(10) << atualSUMOUTMPos.z
//                /*05*/<<'\t'<< std::setprecision(10) << gpsModule->getPosition().x
//                /*06*/<<'\t'<< std::setprecision(10) << gpsModule->getPosition().y
//                /*07*/<<'\t'<< std::setprecision(10) << gpsModule->getPosition().z
//                /*08*/<<'\t'<< std::setprecision(10) << gpsModule->getError()
//                /*09*/<<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().x
//                /*10*/<<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().y
//                /*11*/<<'\t'<< std::setprecision(10) << drModule->getLastKnowPosUtm().z
//                /*12*/<<'\t'<< std::setprecision(10) << drModule->getErrorUtm()
//                /*13*/<<'\t'<< std::setprecision(10) << outageModule->isInOutage()
//                /*14*/<<'\t'<< std::setprecision(10) << anchorNode.inOutage
//                /*15*/<<'\t'<< std::setprecision(10) << anchorNode.realPos.x
//                /*16*/<<'\t'<< std::setprecision(10) << anchorNode.realPos.y
//                /*17*/<<'\t'<< std::setprecision(10) << anchorNode.realPos.z
//                /*18*/<<'\t'<< std::setprecision(10) << anchorNode.gpsPos.x
//                /*19*/<<'\t'<< std::setprecision(10) << anchorNode.gpsPos.y
//                /*20*/<<'\t'<< std::setprecision(10) << anchorNode.gpsPos.z
//                /*21*/<<'\t'<< std::setprecision(10) << anchorNode.errorGPS
//                /*22*/<<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.x
//                /*23*/<<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.y
//                /*24*/<<'\t'<< std::setprecision(10) << anchorNode.deadReckPos.z
//                /*25*/<<'\t'<< std::setprecision(10) << anchorNode.errorDR
//                /*26*/<<'\t'<< std::setprecision(10) << anchorNode.realDist
//                /*27*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistFS
//                /*28*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSIFS
//                /*29*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSIDistTRGI
//                /*30*/<<'\t'<< std::setprecision(10) << anchorNode.realRSSITRGI
//                /*31*/<<'\t'<< std::setprecision(10) << coopPosReal.x
//                /*32*/<<'\t'<< std::setprecision(10) << coopPosReal.y
//                /*33*/<<'\t'<< std::setprecision(10) << coopPosReal.z
//                /*33*/<<'\t'<< std::setprecision(10) << errorCPReal
//                /*34*/<<'\t'<< std::setprecision(10) << coopPosDR.x
//                /*35*/<<'\t'<< std::setprecision(10) << coopPosDR.y
//                /*36*/<<'\t'<< std::setprecision(10) << coopPosDR.z
//                /*37*/<<'\t'<< std::setprecision(10) << coopPosRSSIFS.x
//                /*38*/<<'\t'<< std::setprecision(10) << coopPosRSSIFS.y
//                /*39*/<<'\t'<< std::setprecision(10) << coopPosRSSIFS.z
//                /*40*/<<'\t'<< std::setprecision(10) << coopPosRSSITRGI.x
//                /*41*/<<'\t'<< std::setprecision(10) << coopPosRSSITRGI.y
//                /*42*/<<'\t'<< std::setprecision(10) << coopPosRSSITRGI.z
//            << endl;
//    beaconLogFile.close();

    //The begin of Cooperative Positioning Approach
}


//Update the position of a neighbor vehicle in the as a new beacon is recognized
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


//Update the position of a neighbor vehicle in the as a new beacon is recognized
void LocAppCom::PutInNeighborList(AnchorNode *anchorNode){
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
    <<'\t'<<anchorNode->residual
    << std::endl;
}


void LocAppCom::PrintNeighborList(){
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        std::cout
            << it->vehID
            <<'\t'<<it->timestamp
            <<'\t'<<it->realPos
            <<'\t'<<it->deadReckPos
            <<'\t'<<it->gpsPos
            <<'\t'<<it->realDist
            <<'\t'<<it->deadReckDist
            <<'\t'<<it->gpsDist
            <<'\t'<<it->errorGPS
            <<'\t'<<it->errorDR
            <<'\t'<<it->realRSSIDistFS
            <<'\t'<<it->realRSSIDistTRGI
            <<'\t'<<it->realRSSIFS
            <<'\t'<<it->realRSSITRGI
            <<'\t'<<it->drRSSIDistFS
            <<'\t'<<it->drRSSIDistTRGI
            <<'\t'<<it->drRSSIFS
            <<'\t'<<it->drRSSITRGI
            <<'\t'<<it->realRSSIDistAvgFilterFS
            <<'\t'<<it->realRSSIDistAvgFilterTRGI
            <<'\t'<<it->drRSSIDistAvgFilterFS
            <<'\t'<<it->drRSSIDistAvgFilterTRGI
            <<'\t'<<it->k
            <<'\t'<<it->inOutage
            <<'\t'<<it->residual
            << std::endl;
    }
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


/*void LocAppCom::UpdateRange(){
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        it->realDist = it->realPos.distance(atualSUMOUTMPos);
    }
}*/

/*
 *Discard Beacons with with delta(timestamp, actualtime) > 0.5 seconds
 *Discard beacons with frequency less than 2 Hz
 *
*/
void LocAppCom::DiscardOldBeacons(){
    simtime_t delta;
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        delta = simTime() - it->timestamp;
        if(delta >= 0.5){
            it = anchorNodes.erase(it);
            std::cout << it->vehID <<" beacon erased\n\n";
        }
    }
}

//TODO Thanks again Bourke: http://paulbourke.net/geometry/circlesphere/
//This method eliminate geometric issues in beacons, like one pre filter
bool LocAppCom::IsAGoodBeacon(AnchorNode * anchorNode){
    double distance;
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){

        distance = it->realPos.distance(anchorNode->realPos);

        if(distance > (it->realRSSIDistFS + anchorNode->realRSSIDistFS) ){
            //The two circles not intersect
            return false;
        }//end if
        else{
            if(distance < fabs(it->realRSSIDistFS - anchorNode->realRSSIDistFS)){
                //one circle is contained within the other
                return false;
            }//endif
            else{
                if( (distance == 0) && (it->realRSSIDistFS == anchorNode->realRSSIDistFS) ){
                    //circles are coincident and there are an infinite number of solutions
                    //FIXME Maybe in this case is good use the most "fresh" beacon
                    return false;
                }//end if
            }//end else
        }//end if

    }//end for

    return true;
}

bool sortReverseOrder(const AnchorNode & a, const AnchorNode & b) { return a.residual > b.residual; }

void LocAppCom::SortByResidual(){
    anchorNodes.sort(sortReverseOrder);
}


double LocAppCom::SetResidual(){
    double distance, residual;
    residual = 0;
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        //distance from the estimated point by multilateration to the position of an acnhor node (i)
        distance = multilateration->getEstPosition().distance(it->realPos);
        //residual is the difference between the above distance and the rssi distance
        it->residual = (distance - it->realRSSIDistFS) * (distance - it->realRSSIDistFS);
        residual+= it->residual;
    }
    if(myId==0){
        std::cout<<"Set Residual:"<< std::setprecision(10)<< residual<< "\n";
    }

    return residual;
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
    if (!fileEdges.is_open()){
        std::cout << "Error.." << endl;
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
    /*if(myId == 0){
        exit(0);
    }*/
    //FIXME Probably when reach the total of outages (last vehicles pass)
    //WARNING Dont use exit(0) because the scalars not writed
    //call some method to finish the simulation
    //FIXME another way repeat outages in dataset up to a determined time

}






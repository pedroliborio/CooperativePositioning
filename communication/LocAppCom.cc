#include "LocAppCom.h"

Define_Module(LocAppCom);

const simsignalwrap_t LocAppCom::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t LocAppCom::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

void LocAppCom::initialize(int stage) {
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

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
                getParentModule());
        assert(mac);

        myId = getParentModule()->getId();



        //read parameters
        headerLength = par("headerLength").longValue();
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits").longValue();
        beaconPriority = par("beaconPriority").longValue();
        beaconInterval =  par("beaconInterval").doubleValue();

        forwardBeacons = par("forwardBeacons").boolValue();
        forwardInterval = par("forwardInterval").doubleValue();
        numHops = par("numHops").longValue();

        logSeedPar = par("logSeedPar");
        logDistVehPar = par("logDistVehPar");


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
        //for RMSE statistcs
        rmseGPS = .0;
        rmseDRCP = .0;
        numRMSEs = .0;

        /*
         * INITIALIZING LOCALIZATION MODULES...
         */
        InitLocModules();


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

simtime_t LocAppCom::computeAsynchronousSendingTime(simtime_t interval, t_channel chan) {

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); //usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earlierst in next CCH  (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval*2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() %switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    //check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw()  % (2*switchingInterval.raw()) > switchingInterval.raw()) {
        //firstEvent is within a sch interval
        if (chan == type_CCH) firstEvent -= switchingInterval;
    }
    else {
        //firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == type_SCH) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void LocAppCom::populateWSM(WaveShortMessage* wsm, int rcvId, int serial) {

    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(rcvId);
    wsm->setSerial(serial);
    wsm->setBitLength(headerLength);


    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm) ) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(Channels::CCH);
        bsm->addBitLength(beaconLengthBits);
        wsm->setPriority(beaconPriority);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(Channels::CCH);
        wsa->setTargetChannel(currentServiceChannel);
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else {
        if (dataOnSch) wsm->setChannelNumber(Channels::SCH1); //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else wsm->setChannelNumber(Channels::CCH);
        wsm->addBitLength(dataLengthBits);
        wsm->setPriority(dataPriority);
    }
}

void LocAppCom::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void LocAppCom::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getCurrentPosition();
    curSpeed = mobility->getCurrentSpeed();
}

void LocAppCom::handleParkingUpdate(cObject* obj) {
    //this code should only run when used with TraCI
    isParked = mobility->getParkingState();
    if (communicateWhileParked == false) {
        if (isParked == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
        }
        else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

void LocAppCom::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete(msg);
}

void LocAppCom::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        BasicSafetyMessage* bsm = new BasicSafetyMessage();

        //Put All kinematics GPS, DR, CP information on beacon
        PutBeaconInformation(bsm);

        //Discard old Beacon from the neighbors list
        DiscardOldBeacons();

        //Update CP Positioning
        UpdateCooperativePositioning();

        //Improve Dead Reckoning with CP Positioning
        ImproveDeadReckoning();

        //Update RMSE Statistics GPS e GPS+DR+CP
        RMSEStatistics();

        //Write Log Files
        //WriteLogFiles();

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
            //std::cout << "Evento de forwarding" << endl;
            //Verificar se tem beacons com TTL expirados e deletar.
            DeleteOldBeaconToForward();
            //Verificar se tem beacon na lista e fazer forwarding e depois deletar.
            if(!listFWDBeacons.empty()){
                //envia um beacon e o retira da lista de beacons para envio
                sendDown(listFWDBeacons.front());
                //std::cout << "forwarding beacon: " << listFWDBeacons.front()->getPersistentID() << endl;
                listFWDBeacons.pop_front();
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


void LocAppCom::onBSM(BasicSafetyMessage* bsm) {

    if ( BeaconIsDuplicated(bsm) || BeaconHaveMyId(bsm) || (!BeaconIsAlive(bsm)) || bsm->getInOutage() ){
        //std::cout <<"do nothing..." << endl;
        //delete(bsm);
        return;
    }

    //Verify if it is a RSU beacon and transform to traCI/SUMO coordinates
    if(bsm->getIsRSUBSM()){
        bsm->setSenderRealPos( traci->getTraCIXY( bsm->getSenderPos() ) );
    }


    //std::cout << myId << '\t' << bsm->getRssi() << '\t' << bsm->getSenderAddress() << endl;


    //std::cout << "My Id: "<< myId << " Sender Beacon:" << bsm->getSenderAddress() << "Beacon ID:" << bsm->getId()<< endl;

    AnchorNode anchorNode;
    getAnchorNode(bsm->getSenderAddress(), &anchorNode);
    anchorNode.vehID = bsm->getSenderAddress();
    anchorNode.timestamp = bsm->getTimestamp();
    anchorNode.inOutage = bsm->getInOutage();

    //FIXME Here the distance need to be calculated with my best estimation
    // This can be CP, DR or GPS position
    // The hypothesis is that as the DR will increase the error, and up some threshold,
    // The CP will be best to use
    anchorNode.myPosition = atualSUMOUTMPos;

    anchorNode.realPos = bsm->getSenderRealPos();
    anchorNode.realDist = anchorNode.realPos.distance(atualSUMOUTMPos);

    anchorNode.deadReckPos = bsm->getSenderDRPos();
    anchorNode.errorDR = bsm->getErrorDR();
    anchorNode.deadReckDist = anchorNode.deadReckPos.distance(atualSUMOUTMPos);

    anchorNode.gpsPos = bsm->getSenderGPSPos();
    anchorNode.errorGPS = bsm->getErrorGPS();
    anchorNode.gpsDist = anchorNode.gpsPos.distance(atualSUMOUTMPos);

    //Calculating RSSI using Real Distances
    fsModel->setRSSI(anchorNode.realDist, this->pTx, this->alpha, this->lambda);
    anchorNode.realRSSIFS = fsModel->getRSSI();
    fsModel->setDistance(anchorNode.realRSSIFS, this->pTx, this->alpha, this->lambda);
    anchorNode.realRSSIDistFS = fsModel->getDistance() + (RNGCONTEXT normal( 0, (fsModel->getDistance()*0.1) ) );

    anchorNode.hops = bsm->getHops();


    UpdateNeighborList(&anchorNode);


    /*See: https://omnetpp.org/doc/omnetpp/manual/#sec:simple-modules:self-messages
     * Creating and sending a copy...
     * // (re)transmit packet:
    cMessage *copy = packet->dup();
    send(copy, "out");
    and finally (when no more retransmissions will occur):

    delete packet;*/
    //if(MULTIHOP){
        //Retransmission (Multihop)
        //Message only can be restranmited if timestamp is below of one trheshold
        //works like a TTL
        //if( ){
   if(forwardBeacons){
       BasicSafetyMessage *fwdBSM = bsm->dup();
       fwdBSM->setPersistentID(bsm->getPersistentID());
       fwdBSM->setHops(bsm->getHops()+1);
       //std::cout << bsm->getPersistentID() << "<- BSM, FWDBSM ->"<< fwdBSM->getPersistentID() << endl;

       //Adiciona o beacon para lista de beacons para encaminhamento...
       AddBeaconToForward(fwdBSM);
   }
   //fwdBSM->setHops(bsm->getHops()+1);
   //sendDown(fwdBSM);
            //sendDelayedDown(fwdBSM,forwardingInterval);
            //delete(bsm);
        //}
   // }
}

void LocAppCom::onWSM(WaveShortMessage* wsm) {}
void LocAppCom::onWSA(WaveServiceAdvertisment* wsa) {}



void LocAppCom::finish() {
    recordScalar("generatedWSMs",generatedWSMs);
    recordScalar("receivedWSMs",receivedWSMs);

    recordScalar("generatedBSMs",generatedBSMs);
    recordScalar("receivedBSMs",receivedBSMs);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);

    //Compute and record RMSE statistics...
    ComputeLocStats();
}

LocAppCom::~LocAppCom() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(sendFWDBeaconEvt);
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

void LocAppCom::startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription) {
    if (sendWSAEvt->isScheduled()) {
        error("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, type_CCH);
    scheduleAt(wsaTime, sendWSAEvt);

}

void LocAppCom::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void LocAppCom::sendDown(cMessage* msg) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void LocAppCom::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void LocAppCom::checkAndTrackPacket(cMessage* msg) {
    if (isParked && !communicateWhileParked) error("Attempted to transmit a message while parked, but this is forbidden by current configuration");

    if (dynamic_cast<BasicSafetyMessage*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
        DBG_APP << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}

/*
 * All Modules that I've implemented...
 */

void LocAppCom::InitLocModules(){

    timeSeed = time(0);

    /*if(traciVehicle == NULL){
        std::cout << "Why?" << endl;
        exit(0);
    }*/
    OutCoord outCoord;
    Coord coord;
    //Initialize Projections Module...
    //size of (EntranceExit or ExitEntrance ) == 12
    projection = new Projection( traciVehicle->getRouteId().substr( 0,(traciVehicle->getRouteId().size() - 12) ) );

    //Convert from OMNET to TRACI/SUMO
    coord = traci->getTraCIXY(mobility->getCurrentPosition());

    //Initialize Outage Module

    //Get one outage from the server of outages

    outCoord = check_and_cast<Outages*>(getSimulation()->getModuleByPath("outagesServer.appl"))->getOutage(traciVehicle->getRouteId());

    outageModule = new Outage();

    outageModule->setOutagePos(outCoord.outage);
    outageModule->setRecoverPos(outCoord.recover);

   /*std::cout << outageModule->getOutagePos() << endl;
    std::cout << outageModule->getRecoverPos() << endl;
    std::cout << outageModule->isInOutage() << endl;
    std::cout << outageModule->isInRecover() << endl;*/


    //Criar um objeto dataset que carrega o dataset e sorteia uma entrada de forma randomica
    //e passa os valores pra classe outages...
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
    drModule = new DeadReckoning(projection->getGeoCoord(),beaconInterval.dbl());

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

    errorCPReal = 0;
}


void LocAppCom::PutBeaconInformation(BasicSafetyMessage* bsm){
    /*BEGIN OF UPDATE SELF POSITIONING (GPS and DR)*/

    //Put unique persistent ID for FORWARDING purposes
    bsm->setPersistentID(bsm->getId());

    //Initialize Hops counter with zero
    bsm->setHops(0);

    //This is a vehicle beacon
    bsm->setIsRSUBSM(false);

    //Convert from OMNET to TRACI/SUMO
    Coord coord = traci->getTraCIXY(mobility->getCurrentPosition());

    lastSUMOUTMPos = atualSUMOUTMPos;
    atualSUMOUTMPos = coord;
    //Real Position
    bsm->setSenderRealPos(atualSUMOUTMPos);
    //Detect if in a outage stage...
    outageModule->ControlOutage(&atualSUMOUTMPos);

    /*if(myId ==25){
        std::cout << outageModule->getOutagePos() << '\t' << outageModule->getRecoverPos() <<'\t'<<outageModule->isInOutage() <<'\t'<< outageModule->isInRecover() <<endl;
    }*/

    //std::cout << myId <<'\t'<< outageModule->isInOutage() <<'\t'<< outageModule->isInRecover() << endl;

    //antes da queda
    if(!outageModule->isInOutage() && !outageModule->isInRecover()){
        //Put in WSM that this vehicle isn't in outage stage
       bsm->setInOutage(false);

        //Compute GPS Position and Error
        gpsModule->CompPosition(&atualSUMOUTMPos);

        bsm->setSenderGPSPos(gpsModule->getPosition());
        bsm->setErrorGPS(gpsModule->getError());

        //Only Update Dead Reckoning Module with last GPS Position
        projection->setUtmCoord(gpsModule->getPosition());
        projection->FromUTMToLonLat();
        drModule->setLastKnowPosGeo(projection->getGeoCoord());
        drModule->setLastKnowPosUtm(gpsModule->getPosition());
        drModule->setErrorUtm(gpsModule->getError());
        drModule->setErrorGeo(gpsModule->getError());

        //collecting stats for teh time of outage...
        timestampOutage = simTime();
    }
    else{
        //em queda
        if(outageModule->isInOutage() && !outageModule->isInRecover()){
            bsm->setInOutage(true);

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
            bsm->setSenderDRPos(drModule->getLastKnowPosUtm());
            bsm->setErrorDR(drModule->getErrorUtm());

            //UPDATE GPS error considering last position before outage
            gpsModule->CompError(&atualSUMOUTMPos);

            bsm->setSenderGPSPos(gpsModule->getPosition());
            bsm->setErrorGPS(gpsModule->getError());

            //collecting stats about outage
            timestampRecover = simTime();
        }
        else{
            //Put in WSM that this vehicle isn't in outage stage anymore
            bsm->setInOutage(false);
            gpsModule->CompPosition(&atualSUMOUTMPos);
            drModule->setLastKnowPosUtm(gpsModule->getPosition());
            drModule->setErrorUtm(gpsModule->getError());
            bsm->setSenderGPSPos(gpsModule->getPosition());
            bsm->setErrorGPS(gpsModule->getError());
            bsm->setSenderDRPos(gpsModule->getPosition());
            bsm->setErrorDR(gpsModule->getError());
        }
    }
    /*
    * *END OF UPDATE OF SELF POSITIONING (GPS and DR)
    */
}

void LocAppCom::UpdateCooperativePositioning(){

    if(anchorNodes.size() > 3) {
         //TODO Call Multilateration Method
         double localResidual;
        if(multilateration->DoMultilateration(&anchorNodes,multilateration->GPS_POS, multilateration->FS_DIST)){
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
                 //std::cout<<"\nAntes\n";
                 //PrintNeighborList();
             }
             anchorNodes.pop_front();
             if(myId==0){
                 //std::cout<<"\nDepois\n";
                 //PrintNeighborList();
             }
             //FIXME Verificar isso com calma
             if(!(multilateration->DoMultilateration(&anchorNodes,multilateration->GPS_POS, multilateration->FS_DIST)) ){
                 //Apos retirar uma posição o sisema ficou sem solução entao nao altera
                 anchorNodes = tempList; //devolve o beacon que foi retirado
                 break;
             }

             localResidual = SetResidual();
             SortByResidual();

             if(myId==0){
              //std::cout << "residual" << std::setprecision(10)<< localResidual <<" "<< std::setprecision(10)<<this->residual<< endl;
              //std::cout << "SIZE:" << anchorNodes.size() << endl;
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
                     //std::cout <<"ENOUGH!\n";
                 }
                 anchorNodes = tempList;
                 break;
             }

         }//end while
         tempList.clear();
    }// total of anchor nodes for one fresh multilateration
}

void LocAppCom::ImproveDeadReckoning(){
    //************************
    //**************FIXME First Approach to improve DR
    if( (outageModule->isInOutage() && !outageModule->isInRecover()) && (errorCPReal < drModule->getErrorUtm()) && errorCPReal > 1){
       //(errorCPReal < 20.0 && errorCPReal > 1) && )  ){
        drModule->ReinitializeSensors();
        //Update DR with CP position
        drModule->setLastKnowPosUtm(coopPosReal);
        drModule->setErrorUtm(errorCPReal);
        //Update Geo and UTM coordinates in DR Module
        projection->setUtmCoord(drModule->getLastKnowPosUtm());
        projection->FromUTMToLonLat();
        drModule->setLastKnowPosGeo(projection->getGeoCoord());
        drModule->setErrorGeo(errorCPReal);
    }
}

void LocAppCom::WriteLogFiles(){
    std::fstream beaconLogFile(traciVehicle->getRouteId().substr( 0,(traciVehicle->getRouteId().size() - 12) )+"/"+std::to_string(logSeedPar)+'-'+std::to_string(logDistVehPar)+'-'+std::to_string(numHops)+'-'+std::to_string(myId)+'-'+traciVehicle->getRouteId()+".txt", std::fstream::app);
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
    <<'\t'<< std::setprecision(10) << drModule->getAngle()
    <<'\t'<< std::setprecision(10) << drModule->getArw()
    <<'\t'<< std::setprecision(10) << drModule->getSensitivity()
    <<'\t'<< std::setprecision(10) << drModule->getError()
    <<'\t'<< std::setprecision(10) << drModule->getLPFTheta().getLpf()
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
    <<'\t'<< std::setprecision(10) << mobility->getSpeed()

    << endl;
    beaconLogFile.close();
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


//Update the position of a neighbor vehicle as soon as a new beacon is recognized
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
    //simtime_t delta;
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        //delta = simTime() - it->timestamp;
        /*if(delta >= 0.5){
            it = anchorNodes.erase(it);
            //std::cout << it->vehID <<" beacon erased\n\n";
        }*/
        if (it->hops > numHops){
            it = anchorNodes.erase(it);
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
        //std::cout<<"Set Residual:"<< std::setprecision(10)<< residual<< "\n";
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


//it = anchorNodes.erase(it);
//std::cout << it->vehID <<" beacon erased\n\n";


bool LocAppCom::BeaconIsDuplicated(BasicSafetyMessage* bsm){
    //each node only retransmit exactly one time using the persistent id
    for(std::list<BasicSafetyMessage*>::iterator it=listFWDBeacons.begin(); it!= listFWDBeacons.end(); ++it){
        if((*it)->getPersistentID() == bsm->getPersistentID()){
            //std::cout <<"beacon duplicated:"<<" list: "<< (*it)->getPersistentID() << " beacon: " << bsm->getPersistentID() << endl;
            return true;
        }
    }
    return false;
}

bool LocAppCom::BeaconHaveMyId(BasicSafetyMessage* bsm){
    if(bsm->getSenderAddress() == myId){
        return true;
    }
    return false;
}

bool LocAppCom::BeaconIsAlive(BasicSafetyMessage* bsm){
    if( bsm->getHops() <= numHops){
        return true;
    }
    return false;
}

void LocAppCom::AddBeaconToForward(BasicSafetyMessage* fwdBSM){
    //std::cout << myId <<" putting beacon on list: " << fwdBSM->getPersistentID() <<' '<< fwdBSM->getSenderAddress()  <<' '<< fwdBSM->getTimestamp()<< endl;
    listFWDBeacons.push_back(fwdBSM);
}

void LocAppCom::RMSEStatistics(){
    rmseGPS+= gpsModule->getError();
    rmseDRCP+= drModule->getErrorUtm();
    numRMSEs++;
}

void LocAppCom::ComputeLocStats(){
    recordScalar("rmseGPS",rmseGPS/numRMSEs);
    recordScalar("rmseDRCP",rmseDRCP/numRMSEs);
    recordScalar("timeOutage",(timestampRecover-timestampOutage).dbl());
}

//for a while we dont need to delete old beacons
void LocAppCom::DeleteOldBeaconToForward(){
    for(std::list<BasicSafetyMessage*>::iterator it=listFWDBeacons.begin(); it!= listFWDBeacons.end();){

        /*if( (simTime() - (*it)->getTimestamp()) > 0.5){
          //      std::cout <<" deleting old beacon"<< (*it)->getPersistentID() <<endl;
            delete *it;
            it = listFWDBeacons.erase(it);
        }
        else{
            ++it;
        }*/
        if( (*it)->getHops() > numHops){
            delete *it;
            it = listFWDBeacons.erase(it);
        }
        else{
            ++it;
        }


    }
}




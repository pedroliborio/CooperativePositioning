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

#ifndef __LOCVANET_LOCAPPCOM_H_
#define __LOCVANET_LOCAPPCOM_H_

#include <map>
#include <omnetpp.h>
#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/modules/messages/WaveServiceAdvertisement_m.h"
#include "veins/modules/messages/BasicSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;


//Utils libraries
#include <exception>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <fstream>

//Localization includes
#include <Types.h>
#include <GPS.h>
#include <Multilateration.h>
#include <DeadReckoning.h>
#include <RSSI/FreeSpaceModel.h>
#include <RSSI/TwoRayInterferenceModel.h>
#include <Projections/Projection.h>
#include <Outage/Outage.h>
#include <Filters/Filters.h>
#include <MapMatching/MapMatching.h>
#include <OutagesServer/Outages.h>

//Localization namespace
using namespace Localization;
using namespace GeographicLib;

//#define DBG_APP std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath() << " "

#ifndef DBG_APP
#define DBG_APP EV
#endif

/**
 * @brief
 * WAVE application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class LocAppCom : public BaseApplLayer {

    public:
        ~LocAppCom();
        virtual void initialize(int stage);
        virtual void finish();

        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);

        enum WaveApplMessageKinds {
            SEND_BEACON_EVT,
            SEND_WSA_EVT,
            //FIXME Added by Pedro
            SEND_FWDBEACON_EVT
        };

    private:

        time_t timeSeed;
        //Struct with the attributes of a neighbor node
        bool isInOutage; //Flag that indicates the begins of outage;
        bool isInRecover; //Flag that Indicates the begins of recover;
        std::list<AnchorNode> anchorNodes;//list of neighbor vehicles
        double residual; //sum of residuals
        bool IsMultOk; //Flag that indicates success in multilateration
        time_t timer = 0;

        //Lista que simula um fila FIFO para encaminhar beacons...
        std::list<BasicSafetyMessage*> listFWDBeacons;
        std::list<uint32_t> listIds;


        //**************Position Variables
        Coord coopPosRSSIFS; //CP FS
        Coord coopPosRSSITRGI; //CP TRGI
        Coord coopPosRSSIDR; //CP DR
        Coord coopPosDR;
        Coord coopPosReal;
        double errorCPReal;
        LonLat lastSUMOGeoPos, atualSUMOGeoPos;
        Coord lastSUMOUTMPos, atualSUMOUTMPos;

        //****************Modules GPS and DR
        DeadReckoning *drModule;
        GPS *gpsModule;

        //***************Outage Module
        Outage *outageModule;

        //***************Map Projections
        Projection *projection;

        //***************Multilateration methods
        Multilateration *multilateration;

        //*******MapMatching
        MapMatching *mapMatching;

        //*************Error Variables
        double errorGPSOut, errorGPSRec, errorGDR;

        //************RSSI Models
        FreeSpaceModel *fsModel;
        TwoRayInterferenceModel *trgiModel;

        //************Filters
        Filters *filter;

        //***********************RSSI Variables
        double constVelLight = 299792458.0; //m/s
        double lambda = 0.051; //wave length for CCH frequency
        double frequencyCCH = 5.890; //GHz
        double pTx = 20.0; //milliWatts
        double alpha = 2.0; // pathloss exponent
        double epsilonR = 1.02; //dieletric constant
        double ht = 1.895; //height of antenn transmitter
        double hr = 1.895; //height of antenna receiver

    protected:
        static const simsignalwrap_t mobilityStateChangedSignal;
        static const simsignalwrap_t parkingStateChangedSignal;

        /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
        virtual void handleLowerMsg(cMessage* msg);

        /** @brief handle self messages */
        virtual void handleSelfMsg(cMessage* msg);

        /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
        virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=0, int serial=0);

        /** @brief this function is called upon receiving a WaveShortMessage */
        virtual void onWSM(WaveShortMessage* wsm);

        /** @brief this function is called upon receiving a BasicSafetyMessage, also referred to as a beacon  */
        virtual void onBSM(BasicSafetyMessage* bsm);

        /** @brief this function is called upon receiving a WaveServiceAdvertisement */
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        /** @brief this function is called every time the vehicle receives a position update signal */
        virtual void handlePositionUpdate(cObject* obj);

        /** @brief this function is called every time the vehicle parks or starts moving again */
        virtual void handleParkingUpdate(cObject* obj);

        /** @brief This will start the periodic advertising of the new service on the CCH
         *
         *  @param channel the channel on which the service is provided
         *  @param serviceId a service ID to be used with the service
         *  @param serviceDescription a literal description of the service
         */
        virtual void startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription);

        /** @brief stopping the service and advertising for it */
        virtual void stopService();

        /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
         *
         * @param interval the interval length of the periodic message
         * @param chantype the type of channel, either type_CCH or type_SCH
         */
        virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, t_channel chantype);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         */
        virtual void sendDown(cMessage* msg);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         * @param delay the delay for the message
         */
        virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

        /**
         * @brief helper function for error handling and stats recording purposes
         *
         * @param msg the message to be checked and tracked
         */
        virtual void checkAndTrackPacket(cMessage* msg);

        /*
         * From here are implemented all methods for LocAppCom
         *
         */

        /**
         * @brief Function to initialize all localization modules
         */
        void InitLocModules();

        /**
         * @brief Put all kinematics GPS, DR, CP information on beacon before send it.
         */
        void PutBeaconInformation(BasicSafetyMessage* bsm);

        /**
         * @brief Discar All old beacon from the neighbors list
         */
        void DiscardOldBeacons();

        /**
         * @brief Makes Multilateration and update CP
         */
        void UpdateCooperativePositioning();

        /**
         * @brief Improve DR Position with CP Position
         */
        void ImproveDeadReckoning();


        /**
         * @brief Write Log Files for each vehicle
         */
        virtual void WriteLogFiles();

        /*
         * @brief Update RMSE Statistics
         */
        virtual void RMSEStatistics();

        /*
         * Compute Final Localization statistics and record on scalar file
         */
        virtual void ComputeLocStats();

        bool BeaconIsDuplicated(BasicSafetyMessage*);
        bool BeaconHaveMyId(BasicSafetyMessage*);
        bool BeaconIsAlive(BasicSafetyMessage*);
        void AddBeaconToForward(BasicSafetyMessage*);
        void DeleteOldBeaconToForward();


        void PutInNeighborList(AnchorNode *anchorNode);
        void UpdateNeighborList(AnchorNode *anchorNode);
        void UpdateNeighborListDistances(void);
        void PrintNeighborList();
        void PrintAnchorNode(AnchorNode *anchorNode);
        void GeodesicDRModule(void);
        void VehicleKinematicsModule(void);
        void getAnchorNode(int id, AnchorNode *anchorNode);
        void GetGPSOutageCoordinates();
        void GetOutageDataFromFile(std::string path);
        bool RecognizeOutage();
        bool RecognizeRecover();
        void RecognizeEdges(void);
        std::string GetTunnelString();
        bool IsAGoodBeacon(AnchorNode * anchorNode);

        void UpdateRange();

        void SortByResidual();

        double SetResidual();

    protected:

        /* pointers ill be set when used with TraCIMobility */
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;

        /*Pointer to access Outages Module*/

        /* Logging Parameters*/
        uint32_t logSeedPar;
        uint32_t logDistVehPar;

        AnnotationManager* annotations;
        WaveAppToMac1609_4Interface* mac;

        /* support for parking currently only works with TraCI */
        bool isParked;
        bool communicateWhileParked;

        /* BSM (beacon) settings */
        uint32_t beaconLengthBits;
        uint32_t  beaconPriority;
        simtime_t beaconInterval;
        bool sendBeacons;

        /*BSM (fowarding) settings*/
        bool forwardBeacons;
        uint32_t numHops;
        simtime_t forwardInterval;

        /* WSM (data) settings */
        uint32_t  dataLengthBits;
        uint32_t  dataPriority;
        bool dataOnSch;

        /* WSA settings */
        int currentOfferedServiceId;
        std::string currentServiceDescription;
        Channels::ChannelNumber currentServiceChannel;
        simtime_t wsaInterval;

        /* state of the vehicle */
        Coord curPosition;
        Coord curSpeed;
        int myId;
        int mySCH;

        /* stats */
        uint32_t generatedWSMs;
        uint32_t generatedWSAs;
        uint32_t generatedBSMs;
        uint32_t generatedFWDBSMs;
        uint32_t receivedWSMs;
        uint32_t receivedWSAs;
        uint32_t receivedBSMs;
        uint32_t receivedFWDBSMs;

        double rmseGPS;
        double rmseDRCP;
        double numRMSEs;

        double delaySum;

        simtime_t timestampOutage;
        simtime_t timestampRecover;

        /* messages for periodic events such as beacon and WSA transmissions */
        cMessage* sendBeaconEvt;
        cMessage* sendWSAEvt;
        //FIXME Added by Pedro
        cMessage* sendFWDBeaconEvt;
};

#endif /*__LOCVANET_LOCAPPCOM_H_ */

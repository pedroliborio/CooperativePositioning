/*
 * Types.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef TYPES_TYPES_H_
#define TYPES_TYPES_H_

#include "veins/base/utils/Coord.h"

struct t_LonLat{
    double lon;
    double lat;
};typedef struct t_LonLat LonLat;

struct AnchorNode_t{
            int vehID; //sender vehicle ID
            simtime_t timestamp; // timestamp of the beacon received
            //Positions
            Coord realPos;
            Coord deadReckPos;
            Coord gpsPos;
            //Distances
            double realDist;
            double deadReckDist;
            double gpsDist;
            //Error
            double errorGPS;
            double errorDR;
            //Rssi...
            double realRSSIDistFS; //Real RSSI Distance FS
            double realRSSIDistTRGI;//Real RSSI Distance TRGI
            double realRSSIFS; //Real RSSI FS;
            double realRSSITRGI; //Real RSSI TRGI

            double drRSSIDistFS; //Real RSSI Distance FS
            double drRSSIDistTRGI;//Real RSSI Distance TRGI
            double drRSSIFS; //Real RSSI FS;
            double drRSSITRGI; //Real RSSI TRGI

            // Avg Filter above real rssi dists
            double realRSSIDistAvgFilterFS = .0; //RSSI Distance FS Avg Filter
            double realRSSIDistAvgFilterTRGI = .0; //RSSI Distance TRGI Avg Filter
            //Avg filter above dr rssi dists
            double drRSSIDistAvgFilterFS = .0; //RSSI Distance FS Avg Filter
            double drRSSIDistAvgFilterTRGI = .0; //RSSI Distance TRGI Avg Filter

            int k = 0; //Actual Iteration for the Avg Filter
            bool inOutage;

};typedef struct AnchorNode_t AnchorNode;

namespace Localization {

class Types {
public:
    Types();
    virtual ~Types();
};

} /* namespace Localization */

#endif /* TYPES_TYPES_H_ */

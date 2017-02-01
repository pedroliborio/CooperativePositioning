/*
 * DeadReckoning.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <DeadReckoning/DeadReckoning.h>

namespace Localization {

DeadReckoning::DeadReckoning() {
    // TODO Auto-generated constructor stub

}

DeadReckoning::DeadReckoning(LonLat lastGPSPos) {
    // TODO Auto-generated constructor stub
    //Initialization of DR with last GPS Pos.
    lastKnowPosGeo = lastGPSPos;
}

DeadReckoning::~DeadReckoning() {
    // TODO Auto-generated destructor stub
}

void DeadReckoning::setGeoPos(LonLat *lastSUMOPos, LonLat *atualSUMOPos){
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());// Geodesic
    double lat, lon; //new GDR Cooridnates
    double s_12; //odometer
    double azi_1, azi_2; //azimuths gyrocompass

    //gyro and odometer...
    geod.Inverse(lastSUMOPos->lat, lastSUMOPos->lon, atualSUMOPos->lat, atualSUMOPos->lon, s_12, azi_1, azi_2);

    //calc new GDR position
    geod.Direct(lastKnowPosGeo.lat, lastKnowPosGeo.lon, azi_1, s_12, lat, lon);

    //new Recognized Coordinate LonLat
    lastKnowPosGeo.lat = lat;
    lastKnowPosGeo.lon = lon;
}

void DeadReckoning::setUTMPos(Coord utmCoord){
    this->lastKnowPosUTM = utmCoord;
}

void DeadReckoning::setErrorLonLat(LonLat *atualSUMOPos){
    double s_12;
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());// Geodesic
    //gyro and odometer...
    geod.Inverse(lastKnowPosGeo.lat, lastKnowPosGeo.lon, atualSUMOPos->lat, atualSUMOPos->lon, s_12);
    this->errorGeo = s_12;
}

void DeadReckoning::setErrorUTM(Coord *atualSUMOPos){
    this->errorUTM = this->lastKnowPosUTM.sqrdist(*atualSUMOPos);
}


} /* namespace Localization */

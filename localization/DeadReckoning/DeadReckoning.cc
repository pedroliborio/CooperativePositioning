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
    this->errorUTM = 0;
    this->errorGeo = 0;
    this->timeDR=0;
    this->sigma_theta=1;
}

DeadReckoning::~DeadReckoning() {
    // TODO Auto-generated destructor stub
}

void DeadReckoning::setGeoPos(LonLat *lastSUMOPos, LonLat *atualSUMOPos){
    //TODO save angle and distance traveled (gyro and odometer)
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());// Geodesic
    double lat, lon; //new GDR Cooridnates
    double s_12; //odometer
    double azi_1, azi_2; //azimuths gyrocompass

    //gyro and odometer...
    geod.Inverse(lastSUMOPos->lat, lastSUMOPos->lon, atualSUMOPos->lat, atualSUMOPos->lon, s_12, azi_1, azi_2);

    timeDR+=0.1;

    //update sigma_theta with the sources of noise
    sigma_theta = ANGLE_RANDOM_WALK_NOISE * sqrt(timeDR);

    sigma_theta += OFFSET * timeDR;

    sigma_theta += NON_LINEARITY * sqrt(timeDR);

    //std::cout << "sigma: " << std::setprecision(8) << sigma_theta << "\n\n";

    //FIXME After each one second use ine filter to minimize the error

    //Put the noise on the angle (azimuth)
    azi_1 += RNGCONTEXT normal(0,sigma_theta);

    //calc new GDR position
    geod.Direct(lastKnowPosGeo.lat, lastKnowPosGeo.lon, azi_1, s_12, lat, lon);

    //new Recognized Coordinate LonLat
    lastKnowPosGeo.lat = lat;
    lastKnowPosGeo.lon = lon;

    setErrorLonLat(atualSUMOPos);
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

void DeadReckoning::setErrorUTMPos(Coord *atualSUMOPos){

    this->errorUTM = this->lastKnowPosUTM.distance(*atualSUMOPos);
}



} /* namespace Localization */

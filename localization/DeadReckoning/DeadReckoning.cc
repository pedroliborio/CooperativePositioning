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

DeadReckoning::DeadReckoning(LonLat lastGPSPos, double updateInterval) {
    // TODO Auto-generated constructor stub
    //Initialization of DR with last GPS Pos.
    lastKnowPosGeo = lastGPSPos;
    this->errorUTM = 0;
    this->errorGeo = 0;
    this->timeDR=0;
    this->error=0;
    this->lPFTheta.setLpf(0);
    this->lPFTheta.setPrevxLpf(0);
    this->arw = 0;
    this->sensitivity = 0;

    FREQUENCY = updateInterval;

    //std::cout << updateInterval << endl;
    //Initializing ARW  and SENSITIVITY constants
    if(updateInterval <= 0.1){
        //std::cout << "barril 0.1" << endl;
        ANGLE_RANDOM_WALK_NOISE = 0.063245553;//0.063245553;// FFT 0.02°/s/sqrt(Hz) (1Hz) <-> 0.063245553 (10Hz) in 1 sec , 0.0063245553 in 0.1 sec (10Hz)
        _SENSITIVITY_ = 0.02; //2% also modeled as a white noise that grows linearly with the time
    }
    else{
        if(updateInterval >= 1.0){
            //std::cout << "barril 1.0" << endl;
            ANGLE_RANDOM_WALK_NOISE = 0.02;//0.063245553;// FFT 0.02°/s/sqrt(Hz) (1Hz) <-> 0.063245553 (10Hz) in 1 sec , 0.0063245553 in 0.1 sec (10Hz)
            _SENSITIVITY_ = 0.02; //2% also modeled as a white noise that grows linearly with the time
        }
    }
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

    timeDR+=FREQUENCY;

    //update sigma_theta with the sources of noise
    arw = ANGLE_RANDOM_WALK_NOISE * sqrt(timeDR);

    arw = RNGCONTEXT normal(0,arw);

    sensitivity = RNGCONTEXT normal(0,(azi_1 * _SENSITIVITY_));

    error += arw + sensitivity;

    lPFTheta.DoLowPassFilter(error);

    //std::cout << "sigma: " << std::setprecision(8) << sigma_theta << "\n\n";

    //FIXME After each one second use ine filter to minimize the error

    this->angle = azi_1;

    //Put the noise on the angle (azimuth)
    //azi_1 += RNGCONTEXT normal(0,error);
    azi_1 += lPFTheta.getLpf(); //angle with sentivity error

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

void DeadReckoning::ReinitializeSensors(){
    this->errorUTM = 0;
    this->errorGeo = 0;
    this->timeDR=0;
    this->error=0;
    this->lPFTheta.setLpf(0);
    this->lPFTheta.setPrevxLpf(0);
    this->arw = 0;
    this->sensitivity = 0;
}



} /* namespace Localization */

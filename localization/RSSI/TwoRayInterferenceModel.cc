/*
 * TwoRayInterferenceModel.cc
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <RSSI/TwoRayInterferenceModel.h>

namespace Localization {

TwoRayInterferenceModel::TwoRayInterferenceModel() {
    // TODO Auto-generated constructor stub

}

TwoRayInterferenceModel::~TwoRayInterferenceModel() {
    // TODO Auto-generated destructor stub
}

void TwoRayInterferenceModel::setRSSI(double d, double pTx, double lambda, double ht, double hr, double epsilonR){
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi, attenuation;

    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)

    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path

    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta

    cosTheta = d/distRef; //cos of the angle of incidence

    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection

    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays

    attenuation = pow(4 * M_PI * (d/lambda) * 1/(sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) )), 2); //# mw

    rssi = 10*log10(pTx) - 10 * log10(attenuation);
    //dbmW
}

void TwoRayInterferenceModel::setDistance(double rssi, double d, double pTx, double lambda, double ht, double hr, double epsilonR){
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi;

    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)
    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path
    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta
    cosTheta = d/distRef; //cos of the angle of incidence
    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection
    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays

    distance =  sqrt(pTx) * (lambda / 4 * M_PI) * (pow(10,( (-20 -(rssi)) / 20)))  *
                (sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) ));
    //meters
}

} /* namespace Localization */

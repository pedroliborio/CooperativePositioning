/*
 * Projection.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: liborio
 */

#include <Projections/Projection.h>

namespace Localization {

Projection::Projection() {
    // TODO Auto-generated constructor stub

}

Projection::Projection(std::string fileProjName) {
    std::string catcher, projString;

    std::fstream file("../localization/Projections/parameters/"+fileProjName+".txt");

    file >> this->netOffset.u >> this->netOffset.v;
    std::getline(file,projString);
    projString = projString.substr(1);

    //std::cout << std::setprecision(10) << this->netOffset.u <<'-'<< std::setprecision(10) << this->netOffset.v <<'x'<< projString<<'-'<< endl;

    if (!( this->pj_utm = pj_init_plus(projString.c_str()) )){
        std::cout << "Problem in Projections when init." << endl;
        exit(0);
    }
}

Projection::~Projection() {
    // TODO Auto-generated destructor stub
}

void Projection::FromUTMToLonLat(void){
    projLP geo_coord;
    projXY coord_utm;

    coord_utm.u = utmCoord.x - netOffset.u;
    coord_utm.v = utmCoord.y - netOffset.v;

    geo_coord = pj_inv(coord_utm,pj_utm);

    geoCoord.lon= geo_coord.u;
    geoCoord.lat= geo_coord.v;

    geoCoord.lon*= RAD_TO_DEG;
    geoCoord.lat*= RAD_TO_DEG;
}

void Projection::FromLonLatToUTM(void){
    projLP geo_coord;
    projXY coord_utm;

    geo_coord.u = geoCoord.lon;
    geo_coord.v = geoCoord.lat;

    geo_coord.u*= DEG_TO_RAD;
    geo_coord.v*= DEG_TO_RAD;

    coord_utm = pj_fwd(geo_coord,pj_utm);

    utmCoord.x = coord_utm.u + netOffset.u;
    utmCoord.y = coord_utm.v + netOffset.v;
}

} /* namespace Localization */

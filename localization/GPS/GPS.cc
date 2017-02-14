/*
 * GPS.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <GPS/GPS.h>

namespace Localization {

GPS::GPS() {
    // TODO Auto-generated constructor stub

}

GPS::GPS(std::string outagesFile) {
    GetDataSetMeanSTD(outagesFile);
}

GPS::~GPS() {
    // TODO Auto-generated destructor stub
}

void GPS::GetDataSetMeanSTD(std::string outagesFile){
    std::string path = "../localization/GPS/mean_std_dev/"+outagesFile+".txt";
    std::fstream file(path);
    std::cout << path << endl;
    file >> this->mean >> this->std;
    //std::cout << "Mean, STD:"<< this->mean <<" , "<< this->std << endl;
    file.close();
}


void GPS::CompPosition(Coord *realCoord){
    //FIXME Maybe is necessary use the sumo seed... verify
    double diff = (RNGCONTEXT normal(mean, std));

    position.x = realCoord->x + diff;
    position.y = realCoord->y + diff;
    position.z = realCoord->z + diff;

    CompError(realCoord);

    //std::cout <<"Real Coord: "<< *realCoord<< endl;
    //std::cout <<"GPS Position: "<< position << endl;
    //std::cout <<"Error: "<< error << endl;

}

void GPS::CompError(Coord *realCoord){
    error = position.distance(*realCoord);
}


//FIXME This method will be migrated to Outages Module
/*void GPS::GetOutageInformation(std::string outagesFile){
    std::string date, time, line;
    std::string path = "../localization/GPS/outagesxy/"+outagesFile+".txt";
    std::fstream file(path);
    //FIXME probably need to indicate that the second file is to output verify it
    std::fstream fileOut("temp.txt");

    getline(file, line); // get header

    file >> date >> time >> gpsOutGeoPos.lat >> gpsOutGeoPos.lon >> errorGPSOut;

    file >> date >> time >> gpsRecGeoPos.lat >> gpsRecGeoPos.lon >> errorGPSRec;

    //put header back
    fileOut << line;
    //put another lines back
    while (getline(file,line)){
        fileOut << line << '\n';
    }

    fileOut.close();
    file.close();
}*/


} /* namespace Localization */

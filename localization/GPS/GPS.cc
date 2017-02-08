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
    GetOutageInformation(outagesFile);
}

GPS::~GPS() {
    // TODO Auto-generated destructor stub
}

void GPS::GetOutageInformation(std::string outagesFile){
    std::string date, time, line;
    std::string path = "../localization/GPS/outages/"+outagesFile+".txt";
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
}


} /* namespace Localization */

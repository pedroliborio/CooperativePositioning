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
    //GetDataSetMeanSTD(outagesFile);

    if(outagesFile == "DMATEntranceExit"){
        this->mean = 6.19143868296;
        this->std = 2.88459214044;
    }
    else{
        if(outagesFile == "DMATExitEntrance"){
            this->mean = 6.42315413843;
            this->std = 2.84527635525;
        }
        else{
            if(outagesFile == "DPTEntranceExit"){
                this->mean = 6.23195053621;
                this->std = 2.95601551205;
            }
            else{
                if(outagesFile=="DPTExitEntrance"){
                    this->mean = 7.59763440355;
                    this->std = 3.41503134687;
                }
                else{
                    if(outagesFile=="RCLTEntranceExit"){
                        this->mean=6.97907769597;
                        this->std = 3.30866358332;
                    }
                    else{
                        if(outagesFile=="RCLTExitEntrance"){
                            this->mean = 6.13565937547;
                            this->std = 3.21445123213;
                        }
                        else{
                            if(outagesFile=="YBTEntranceExit"){
                                this->mean = 8.93352084142;
                                this->std=3.41637848527;
                            }
                            else{
                                if(outagesFile=="YBTExitEntrance"){
                                    this->mean = 9.5998328591;
                                    this->std = 2.83305514461;

                                }
                                else{
                                    if(outagesFile=="RIO450EntranceExit"){
                                        this->mean= 9.50330634515;
                                        this->std=3.42732536713;
                                    }
                                }

                            }
                        }
                    }
                }
            }
        }
    }
}

GPS::~GPS() {
    // TODO Auto-generated destructor stub
}

/*void GPS::GetDataSetMeanSTD(std::string outagesFile){
    std::string path = "../localization/GPS/mean_std_dev/"+outagesFile+".txt";
    std::fstream file(path);
    //std::cout << path << endl;
    file >> this->mean >> this->std;
    //std::cout << "Mean, STD:"<< this->mean <<" , "<< this->std << endl;
    file.close();
}*/


void GPS::CompPosition(Coord *realCoord){
    //FIXME Maybe is necessary use the sumo seed... verify
    //Here we generate a random number in teh perimeter of an circle
    double diff, angle, radius;

    diff = (RNGCONTEXT normal(mean, std));

    position.x = realCoord->x + diff;
    position.y = realCoord->y + diff;
    position.z = realCoord->z;

    radius = realCoord->distance(position);

    angle = ( RNGCONTEXT normal(0,3.141592653589793*2) );

    position.x = realCoord->x + cos(angle)*radius;
    position.y = realCoord->y + sin(angle)*radius;

    /*if(position.x < 1){
        position.x = 0;
    }
    if(position.y < 1){
        position.y = 0;
    }*/
    //TODO Decides after if will use a 3D approach...
    //3D position.z = realCoord->z + diff;i

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

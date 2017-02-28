/*
 * Outage.cc
 *
 *  Created on: Jan 23, 2017
 *      Author: liborio
 */

#include <Outage/Outage.h>

namespace Localization {

Outage::Outage() {
    // TODO Auto-generated constructor stub


}

Outage::Outage(std::string outagesFile) {
    //FIXME Use one counter to know what is the correct line or take out lines read of the file
    double error;
    int i,numOutage, totalOutage;
    std::string date, time, line;

    std::string pathGPS = "../localization/GPS/outagesxy/"+outagesFile+".txt";
    std::string pathCount = "../localization/GPS/outagesxy/"+outagesFile+"Count.txt";

    std::fstream file(pathGPS.c_str());
    std::fstream fileCount(pathCount.c_str());

    if(!file.is_open()){
        std::cout << "Outages.cc: GPS FILE Error" << endl;
        exit(0);
    }
    else{
        if(!fileCount.is_open()){
            std::cout << "Outages.cc: Counter FILE Error" << endl;
            exit(0);
        }
    }

    fileCount >> numOutage >> totalOutage;//read counter of outages
    //FiXME verify if numoutage reaches totoutages condicao de parada do bagui
    if (numOutage == totalOutage){
        numOutage = 0;
    }

    getline(file, line); // get header GPS outages file

    for(i=0; i < numOutage; i++){
        file >> date >> time >> outagePos.x >> outagePos.y >> error;
        file >> date >> time >> recoverPos.x >> recoverPos.y >> error;
    }
    /*std:: cout << "Outage" <<endl;
    std::cout << outagePos << endl;
    std::cout << recoverPos << endl;*/

    fileCount.close();
    file.close();

    fileCount.open(pathCount.c_str(),std::fstream::out);
    if(!fileCount.is_open()){
        std::cout << "Error in Class Outage: files!" << endl;
        exit(0);
    }

    numOutage++;

    fileCount << numOutage <<'\t'<< totalOutage<<'\n';
    fileCount.close();

    //initialize outage recover flags
    inOutage = false;
    inRecover = false;
}


void Outage::ControlOutage(Coord *sumoPos){
    double dist;
    if(inOutage == false && inRecover==false ){
        dist = outagePos.distance(*sumoPos);
        if(dist > distOutage){
            inOutage = true;
        }
        else{
            distOutage = dist;
        }
    }
    else{
        if(inOutage == true && inRecover==false ){
            dist = recoverPos.distance(*sumoPos);
            if(dist > distRecover){
                inOutage = false;
                inRecover = true;
            }
            else{
                distRecover = dist;
            }
        }
    }
}

Outage::~Outage() {
    // TODO Auto-generated destructor stub
}

} /* namespace Localization */

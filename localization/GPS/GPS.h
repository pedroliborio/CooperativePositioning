/*
 * GPS.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_GPS_GPS_H_
#define LOCALIZATION_GPS_GPS_H_


#include <Types.h>
#include <fstream>

using namespace std;

namespace Localization {

class GPS {
private:
    double std;
    double mean;
    Coord position;
    double error;
public:
    GPS();
    GPS(std::string outagesFile);
    virtual ~GPS();
    void GetDataSetMeanSTD(std::string outagesFile);
    void CompPosition(Coord *realCoord);
    void CompError(Coord *realCoord);

    double getError() const {
        return error;
    }

    void setError(double error) {
        this->error = error;
    }

    const Coord& getPosition() const {
        return position;
    }

    void setPosition(const Coord& position) {
        this->position = position;
    }

    double getMean() const {
        return mean;
    }

    void setMean(double mean) {
        this->mean = mean;
    }

    double getStd() const {
        return std;
    }

    void setStd(double std) {
        this->std = std;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_GPS_GPS_H_ */

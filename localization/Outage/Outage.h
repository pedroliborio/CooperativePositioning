/*
 * Outage.h
 *
 *  Created on: Jan 23, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_OUTAGE_H_
#define LOCALIZATION_OUTAGE_H_

#include <Types.h>
#include <limits>
#include <fstream>

namespace Localization {

class Outage {
private:
    bool inOutage;
    bool inRecover;
    Coord outagePos;
    Coord recoverPos;
    double distOutage = std::numeric_limits<double>::max();
    double distRecover = std::numeric_limits<double>::max();
    void LoadOutage(std::string outagesFile);
    void RCLT(std::string outagesFile);
    void DMAT(std::string outagesFile);
    void YBT(std::string outagesFile);
    void DPT(std::string outagesFile);
    void RIO450(std::string outagesFile);


public:
    Outage();
    Outage(std::string outagesFile);
    virtual ~Outage();
    void ControlOutage(Coord *sumoPos);

    double getDistOutage() const {
        return distOutage;
    }

    void setDistOutage(double distOutage) {
        this->distOutage = distOutage;
    }

    double getDistRecover() const {
        return distRecover;
    }

    void setDistRecover(double distRecover) {
        this->distRecover = distRecover;
    }

    bool isInOutage() const {
        return inOutage;
    }

    void setInOutage(bool inOutage) {
        this->inOutage = inOutage;
    }

    bool isInRecover() const {
        return inRecover;
    }

    void setInRecover(bool inRecover) {
        this->inRecover = inRecover;
    }

    const Coord& getOutagePos() const {
        return outagePos;
    }

    void setOutagePos(const Coord& outagePos) {
        this->outagePos = outagePos;
    }

    const Coord& getRecoverPos() const {
        return recoverPos;
    }

    void setRecoverPos(const Coord& recoverPos) {
        this->recoverPos = recoverPos;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_OUTAGE_H_ */

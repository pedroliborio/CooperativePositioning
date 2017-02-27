/*
 * LowPassFilter.h
 *
 *  Created on: Feb 25, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_FILTERS_LOWPASSFILTER_H_
#define LOCALIZATION_FILTERS_LOWPASSFILTER_H_

namespace Localization {

class LowPassFilter {
private:
    //FIXME Probably the best way is increase alpha with the time...
    double alpha;// weight applied above the estimation
    double xLPF;// actual LPF estimation
    double prevxLPF; //previous xLPF
    bool firstRun;//First Run
public:
    LowPassFilter();
    virtual ~LowPassFilter();
    void DoLowPassFilter(double measure);

    double getPrevxLpf() const {
        return prevxLPF;
    }

    void setPrevxLpf(double prevxLpf) {
        prevxLPF = prevxLpf;
    }

    double getLpf() const {
        return xLPF;
    }

    void setLpf(double lpf) {
        xLPF = lpf;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_FILTERS_LOWPASSFILTER_H_ */

/*
 * LowPassFilter.cc
 *
 *  Created on: Feb 25, 2017
 *      Author: liborio
 */

#include <LowPassFilter.h>

namespace Localization {

LowPassFilter::LowPassFilter() {
    firstRun = true;
    alpha = 0.9;
}

LowPassFilter::~LowPassFilter() {
    // TODO Auto-generated destructor stub
}

void LowPassFilter::DoLowPassFilter(double x){
    //TODO alternate alpha with the time of usage
    if(firstRun){
        prevxLPF = x;
        firstRun = false;
    }
    else{
        xLPF = alpha * prevxLPF + (1-alpha) * x;

        prevxLPF = xLPF;
    }

}

} /* namespace Localization */

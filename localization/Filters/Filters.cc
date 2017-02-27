/*
 * Filters.cc
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <Filters.h>

namespace Localization {

Filters::Filters() {
    // TODO Auto-generated constructor stub

}

Filters::~Filters() {
    // TODO Auto-generated destructor stub
}

void Filters::setAverageFilter(int k, double mean, double x_k){
    double alpha = (k-1)/(double)k;
    avgFilter = (alpha * mean) + (1-alpha) * x_k;
}



} /* namespace Localization */

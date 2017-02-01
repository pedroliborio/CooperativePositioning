/*
 * Filters.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_FILTERS_FILTERS_H_
#define LOCALIZATION_FILTERS_FILTERS_H_

namespace Localization {

class Filters {
private:
    double avgFilter;
public:
    Filters();
    virtual ~Filters();
    void setAverageFilter(int k, double mean, double x_k);

    double getAvgFilter() const {
        return avgFilter;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_FILTERS_FILTERS_H_ */

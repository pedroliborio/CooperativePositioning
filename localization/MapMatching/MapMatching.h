/*
 * MapMatching.h
 *
 *  Created on: Feb 8, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_MAPMATCHING_MAPMATCHING_H_
#define LOCALIZATION_MAPMATCHING_MAPMATCHING_H_

#include <Types.h>
#include <fstream>

struct t_Edge{
    std::string id;
    std::vector<Coord> shape;
};typedef struct t_Edge Edge;

namespace Localization {

class MapMatching {
private:
    Coord matchPoint;//point inside the edge with min dist to P_gps
    double distGPSMM;//distance between P_GPS, P_Mp

    std::vector<Edge> listEdges;//Edges extracted from .net file
    double Magnitude( Coord *A, Coord *B );
    int DistancePointLine(Coord *A, Coord *B, Coord *P, Coord *intersection, double *distance);
public:
    MapMatching();
    MapMatching(std::string route);
    void DoMapMatching(std::string edgeID, Coord pGPS);
    virtual ~MapMatching();

    double getDistGpsmm() const {
            return distGPSMM;
        }

    const Coord& getMatchPoint() const {
        return matchPoint;
    }
};

} /* namespace Localization */

#endif /* LOCALIZATION_MAPMATCHING_MAPMATCHING_H_ */

/*
 * MapMatching.cc
 *
 *  Created on: Feb 8, 2017
 *      Author: liborio
 */

#include <MapMatching/MapMatching.h>

namespace Localization {

MapMatching::MapMatching() {
    // TODO Auto-generated constructor stub

}

MapMatching::MapMatching(std::string route) {
    // TODO Parse the path for the correct when needed

    std::string routeFileName = "../localization/Graphs/out/"+route+".txt";
    std::fstream fileGraph(routeFileName.c_str());
    Edge edge;
    Coord coord;
    int i, totalCoords;

    while(fileGraph >> edge.id >> totalCoords){

        for (i = 0; i < totalCoords; i++){
            fileGraph >> coord.x >> coord.y;
            edge.shape.push_back(coord);
        }

        listEdges.push_back(edge);
    }
}

//Calcular menor distancia ponto reta http://stackoverflow.com/questions/12132352/distance-from-a-point-to-a-line-segment
//http://stackoverflow.com/questions/3120357/get-closest-point-to-a-line
//Thanks for Paul Bourke http://paulbourke.net/geometry/pointlineplane/
//FIXME Calculate in 3D Space anyway.

double MapMatching::Magnitude( Coord *A, Coord *B )
{
    std::cout << *A << *B << endl;
    return sqrt( ( (B->x - A->x) * (B->x - A->x) ) + ((B->y - A->y) * (B->y - A->y) ) );
}

int MapMatching::DistancePointLine(Coord *A, Coord *B, Coord *P, Coord *intersection, double *distance){

    std::cout << *A << *B << *P <<"\n\n";

    double magnitude;
    double U;
    magnitude = Magnitude(A,B);

    U = ( ( (P->x - A->x) * (B->x - A->x) ) +
          ( (P->y - A->y) * (B->y - A->y) ) )
        / (magnitude * magnitude);

    std::cout << U <<"****\n";

    if( U < (double) 0.0 || U > (double) 1.0){
        return 0; //closest point does not fall within the line segment
    }

    intersection->x = A->x + U * (B->x - A->x);
    intersection->y = A->y + U * (B->y - A->y);

    *distance = Magnitude(P,intersection);

    return 1; //lies at the intersection point and with distance
}


void MapMatching::DoMapMatching(std::string edgeID, Coord pGPS){
    uint i;
    Coord A, B, P;

    for(std::vector<Edge>::iterator it = listEdges.begin(); it!= listEdges.end(); ++it){
        if(it->id == edgeID){
            for(i=1; i < it->shape.size(); i++){
                A = it->shape.at(i-1);
                B = it->shape.at(i);
                P = pGPS;

                if( DistancePointLine( &A, &B, &P, &matchPoint, &distGPSMM) ){
                    std::cout << "Computado com sucesso!\n\n";
                    break;
                }
                else{
                    std::cout << "Aresta nao encontrada!\n\n";
                }
            }
            break;
        }
    }
}

MapMatching::~MapMatching() {
    // TODO Auto-generated destructor stub
}

} /* namespace Localization */

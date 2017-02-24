/*
 * Multilateration.cc
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <Multilateration.h>

namespace Localization {

Multilateration::Multilateration() {
    // TODO Auto-generated constructor stub

}

Multilateration::~Multilateration() {
    // TODO Auto-generated destructor stub
}

void Multilateration::LeastSquares(void){
        int i;
        //Minus one because the last line of the matrix will be subtracted by the others lines
        int size = positions.size();
        std::cout << size << endl;
        size--;

        std::cout << size << endl;

        //Create matrixes using the TNT library
        //Composing the Linear Equation Ax - b to be solved by LeastSquares
        TNT::Array2D<double> A(size,2);
        TNT::Array1D<double> b(size);
        TNT::Array1D<double> x(size);

        //posToSubtract = positions[size];
        //distToSubtract = (double) distances->at(size);

        for(i=0; i < size; i++){
            std::cout << positions.at(i) << "\t" << distances.at(i) << endl;
            A[i][0] =  2.0 * (positions.at(i).x - positions.at(size).x);
            A[i][1] =  2.0 * (positions.at(i).y - positions.at(size).y);

            b[i] = ( pow(distances.at(size),2) - pow(distances.at(i),2)     ) +
                   ( pow(positions.at(i).x,2) - pow(positions.at(size).x,2) ) +
                   ( pow(positions.at(i).y,2) - pow(positions.at(size).y,2) );
        }

        std::cout << positions.at(size) << "\t" << distances.at(size) << endl;

        JAMA::QR<double> qrFact(A);

        x = qrFact.solve(b);

        if(x.dim1() == 0){
            std::cout << "Error in last Squares!" << endl;
        }

        estPosition.x = x[0];
        estPosition.y = x[1];
        estPosition.z = 0;

        for(std::vector<double>::iterator it = distances.begin(); it!= distances.end(); ++it ){
            std::cout << std::setprecision(10) << *it << endl;
        }

        for(std::vector<Coord>::iterator it = positions.begin(); it!= positions.end(); ++it ){
            std::cout << *it << endl;
        }

        std::cout << estPosition << endl;
}

void Multilateration::getDistList(std::list<AnchorNode> *anchorNodes, const int DIST_TYPE){
    int i = 0;
    if(DIST_TYPE == REAL_DIST){
        for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
            distances.push_back(it->realDist);
            i++;
        }
    }
    else{
        if(DIST_TYPE == DR_DIST){
            for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
                distances.push_back(it->deadReckDist);
                i++;
            }
        }
        else{
            if(DIST_TYPE == GPS_DIST){
                for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
                    distances.push_back(it->gpsDist);
                    i++;
                }
            }
            else{
                if(DIST_TYPE == FS_DIST){
                    for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
                        distances.push_back(it->realRSSIDistFS);
                        i++;
                    }
                }
                else{
                    if(DIST_TYPE == TRGI_DIST){
                        for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
                            distances.push_back(it->realRSSIDistTRGI);
                            i++;
                        }
                    }
                    else{
                        std::cout << "Multilateration: Type of distance not found";
                        exit(0);
                    }
                }

            }
        }
    }
}

void Multilateration::getPosList(std::list<AnchorNode> *anchorNodes, const int POS_TYPE){
    int i =0;
    if(POS_TYPE == REAL_POS){
        for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
            positions.push_back(it->realPos);
            i++;
        }
    }
    else{
        if(POS_TYPE == DR_POS){
            for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
                positions.push_back(it->deadReckPos);
                i++;
            }
        }
        else{
            if(POS_TYPE == GPS_POS){
               for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
                   positions.push_back(it->gpsPos);
                   i++;
               }
            }
            else{
                std::cout << "Multilateration: Type of Position not found";
                exit(0);
            }
        }
    }
}

void Multilateration::DoMultilateration(std::list<AnchorNode> *anchorNodes, const int POS_TYPE, const int DIST_TYPE){
    //get distances
    getDistList(anchorNodes, DIST_TYPE);
    //get positions
    getPosList(anchorNodes, POS_TYPE);
    //Call Multilateration using LeastSquares
    LeastSquares();
    //Free memory
    this->distances.clear();
    this->positions.clear();
}
} /* namespace Localization */

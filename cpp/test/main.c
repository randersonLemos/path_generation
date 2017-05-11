#include <iostream>
#include "testFunctions.h"

int main(){
    float arr[] = {1,1,1};
    changeArray(arr);
    
    for(int i=0; i!=3; ++i){
        std::cout << *(arr+i) << " ";
    }
    std::cout << std::endl;

    float **arr2d = new float*[3];
    for(int i=0; i!=3; ++i){
        arr2d[i] = new float[3];
    }
    
    for(int i=0; i!=3; ++i){
        for(int j=0; j!=3; ++j){
            arr2d[i][j] = 1;
        }
    }

    changeMatrix(arr2d);

    for(int i=0; i!=3; ++i){
        for(int j=0; j!=3; ++j){
            std::cout << arr2d[i][j] <<  " ";
        }
        std::cout << std::endl;
    }
}

#include "testFunctions.h"

int plusOne(int x){
    return x + 1;
}

int changeArray(float *arr){
    arr[0] = 10;
    return 1;
}

int changeMatrix(float **mat){
    mat[0][0] = 10;
    return 1;
}

#ifndef RANSAC2DLINE_H
#define RANSAC2DLINE_H

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ransac_2Dline - ajusta uma reta que melhor se encaixa a nuvem de pontos utilizando o algoritmo ransac

	Entradas
	data 	: matriz contendo a nuvem de pontos em corrdenadas cartesianas
	n		: numero de elementos na matriz data
	maxT	: numero máximo de iteracoes para o algoritmo ransac
	threshold: distancia maxima para que um ponto seja considerado inlier
	verbose	: se verbose=1, a funçao imprime algumas informaçoes uteis na saida padrao	

	Saidas
	bestModel	: coeficientes do melhor modelo encontrado pelo ransac
	bestInliers	: quantidade de pontos que se encaixam ao modelo bestModel dentro do threshold definido
	
	Retorna 0 em caso de sucesso.
*/
extern "C" int ransac_2Dline(float **data, int n, int maxT, float threshold,
					         float *bestModel, float *bestScore, int *bestInliers,
                             int side, int verbose);


/* randomSelect - retira n pontos aleatorios de uma matriz

	Entradas
	nsel	: numero de pontos aleatorios a serem selecionados
	data 	: matriz contendo a nuvem de pontos em coordenadas cartesianas
	ndata	: numero de elementos na matriz data

	Saidas
	sel		: matriz contendo os pontos selecionados e removidos de data
	
	Retorna 0 em caso de sucesso.
*/
int randomSelect(float **sel, int nsel, float **data, int ndata);


/* fitModel_line - verifica se um dado ponto se encaixa a um modelo dentro do threshold definido

	Entradas
	point	: coordenadas do ponto (x, y)
	l 		: vetor contendo os coeficientes do modelo (a*x + b*y + c = 0)
	threshold: distancia maxima para que um ponto seja considerado inlier
	
	Retorna 1 caso o ponto seja um inlier e 0 caso seja outlier.
*/
int fitModel_line(float *point, float *l, float threshold);


/* estimateModel_line - estima o modelo de uma reta que se encaixa aos pontos dados, o modelo a*x + b*y + c = 0 é obtido por decomposicao SVD.

	Entradas
	P	: vetor contendo os pontos
	n	: numero de pontos no vetor P
	
	Saidas
	l 	: vetor contendo os coeficientes do modelo (a*x + b*y + c = 0)
*/
void estimateModel_line(float *l, float **P, int n);

/* twoPointsLine - calcula os coeficientes de uma reta que passa pelos dois pontos dados.

	Entradas
	P	: vetor contendo dois pontos (x,y)
	
	Saidas
	l 	: vetor contendo os coeficientes do modelo (a*x + b*y + c = 0)
*/
void twoPointsLine(float *l, float **P);

#endif /* RANSAC2DLINE_H */

#include "ransac2Dline.hpp"

int ransac_2Dline(float **data, int n, int maxT, float threshold,
                  float *bestModel, float *bestScore, int *bestInliers,
                  int side, int verbose) {

    if(verbose)
        printf("Start RANSAC, n=%d, maxT=%d, t=%.2f\n\n", n, maxT, threshold);

    *bestScore = 0;

    int Tcount = 0;
    int ndata = n;
    int nr = 2;
    int i = 0;

    float **randSet = (float **) malloc(nr * sizeof(float *));
    if(randSet == NULL) { perror("out of memory\n"); exit(0); }

    for(i = 0; i < nr; i++)
    {
        randSet[i] = (float *) malloc(2 * sizeof(float));
        if(randSet[i] == NULL) { perror("out of memory\n"); exit(0); }
    }

    float **conSet = (float **) malloc(ndata * sizeof(float *));
    if(conSet == NULL) { perror("out of memory\n"); exit(0); }

    for(i = 0; i < ndata; i++)
    {
        conSet[i] = (float *) malloc(2 * sizeof(float));
        if(conSet[i] == NULL) { perror("out of memory\n"); exit(0); }
    }

    srand((unsigned)time(NULL)); // set rand seed

    while(maxT > Tcount)
    {
        if(verbose)
            printf("\n#%d ITERATION >>>>>>>>>>>>>>>>>>>>>>>\n", Tcount);

        // Select 2 points at random to form a trial model
        if(randomSelect(randSet, nr, data, ndata)) { break; }

        if(verbose)
            printf(" selected points: (%.3f, %.3f) and (%.3f, %.3f)\n", 
                    randSet[0][0], randSet[0][1], randSet[1][0], 
                    randSet[1][1]);

        // Fit model to the random selection of data points
        float randModel[3];
        twoPointsLine(randModel, randSet);

        if(verbose)
            printf(" rand model: %.3f*x + %.3f*y + %.3f = 0\n", randModel[0], 
            randModel[1], randModel[2]);

        int inliers = 0;
        float point[2];

        // float inner_point[2];
        // inner_point[0] = data[0][0]; inner_point[1] = data[0][1];
        // Evaluate distances between points and model.
        // Given a threshold, create a consensus set with the points
        // that are inliers.
        for(i = 0; i < n; i++)
        {
            point[0] = data[i][0];
            point[1] = data[i][1];

            if(fitModel_line(point, randModel, threshold))
            {
                //if(fabs(inner_point[1]) >= fabs(point[1]))
                //{
                //    inner_point[0] = point[0];
                //    inner_point[1] = point[1];
                //}
                conSet[inliers][0] = point[0];
                conSet[inliers][1] = point[1];
                inliers++;
            }
        }

        if(verbose)
            printf(" inliers = %d\n", inliers);

        // added code. Trying to bias the model selection to the ones most
        // likely to generate horizontal lines
        float bias = 0;
        float angCoeff = -randModel[0]/randModel[1];
        if(side == 0)
        {
            bias = 1.0 + 1.0*exp(- pow((angCoeff+0.15)/(0.20),2.0));
        }
        else
        {
            bias = 1.0 + 1.0*exp(- pow((angCoeff-0.15)/(0.20),2.0));
        }


        if(bias*inliers > *bestScore)  // Largest set of inliers.
        {
            if(verbose)
                printf(" >> IT'S THE BEST MODEL !!! <<\n");

            estimateModel_line(bestModel, conSet, inliers);
            //bestModel[2] =   0.40*(-inner_point[0]*bestModel[0] -inner_point[1]*bestModel[1])
            //               + 0.60*bestModel[2];
            *bestInliers = inliers;
            *bestScore = bias*inliers;

            if(verbose)
            {
                printf(" reestimated model: %.3f*x + %.3f*y + %.3f = 0\n",
                        bestModel[0], bestModel[1], bestModel[2]);
                printf(" inliers: %d\n", *bestInliers);
                printf(" bias: %f\n", bias);
                printf(" bias inliers: %f\n", *bestScore);
            }
        }

        Tcount++;
        if(Tcount > maxT)
            break;
    }

    if((int)*bestScore==0)
    {
        printf("\n### ERROR: ransac was unable to find a useful solution.\n");
        return(-1);
    }

    for(i = 0; nr < 2; i++){
        free(randSet[i]);
    }
    free(randSet);
    for(i = 0; i < n; i++){
        free(conSet[i]);
    }
    free(conSet);

    return(0);
}

int randomSelect(float **sel, int nsel, float **data, int ndata) {

    int r = 0;
    int k = ndata;
    int i;

    if(nsel > ndata)
    {
        printf("randomSelect: unable to select %d points from dataset[%d]\n", 
                nsel, ndata);
        return 0;
    }

    for(i = 0; i < nsel; i++, k--)
    {
        r = rand()%(k);
        //printf("%d\n",r);

        sel[i][0] = data[r][0];
        sel[i][1] = data[r][1];

        data[r][0] = data[k-1][0];
        data[r][1] = data[k-1][1];
        
        data[k-1][0] = sel[i][0];
        data[k-1][1] = sel[i][1];
    }

    //printf("k = %d\n", k);

    return 0;
}

int fitModel_line(float *point, float *l, float threshold) {
    // Estimate distance between point and model
    // d = abs(a*x + b*y + c)/sqrt(a^2 + b^2)

    float d=0;

    d = fabs(l[0]*point[0] + l[1]*point[1] + l[2])/sqrt(pow(l[0], 2) + pow(l[1], 2));

    if(d<=threshold)
        return 1;
    else
        return 0;
}

void estimateModel_line(float *l, float **P, int n) {
    if(n<=2) {
        perror("Need at least tree points\n");
        l[0] = 0;
        l[1] = 0;
        l[2] = 0;
        return;
    }
    
    float x_bar = 0.0;
    float y_bar = 0.0;
    for(int i=0; i!=n; ++i){
        x_bar += P[i][0];
        y_bar += P[i][1];
    }
    x_bar = x_bar/n;
    y_bar = y_bar/n;

    float c0 = 0.0;
    float c1 = 0.0;
    for(int i=0; i!=n; ++i){
        c0 +=  (P[i][0] - x_bar)*(P[i][1] - y_bar);
        c1 += -pow(P[i][0] - x_bar,2) + pow(P[i][1] - y_bar,2);

    // Line in normal vector representation
    float phi, r;
    phi = 0.5*atan2(-2*c0,c1);
    r = x_bar*cos(phi) + y_bar*sin(phi);

    // Line in tradicional representation
    float angCoeff, linCoeff;
    angCoeff = -cos(phi)/(sin(phi) + 1e-6);
    linCoeff = r/(sin(phi) + 1e-6);

    // Line in representation ax + by + c = 0
    l[0] = -angCoeff;
    l[1] = 1.0;
    l[2] = -linCoeff;
    }
}

void twoPointsLine(float *l, float **P){
    l[0] = 1;
    l[1] = -((P[0][0]-P[1][0])/(P[0][1]-P[1][1]));
    l[2] = -P[0][0] -l[1]*P[0][1];
}

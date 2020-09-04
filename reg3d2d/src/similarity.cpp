//	similarity.cpp	--> implementation of similarity.h

#include <math.h>
#include <limits>
#include <iostream>
#include "similarity.h"
using namespace std;

//////////////////////////////////////////////////////////////
Similarity::Similarity( void )
{
    Mode = 0;

    Height = 0;
    Width = 0;

    FactorR = 0;
    FactorG = 0;
    FactorB = 0;
}

Similarity::~Similarity( void )
{
    Mode = 0;

    Height = 0;
    Width = 0;

    FactorR = 0;
    FactorG = 0;
    FactorB = 0;
}

void Similarity::init( int mode )
{
    Mode = mode;
}

void Similarity::init( int height, int width )
{
    Height = height;
    Width = width;
}

void Similarity::init( float factorR, float factorG, float factorB )
{
    FactorR = factorR;
    FactorG = factorG;
    FactorB = factorB;
}

// template<class T>
// float Similarity::findsimilarity( T *image1, T *image2 )
// unsigned char range: 0 -255 
float Similarity::findsimilarity( unsigned char *image1, unsigned char *image2 )
{
    float similarity=0.0;
    switch( Mode )
    {
        case 1:
            similarity = usecorr( image1, image2 );
            break;
        case 2:
            similarity = usemutualInfo( image1, image2 );
            break;
        default:
            break;
    }

    return similarity;
}
// function to calculate cross correlation between two depth images
float Similarity::findsimilarityDepth( float *image1, float _minDepth, float _maxDepth,float *image2 )
{
    if( !Height || !Width )	//do not devide by zero
        return 0;

    float similarity = 0.0;
    float meanSqErr = 0;
    float mean1 = 0;
    float mean2 = 0;
//  int zNear = 1;
//  int zFar = 100;
//  float maxCTDepth = INT_MIN;
//  float minCTDepth = INT_MAX;
//   float maxVideoDepth = INT_MIN;
//   float minVideoDepth = INT_MAX;
    long i;
    float temp1;
    float temp2;
    for( i=0; i<Height*Width; i++ )
    {
        // image1 = SFS_depth
        // D_Video = (SFSImg-min(min(SFSImg)))*99/(max(max(SFSImg))-min(min(SFSImg)))+1;
        // temp1 = (image1[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;
//       maxVideoDepth = (temp1 > maxVideoDepth) ? temp1 : maxVideoDepth;
// 	  minVideoDepth = (temp1 < minVideoDepth) ? temp1 : minVideoDepth;
// 	  // image2 = z_buffer
// 	  //z_n = 2.0*z_b-1.0;
//       //z_e = 2.0* zNear*zFar./(zFar+zNear-z_n*(zFar-zNear));
//       temp2 = 2.0* zNear*zFar/(zFar+zNear-(2.0*image2[i]-1.0)*(zFar-zNear));
//       // D_CT = ((z_e<50).*z_e + (z_e>=50)*50)*2;
//       if (temp2<50)
//       	temp2 = temp2*2;
//       else
//       	temp2 = 100;
//      

        temp1 = image1[i];
        temp2 = image2[i];
//	  maxCTDepth = (temp2 > maxCTDepth) ? temp2 : maxCTDepth;
//	  minCTDepth = (temp2 < minCTDepth) ? temp2 : minCTDepth;
        meanSqErr += (temp1 - temp2) * (temp1 - temp2);
        mean1 += temp1;
        mean2 += temp2;

    }
    // cout << "maxVideoDepth=" << maxVideoDepth << ";" << "minVideoDepth="<< minVideoDepth << endl;
//  cout << "maxCTDepth=" << maxCTDepth << ";" << "minCTDepth="<< minCTDepth << endl;
    float size = (float)(Height) * (float)(Width);
    meanSqErr = (float)sqrt( meanSqErr / size );
    mean1 = mean1 / size;
    mean2 = mean2 / size;

    float dev1 = 0;
    float dev2 = 0;
    float corr = 0;
    for( i=0; i<Height*Width; i++ )
    {
        // temp1 = (image1[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;
//       maxVideoDepth = (temp1 > maxVideoDepth) ? temp1 : maxVideoDepth;
// 	  minVideoDepth = (temp1 < minVideoDepth) ? temp1 : minVideoDepth;
// 	  // image2 = z_buffer
// 	  //z_n = 2.0*z_b-1.0;
//       //z_e = 2.0* zNear*zFar./(zFar+zNear-z_n*(zFar-zNear));
//       temp2 = 2.0* zNear*zFar/(zFar+zNear-(2.0*image2[i]-1.0)*(zFar-zNear));
//       // D_CT = ((z_e<50).*z_e + (z_e>=50)*50)*2;
//       if (temp2<50)
//       	temp2 = temp2*2;
//       else
//       	temp2 = 100;
        temp1 = image1[i];
        temp2 = image2[i];
        dev1 += (temp1 - mean1) * (temp1 - mean1);
        dev2 += (temp2 - mean2) * (temp2 - mean2);
        corr += (temp1 - mean1) * (temp2 - mean2);
    }

    dev1 /= size;
    dev2 /= size;
    corr /= size;

    if( !dev1 || !dev2 )	//do not devide by zero
        return MAX;

    corr = corr / (float)sqrt( dev1 * dev2 );

    similarity = meanSqErr / corr;
    // cout << "dev1=" << dev1 << " dev2=" << dev2 << " corr=" << corr << endl;
    if (similarity <0)
        return MAX;
    return similarity;
    //return corr;
}

// function to calculate mutual information between two depth images
float Similarity::findsimilarityDepthMI( float *image1, float _minDepth, float _maxDepth,float *image2 )
{
    if( !Height || !Width )	//do not devide by zero
        return 0;
    cout << "height="<<Height << " width=" << Width << endl;
    int n;
    float p;
    long i;

    float similarity = MAX;
    float size = (float)(Height) * (float)(Width);
    float MI = 0;
    float Hx = 0;
    float Hy = 0;
    float Hxy = 0;
    // calculate entropy of each image
    int HistX[100] = {};
    int HistY[100] = {};
    int HxyMatrix[100*100] = {};
    int matrixSize = 101;

    // int zNear = 1;
//   int zFar = 100;
    // float maxCTDepth = INT_MIN;
//   float minCTDepth = INT_MAX;
//   float maxVideoDepth = INT_MIN;
//   float minVideoDepth = INT_MAX;
    int min12 = 1;
    int max12 = 100;
    float temp1;
    float temp2;

    // for( i=0; i<Height*Width; i++ )
//     {
//       // image1 = SFS_depth
//       // D_Video = (SFSImg-min(min(SFSImg)))*99/(max(max(SFSImg))-min(min(SFSImg)))+1;
//       temp1 = (image1[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;
//       maxVideoDepth = (temp1 > maxVideoDepth) ? temp1 : maxVideoDepth;
// 	  minVideoDepth = (temp1 < minVideoDepth) ? temp1 : minVideoDepth;
// 	  // image2 = z_buffer
// 	  //z_n = 2.0*z_b-1.0;
//       //z_e = 2.0* zNear*zFar./(zFar+zNear-z_n*(zFar-zNear));
//       temp2 = 2.0* zNear*zFar/(zFar+zNear-(2.0*image2[i]-1.0)*(zFar-zNear));
//       // D_CT = ((z_e<50).*z_e + (z_e>=50)*50)*2;
//       if (temp2<50)
//       	temp2 = temp2*2;
//       else
//       	temp2 = 100;
//      
// 	  maxCTDepth = (temp2 > maxCTDepth) ? temp2 : maxCTDepth;
// 	  minCTDepth = (temp2 < minCTDepth) ? temp2 : minCTDepth;
// 
//     }
//     if (minCTDepth < minVideoDepth)
//     	min12 = minCTDepth;
//     else
//     	min12 = minVideoDepth;
//     	
//     if (maxCTDepth > maxVideoDepth)
//     	max12 = maxCTDepth;
//     else
//     	max12 = maxVideoDepth;
//     	
//   cout << "maxVideoDepth=" << maxVideoDepth << ";" << "minVideoDepth="<< minVideoDepth << endl;
//   cout << "maxCTDepth=" << maxCTDepth << ";" << "minCTDepth="<< minCTDepth << endl;
//   cout << "max12=" << max12 << " min12=" << min12 << endl;
    cout << "max12=" << max12 << " min12=" << min12 << endl;
    matrixSize = max12-min12+1;
    cout << "matrixSize = " << matrixSize << endl;
    // calculate joint entropy of image1 and image2
    for( i=0; i<Height*Width; i++ )
    {
        // temp1 = (image1[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;
//       
//       temp2 = 2.0* zNear*zFar/(zFar+zNear-(2.0*image2[i]-1.0)*(zFar-zNear));
//       // D_CT = ((z_e<50).*z_e + (z_e>=50)*50)*2;
//       if (temp2<50)
//       	temp2 = temp2*2;
//       else
//       	temp2 = 100;
//       	
        temp1 = image1[i];
        temp2 = image2[i];
        // cout << "Temp1="<<temp1<<"Temp2="<<temp2<<endl;
// 			cout <<"((int)temp1-min12)*matrixSize+((int)temp2-min12)="<<((int)temp1-min12)*matrixSize+((int)temp2-min12) << endl;
//       		cout <<"(int)temp1-min12="<<(int)temp1-min12<<endl;
//       		cout <<"(int)temp2-min12="<<(int)temp2-min12<<endl;
        HxyMatrix[((int)temp1-min12)*matrixSize+((int)temp2-min12)]++;
        HistX[(int)temp1-min12]++;
        HistY[(int)temp2-min12]++;
        //cout << "i=" << i << endl;

    }

    for (i = 0; i <(matrixSize)*(matrixSize);i++) {
        p = (float)HxyMatrix[i]/size;
        Hxy = Hxy -log2(p+std::numeric_limits<float>::epsilon())*p;
        //cout << "i=" << i << endl;
    }

    for (int j = 0; j < matrixSize ; j++) {
        Hx = Hx-log2((float)HistX[j]/size+std::numeric_limits<float>::epsilon())*((float)HistX[j]/size);
        Hy = Hy-log2((float)HistY[j]/size+std::numeric_limits<float>::epsilon())*((float)HistY[j]/size);
    }
    MI = Hx+Hy-Hxy;
    similarity = 1/sqrt((MI/Hx)*(MI/Hy));
    cout << " in similarity.findsimilarityDepthMI" << endl;
    return similarity;
    //return corr;
}


template<class T>
float Similarity::usecorr( T *image1, T *image2 )
{
    //this function calculates a similarity measure
    //between image1 and image2 using correlation criteria: [MORI00]

    if( !Height || !Width )	//do not devide by zero
        return 0;

    float similarity;
    float meanSqErr = 0;
    float mean1 = 0;
    float mean2 = 0;
    long i;
    for( i=0; i<Height*Width; i+=4 )
    {
        float temp1 = (float)image1[4*i]*FactorR
                      + (float)image1[4*i+1]*FactorG
                      + (float)image1[4*i+2]*FactorB;

        float temp2 = (float)image2[4*i]*FactorR
                      + (float)image2[4*i+1]*FactorG
                      + (float)image2[4*i+2]*FactorB;

        meanSqErr += (temp1 - temp2) * (temp1 - temp2);
        mean1 += temp1;
        mean2 += temp2;

    }
    float size = (float)(Height) * (float)(Width);
    meanSqErr = (float)sqrt( meanSqErr / size );
    mean1 = mean1 / size;
    mean2 = mean2 / size;

    float dev1 = 0;
    float dev2 = 0;
    float corr = 0;
    for( i=0; i<Height*Width; i+=4 )
    {
        float temp1 = (float)image1[4*i]*FactorR
                      + (float)image1[4*i+1]*FactorG
                      + (float)image1[4*i+2]*FactorB;

        float temp2 = (float)image2[4*i]*FactorR
                      + (float)image2[4*i+1]*FactorG
                      + (float)image2[4*i+2]*FactorB;

        dev1 += (temp1 - mean1) * (temp1 - mean1);
        dev2 += (temp2 - mean2) * (temp2 - mean2);
        corr += (temp1 - mean1) * (temp2 - mean2);
    }

    dev1 /= size;
    dev2 /= size;
    corr /= size;

    if( !dev1 || !dev2 )	//do not devide by zero
        return MAX;

    corr = corr / (float)sqrt( dev1 * dev2 );

    similarity = meanSqErr / corr;

    return similarity;
}

template<class T>
float Similarity::usemutualInfo( T *image1, T *image2 )
{
    //this function calculates a similarity measure
    //between image1 and image2 using correlation criteria: [MORI00]

    if( !Height || !Width )	//do not devide by zero
        return 0;

    int n;
    float p;
    long i;

    float similarity;  // normalised mutual information
    float size = (float)(Height) * (float)(Width);
    float MI = 0;
    float Hx = 0;
    float Hy = 0;
    float Hxy = 0;
    int HxyMatrix[256*256] = {};
    // calculate entropy of each image
    int HistX[255] = {};
    int HistY[255] = {};
    int matrixSize = 256;
    int min12 = 255;
    int max12 = 0;

    // find the max and min of pixel intensity in both images to rescale
    for( i=0; i<Height*Width; i+=4 )
    {
        float temp1 = (float)image1[4*i]*FactorR
                      + (float)image1[4*i+1]*FactorG
                      + (float)image1[4*i+2]*FactorB;

        float temp2 = (float)image2[4*i]*FactorR
                      + (float)image2[4*i+1]*FactorG
                      + (float)image2[4*i+2]*FactorB;

        if ((int)temp1<min12)
            min12 = (int)temp1;
        if ((int)temp2<min12)
            min12 = (int)temp2;
        if ((int)temp1>max12)
            max12 = (int)temp1;
        if ((int)temp2>max12)
            max12 = (int)temp2;
    }
    cout << "max12=" << max12 << " min12=" << min12 << endl;
    matrixSize = max12-min12+1;
    cout << "matrixSize = " << matrixSize << endl;

    // calculate joint entropy of image1 and image2

    for( i=0; i<Height*Width; i+=4 )
    {
        float temp1 = (float)image1[4*i]*FactorR
                      + (float)image1[4*i+1]*FactorG
                      + (float)image1[4*i+2]*FactorB;

        float temp2 = (float)image2[4*i]*FactorR
                      + (float)image2[4*i+1]*FactorG
                      + (float)image2[4*i+2]*FactorB;

        HxyMatrix[((int)temp1-min12)*matrixSize+((int)temp2-min12)]++;
        HistX[(int)temp1-min12]++;
        HistY[(int)temp2-min12]++;

    }
    for (i = 0; i <(matrixSize)*(matrixSize);i++) {
        p = (float)HxyMatrix[i]/size;
        Hxy = Hxy -log2(p+std::numeric_limits<float>::epsilon())*p;
    }

    for (int j = 0; j < matrixSize ; j++) {
        Hx = Hx-log2((float)HistX[j]/size+std::numeric_limits<float>::epsilon())*((float)HistX[j]/size);
        Hy = Hy-log2((float)HistY[j]/size+std::numeric_limits<float>::epsilon())*((float)HistY[j]/size);
    }

    MI = Hx+Hy-Hxy;
    similarity = 1/sqrt((MI/Hx)*(MI/Hy));

    return similarity;
}


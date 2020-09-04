//#include <gandalf/image.h>
//#include <gandalf/vision/convolve1D.h>

#include <math.h>
#include "FL/fl_draw.h"
#include <iostream>
#include <fstream>
#include <vector>
using std::vector;

#include "glm.h"
#include "shapeshade.h"
#include "global.h"
#include "main.h"

#include "define_test.h"

using namespace std;

#define MAX 9000000

typedef vector<GLMpoint2d> vector_pq;
typedef vector<GLMpoint> vector_obj;
typedef vector<GLMpoint2d_d> vector_dpq;
typedef vector<GLMpoint_d> vector_dobj;
typedef vector<long> vector_long;
typedef vector<float> vector_float;

// function to calculate pq components for input z_buffer
// called in projection.cpp by: bool flag = pqmodel( scaled_buffer, pq_model, &inf_vector, mousecoord, info, focal, iwidth, near_clip, far_clip );
bool pqmodel( float *zbuf, vector<GLMpoint_d> *pq_space, vector<long> *inf_vector, int *mousecoord,
              float *info, float focal, int zbuf_width, float fnear, float ffar ,char* filename)
{
    //printf("In pqmodel(), shapeshade.cpp\n");

    //copy mouse coordinates to (x0,y0)->lower left corner, (x1,y1)->upper right corner
    int x0 = mousecoord[0];	int y0 = mousecoord[1];
    int x1 = mousecoord[2];	int y1 = mousecoord[3];
    int width = x1 - x0 +1;
    int height = y1 - y0 +1;

    double iwidth = (double)(info[0]);		double iheight = (double)(info[1]);
    double imgwidth = (double)(info[2]);	double imgheight = (double)(info[3]);
    //printf("x0=%d,y0=%d,x1=%d,y1=%d,width=%d,height=%d,iwidth=%f,iheight=%f,imgwidth=%f,imgheight=%f",x0,y0,x1,y1,width,height,iwidth,iheight,imgwidth,imgheight);
    bool flag = false;	// flag=false if every z-buffer pixel == 1 =>object is not in the FOV

#ifdef PRINT_MODE
    vector<GLMpoint_d> lets_test;
    GLMpoint_d lets_temp;
#endif
/////////////////////////////////// testing
    bool allone = true;
    int countZ = 0;
    for(int i=0;i<(x1*y1);i++)
    {
        if (zbuf[i]!=1) {
            allone=false;
            //cout <<zbuf[i]<<" ";
            countZ++;
        }
    }
    if (allone==true)
        cout << "zbuffer all one"<<endl;
//	  else
//	  cout << "zbuffer not all one "<<"have "<<countZ<<"non-one points, in pqmodel"<<endl;
    GLMpoint_d pq_point;
    long count = 0;
    double zdepth[9] = {0.0};
    for( int i=y0; i<=y1; i++ )
    {
        for( int j=x0; j<=x1; j++ )
        {
            zdepth[0] = (double)(zbuf[i*zbuf_width+j]);			//(i,j)
            zdepth[1] = (double)(zbuf[(i-1)*zbuf_width+(j-1)]);	//(i-1,j-1)
            zdepth[2] = (double)(zbuf[(i-1)*zbuf_width+j]);		//(i-1,j)
            zdepth[3] = (double)(zbuf[(i-1)*zbuf_width+(j+1)]);	//(i-1,j+1)
            zdepth[4] = (double)(zbuf[i*zbuf_width+(j-1)]);		//(i,j-1)
            zdepth[5] = (double)(zbuf[i*zbuf_width+(j+1)]);		//(i,j+1)
            zdepth[6] = (double)(zbuf[(i+1)*zbuf_width+(j-1)]);	//(i+1,j-1)
            zdepth[7] = (double)(zbuf[(i+1)*zbuf_width+j]);		//(i+1,j)
            zdepth[8] = (double)(zbuf[(i+1)*zbuf_width+(j+1)]);	//(i+1,j+1)

//			for( int k=0; k<9; k++ )
//				zdepth[k] = (double)(fnear) + zdepth[k]*(double)(ffar-fnear);

// differentiate zdepth wrt x and y
            double zx = (-zdepth[1]+zdepth[3]-2.0*zdepth[4]+2.0*zdepth[5]-zdepth[6]+zdepth[8]) / 8.0;
            double zy = (zdepth[1]+2.0*zdepth[2]+zdepth[3]-zdepth[6]-2.0*zdepth[7]-zdepth[8]) / 8.0;

#ifdef PRINT_MODE
            lets_temp.x = zx;
            lets_temp.y = zy;
            lets_temp.z = zdepth[0];
            lets_test.push_back( lets_temp );
#endif

            double z_plus = zdepth[0];

            double x = (double)(j)*iwidth/imgwidth-iwidth/2.0;
            double y = (double)(i)*iheight/imgheight-iheight/2.0;
            double con = iwidth / imgwidth;

            // scale p q values
            double p, q;
            if( x*zx+(fnear+z_plus)*con )
                p = (double)(fnear) * zx / (x*zx+((double)(fnear)+z_plus)*con);
            else p =0.0;

            if( y*zy+(fnear+z_plus)*con )
                q = (double)(fnear) * zy / (y*zy+((double)(fnear)+z_plus)*con);
            else
                q = 0.0;

            pq_point.x = p;
            pq_point.y = q;
            pq_point.z = 1.0;

            //if( zbuf[0] == 1.0 || (!p && !q) )
            if( zdepth[0] == 1.0 || (!p && !q) )
            {
                inf_vector->push_back( count );
                pq_point.x = 0.0;
                pq_point.y = 0.0;
                pq_point.z = 1.0;
                //printf("zbuff==1\n");
                //cout << zdepth[0]<<" "<<p <<" "<<q<<endl;
            }
            else {
                flag = true;
                //cout << zdepth[0]<<" "<<p <<" "<<q<<endl;
            }

            pq_space->push_back( pq_point );

            count++;
        }
    }

#ifdef PRINT_MODE
    int wid_heig[2] = { x1+2, y1+2};
    cout << "width" << wid_heig[0] << "height" << wid_heig[1] << endl;
    // store_2D_img( &lets_test, wid_heig, "/Users/malishen/Documents/reg3d2d/zbuffer_zx.txt",
// 								 	"/Users/malishen/Documents/reg3d2d/zbuffer_zy.txt",
// 									"/Users/malishen/Documents/reg3d2d/zbuffer_zbuffer.txt" );
// 		bool success = store_2D_img( &lets_test, wid_heig, 3,"/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-c/zbuffer_depth/frame_0000.txt");
    bool success = store_2D_img( &lets_test, wid_heig, 3,"/Users/malishen/Documents/MATLAB/TestDepthOpengl.txt");
    if (success)
        printf("zbuffer saved to file\n");
#endif


    return flag;
}

/* Convert input image to pq_space */
void pq_image( unsigned char *image, vector<GLMpoint_d> *pq_space, int *mousecoord,
               int width, int height, float *step, float focal_ratio )
{
    const long imgsize = width*height;
    float *greyimg = new float[imgsize];
    // covert RGB image to grayscale image
    greyscaleimg( image, greyimg, width, height );

    double scale = 1.0f;	//default

    int x0 = mousecoord[0];	int y0 = mousecoord[1];
    int x1 = mousecoord[2];	int y1 = mousecoord[3];
    int w = x1 - x0 +1;
    int h = y1 - y0 +1;

    if( w>width || h>height )
    {
        cout<<"pq_image:mouse coordinates does not match image dimension\n";
        return;
    }

    GLMpoint_d pq_point;
    for( int i=y0; i<=y1; i++ )
    {
        for( int j=x0; j<=x1; j++ )
        {
            double I1 = (double)(greyimg[i*width+j]);			//I(i,j)
            double I2 = (double)(greyimg[(i-1)*width+(j-1)]); 	//I(i-1,j-1)
            double I3 = (double)(greyimg[(i-1)*width+j]);		//I(i-1,j)
            double I4 = (double)(greyimg[(i-1)*width+(j+1)]);	//I(i-1,j+1)
            double I5 = (double)(greyimg[i*width+(j-1)]);		//I(i,j-1)
            double I6 = (double)(greyimg[i*width+(j+1)]);		//I(i,j+1)
            double I7 = (double)(greyimg[(i+1)*width+(j-1)]);	//I(i+1,j-1)
            double I8 = (double)(greyimg[(i+1)*width+j]);		//I(i+1,j)
            double I9 = (double)(greyimg[(i+1)*width+(j+1)]);	//I(i+1,j+1)

            double Rx, Ry;
            if( !I1 || !I2 || !I3 || !I4 || !I5 || !I6 || !I7 || !I8 || !I9 )
            {
                pq_point.x = 0.0f;
                pq_point.y = 0.0f;
                pq_point.z = 0.0f;
                pq_space->push_back( pq_point );
                continue;
            }

            Rx = ((double)(width) / 2.0)*(-I2-2.0*I5-I7+I4+2.0*I6+I9) / 8.0;
            Ry = ((double)(height) / 2.0)*(I2+2.0*I3+I4-I7-2.0*I8-I9) / 8.0;
            double E = (I1+I2+I3+I4+I5+I6+I7+I8+I9) / 9.0;
            if( !E || (!Rx && !Ry) )
            {
                pq_point.x = 0.0f;
                pq_point.y = 0.0f;
                pq_point.z = 0.0f;
                pq_space->push_back( pq_point );
                continue;
            }
            Rx /= E;
            Ry /= E;

            Rx *= scale;
            Ry *= scale;

            //scale pixel coordinates to view coordinates:
            double x = 2.0*(double)(j)/(double)(width) - 1.0;
            double y = 2.0*(double)(i)/(double)(height) - 1.0;

            double focal_len = focal_ratio;	//focal_ratio = (height/2) / focal_len -- Normalized height=2.0
            x /= focal_len;
            y /= focal_len;

            double temp = ( 1.0 + x*x + y*y );
            double A1 = (-x*Rx+3.0)*temp - 3.0*x*x;
            double B1 = -Rx*temp*y - 3.0*x*y;
            double C1 = Rx*temp + 3.0*x;
            double A2 = -Ry*temp*x - 3.0*x*y;
            double B2 = (-y*Ry+3.0)*temp - 3.0*y*y;
            double C2 = Ry*temp + 3.0*y;

            float norm = B1*A2-A1*B2;
            if( (norm > 0.01 || norm < -0.01) )
            {
                double p = (B2*C1-B1*C2);
                if( p )
                    p /= norm;
                double q = (-A2*C1+A1*C2);
                if( q )
                    q /= norm;

                pq_point.x = p;
                pq_point.y = q;
                pq_point.z = 1.0;

                pq_space->push_back( pq_point );
            }
            else
            {
                pq_point.x = 0.0;
                pq_point.y = 0.0;
                pq_point.z = 0.0;
                pq_space->push_back( pq_point );
            }
        }
    }
    delete [] greyimg;
}

void stereograph( vector<GLMpoint> *pq_orig, vector<GLMpoint> **pq_stereograph )
{
    long length = (pq_orig)->size();

    if( !length )
        return;

    *pq_stereograph = new vector_obj;
    GLMpoint orig, ster;
    vector_obj::iterator iter = pq_orig->begin();
    for( long i=0; i<length; i++ )
    {
        orig = iter[i];
        //calculate stereographic coordinates
        float w_ster = sqrt( orig.x*orig.x + orig.y*orig.y + 1.0 );
        float f_ster = 2.0*orig.x/(1.0+w_ster);
        float g_ster = 2.0*orig.y/(1.0+w_ster);

        //insert point to output vector: pq_stereograph
        ster.x = f_ster;
        ster.y = g_ster;
        ster.z = w_ster;

        (*pq_stereograph)->push_back( ster );
    }
}


void greyscaleimg( unsigned char *origimg, float *greyimg, int width, int height )
{
    const int siz = 4; //RGBA image
    const long imgsize = width * height;

    const float FactorR = (float)(1.0/3.0);
    const float FactorG = (float)(1.0/3.0);
    const float FactorB = (float)(1.0/3.0);

    for( int i=0; i<imgsize; i++ )
    {
        greyimg[i] = FactorR*(float)origimg[siz*i] +
                     FactorG*(float)origimg[siz*i+1] +
                     FactorB*(float)origimg[siz*i+2];
        greyimg[i] /= 255.0;	//normalize from 0 to 1
    }
}


void VectorRestoration( vector<GLMpoint2d> **pq_orig, int *imgcoord, int n, float w, float l, float Dt )
{
    //n -> number of iterations
    //w -> weight of neightbour points
    //l -> weight of the pixel under consideration
    //Dt-> the rate of changing the initial vector

    int width = imgcoord[2] - imgcoord[0] +1;
    int height = imgcoord[3] - imgcoord[1] +1;

    int size = (*pq_orig)->size();

    vector<GLMpoint2d> *pq_temp = new vector<GLMpoint2d>;
    vector<GLMpoint2d> *pq_res = new vector<GLMpoint2d>;

    //copy pq_orig to temp
    for( vector_pq::iterator iter_pq_or=(*pq_orig)->begin(); iter_pq_or!=(*pq_orig)->end(); iter_pq_or++ )
    {
        GLMpoint2d point = *iter_pq_or;
        pq_temp->push_back( point );
    }

    float temp;
    float tempx, tempy, temp1x, temp1y;
    float gx, gy, g;
    float utx, uty, u;
    GLMpoint2d point2d;
    for( int k=0; k<n; k++ )
    {
        for( int i=0; i<height; i++)
        {
            for( int j=0; j<width; j++ )
            {
                vector_pq::iterator iter_temp = (*pq_orig)->begin();
                point2d = iter_temp[i*width+j];

                if( !i || i==(height-1) || !j || (j==width-1) )
                {
                    pq_res->push_back( point2d );
                    continue;
                }

                tempx = point2d.x;
                tempy = point2d.y;

                iter_temp = pq_temp->begin();
                point2d = iter_temp[i*width+j];
                temp1x = point2d.x;
                temp1y = point2d.y;

                if( temp1x || temp1y )
                {
                    //add neighbours
                    point2d = iter_temp[(i-1)*width+j];	//(i-1,j)
                    gx = point2d.x;
                    gy = point2d.y;
                    point2d = iter_temp[(i+1)*width+j];	//(i+1,j)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;
                    point2d = iter_temp[i*width+j-1];	//(i,j-1)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;
                    point2d = iter_temp[i*width+j+1];	//(i,j+1)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;

                    point2d = iter_temp[(i-1)*width+j-1];	//(i-1,j-1)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;
                    point2d = iter_temp[(i-1)*width+j+1];	//(i-1,j+1)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;
                    point2d = iter_temp[(i+1)*width+j-1];	//(i+1,j-1)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;
                    point2d = iter_temp[(i+1)*width+j+1];	//(i+1,j+1)
                    temp = point2d.x;
                    gx = gx + temp;
                    temp = point2d.y;
                    gy = gy + temp;

                    gx = gx * w + tempx * l;
                    gy = gy * w + tempy * l;

                    g = sqrt( gx*gx + gy*gy );
                    //find the perpendicular vector onto u. (theta+90)
                    u = sqrt( temp1x*temp1x + temp1y*temp1y );
                    utx =  -temp1y / u;
                    uty = temp1x / u;

                    //if cos(theta+90-fi)<0 change direction
                    float coss;
                    coss = ( gy * temp1x - gx * temp1y ) / u;
                    if ( coss < 0 )
                    {
                        utx = - utx;
                        uty = - uty;
                    }

                    float Fb, Fbx, Fby;
                    //find projection
                    Fb = Myabs( coss );
                    Fbx = Fb * utx;
                    Fby = Fb * uty;

                    //find final
                    Fbx = temp1x + Dt * Fbx;
                    Fby = temp1y + Dt * Fby;
                    Fb = sqrt( Fbx * Fbx + Fby * Fby );
                    Fbx = ( Fbx / Fb ) * u;
                    Fby = ( Fby / Fb ) * u;

                    //store the result
                    point2d.x = Fbx;
                    point2d.y = Fby;
                    pq_res->push_back( point2d );
                }
                else
                {
                    point2d = iter_temp[i*width+j];
                    pq_res->push_back( point2d );
                }

            }
        }
        vector<GLMpoint2d> *pq_tempimg;
        pq_tempimg = pq_temp;
        pq_temp = pq_res;
        pq_res = pq_tempimg;
        pq_res->clear();

    }

    delete (*pq_orig);

    (*pq_orig) = pq_temp;

    delete pq_res;
}




bool PQtoFile( char *fname, vector<GLMpoint> *pq_space, int *imgcoord )
{
    ofstream fout(fname);
    if (!fout) return false;
    fout << imgcoord[0] << ' ' << imgcoord[1] << ' ' << imgcoord[2] << ' ' << imgcoord[3] << ' ' <<pq_space->size() << endl;
    for (vector_obj::iterator icl = pq_space->begin(); icl != pq_space->end(); icl++)
        fout << (*icl).x << ' ' << (*icl).y << endl;

    fout.close();
    return true;
}

bool PQfromFile( char *fname, vector<GLMpoint> *pq_space )
{
    ifstream fin(fname);
    if (!fin) return false;

    while( fin.good() )
    {
        GLMpoint point;
        fin >> point.x >> point.y;
        if( fin.good() )
            pq_space->push_back(point);
    }
    fin.close();

    return true;
}

bool FloatfromFile( char *fname, vector<float> *vector_fl )
{
    ifstream fin(fname);
    if (!fin) return false;

    while( fin.good() )
    {
        float point;
        fin >> point;
        if( fin.good() )
            vector_fl->push_back(point);
    }
    fin.close();

    return true;
}

bool DataToFile( char *fname, vector<long> *data_vect, int *imgcoord )
{
    ofstream fout(fname);
    if (!fout) return false;

    fout << imgcoord[0] << ' ' << imgcoord[1] << ' ' << imgcoord[2] << ' ' << imgcoord[3] << ' ' <<data_vect->size() << endl;
    for (vector_long::iterator icl = data_vect->begin(); icl != data_vect->end(); icl++)
        fout << (*icl)<< endl;

    return true;
}

void dotprod_testfun2( vector<GLMpoint> *pq_obj, vector<GLMpoint> *pq_img, int *coord, char *path, float scale )
{
    long obj_siz = pq_obj->size();
    long img_siz = pq_img->size();

    if( obj_siz != img_siz )
    {
        printf("dot_prod_testfun2: pq_obj and pq_img not compatible\n" );
        return;
    }

    float *anglef = new float[obj_siz];
    int width = coord[2] - coord[0] +1;
    int height = coord[3] - coord[1] +1;

    GLMpoint p_obj, p_img;
    vector_obj::iterator iter_obj = pq_obj->begin();
    vector_obj::iterator iter_img = pq_img->begin();
    for( long i=0; i<obj_siz; i++ )
    {
        p_obj = iter_obj[i];
        p_img = iter_img[i];

        float dot_product = Myabs(p_obj.x*p_img.x + p_obj.y*p_img.y);
        float norm = sqrt( (p_obj.x*p_obj.x+p_obj.y*p_obj.y) * (p_img.x*p_img.x+p_img.y*p_img.y) );
        if(norm)
            dot_product /= norm;
        else
            dot_product = 1;

        anglef[i] = dot_product;
    }

    //scale and store
    unsigned char *angleuc = new unsigned char[ 3*obj_siz ];

    float max = scale;
    float min = -scale;

    long count = 0;
    int i;
    for( i=0; i<obj_siz; i++ )
    {

        float p = anglef[i];
        if( p > max )
            max = p;
        if( p < min )
            min = p;

        if( p >= max )
            p = 255.0;
        else if( p <= min )
            p = 0.0;
        else
            p = 256.0 * (p-min) / (max-min);


        angleuc[3*count] = (unsigned char)p;
        angleuc[3*count+1] = (unsigned char)p;
        angleuc[3*count+2] = (unsigned char)p;
        count++;

    }

    glmWritePPM( path, angleuc, width, height );


    delete [] anglef;
    delete [] angleuc;
}

void test_fun1( int *mousecoord, vector<GLMpoint> *pq_space, char *pathp, char *pathq, float scale )
{
    int x0 = (float)mousecoord[0];
    int y0 = (float)mousecoord[1];
    int x1 = (float)mousecoord[2];
    int y1 = (float)mousecoord[3];
    int width = x1 - x0 +1;
    int height = y1 - y0 +1;

    float max_p = scale;
    float min_p = -scale;
    float max_q = scale;
    float min_q = -scale;

    int size = pq_space->size();
    unsigned char *img_p = new unsigned char[ 3*size ];
    unsigned char *img_q = new unsigned char[ 3*size ];

    long count = 0;
    for( vector_obj::iterator iter= pq_space->begin(); iter!=pq_space->end(); iter++ )
    {
        float p;
        float q;
        if( (*iter).x >= max_p )
            p = 255.0;
        else if( (*iter).x <= min_p )
            p = 0.0;
        else
            p = 256.0 * ((*iter).x-min_p) / (max_p-min_p);

        if( (*iter).y >= max_q )
            q = 255.0;
        else if( (*iter).y <= min_q )
            q = 0.0;
        else
            q = 256.0 * ((float)(*iter).y-min_q) / (max_q-min_q);

        img_p[3*count] = p;
        img_p[3*count+1] = p;
        img_p[3*count+2] = p;

        img_q[3*count] = q;
        img_q[3*count+1] = q;
        img_q[3*count+2] = q;

        count++;
    }

    glmWritePPM( pathp, img_p, width, height );
    glmWritePPM( pathq, img_q, width, height );

    delete [] img_p;
    delete [] img_q;

}

void test_fun1( int *mousecoord, vector<GLMpoint2d> *pq_space, char *pathp, char *pathq, float scale )
{
    int x0 = (float)mousecoord[0];
    int y0 = (float)mousecoord[1];
    int x1 = (float)mousecoord[2];
    int y1 = (float)mousecoord[3];
    int width = x1 - x0 +1;
    int height = y1 - y0 +1;

    float max_p = scale;
    float min_p = -scale;
    float max_q = scale;
    float min_q = -scale;

    int size = pq_space->size();
    unsigned char *img_p = new unsigned char[ 3*size ];
    unsigned char *img_q = new unsigned char[ 3*size ];

    long count = 0;
    for( vector_pq::iterator iter= pq_space->begin(); iter!=pq_space->end(); iter++ )
    {
        float p;
        float q;
        if( (*iter).x >= max_p )
            p = 255.0;
        else if( (*iter).x <= min_p )
            p = 0.0;
        else
            p = 256.0 * ((*iter).x-min_p) / (max_p-min_p);

        if( (*iter).y >= max_q )
            q = 255.0;
        else if( (*iter).y <= min_q )
            q = 0.0;
        else
            q = 256.0 * ((float)(*iter).y-min_q) / (max_q-min_q);

        img_p[3*count] = p;
        img_p[3*count+1] = p;
        img_p[3*count+2] = p;

        img_q[3*count] = q;
        img_q[3*count+1] = q;
        img_q[3*count+2] = q;

        count++;
    }

    glmWritePPM( pathp, img_p, width, height );
    glmWritePPM( pathq, img_q, width, height );

    delete [] img_p;
    delete [] img_q;

}

void test_fun2( int *mousecoord, vector<GLMpoint2d> *pq_space, char *pathp )
{
    int x0 = (float)mousecoord[0];
    int y0 = (float)mousecoord[1];
    int x1 = (float)mousecoord[2];
    int y1 = (float)mousecoord[3];
    int width = x1 - x0 +1;
    int height = y1 - y0 +1;

    float max_mag = -MAX;
    float min_mag = MAX;

    int size = pq_space->size();
    unsigned char *img_mag = new unsigned char[ 3*size ];


    for( vector_pq::iterator it=pq_space->begin(); it!=pq_space->end(); it++ )
    {
        float p = sqrt( (*it).x*(*it).x + (*it).y*(*it).y );

        if( p > max_mag )
            max_mag = p;
        if( p < min_mag )
            min_mag = p;
    }

    long count = 0;
    for( vector_pq::iterator iter= pq_space->begin(); iter!=pq_space->end(); iter++ )
    {
        float p = sqrt( (*iter).x*(*iter).x + (*iter).y*(*iter).y );

        if( p >= max_mag )
            p = 255.0;
        else if( p <= min_mag )
            p = 0.0;
        else
            p = 255.0 * (p-min_mag) / (max_mag-min_mag);


        img_mag[3*count] = p;
        img_mag[3*count+1] = p;
        img_mag[3*count+2] = p;

        count++;
    }

    glmWritePPM( pathp, img_mag, width, height );

    delete [] img_mag;

}

void test_fun3( float *image, char *path, int iwidth, int iheight, float scale )
{
    float max = scale;
    float min = 0.0;

    int size = iwidth*iheight;
    unsigned char *img = new unsigned char[ 3*size ];

    for( long i=0; i<size; i++ )
    {
        float p;
        if( image[i] >= max )
            p = 255.0;
        else if( image[i] <= min )
            p = 0.0;
        else
            p = 256.0 * (image[i]-min) / (max-min);


        img[3*i] = p;
        img[3*i+1] = p;
        img[3*i+2] = p;
    }

    glmWritePPM( path, img, iwidth, iheight );

    delete [] img;
}
void test_fun4( float *image, char *path, int iwidth, int iheight, float min, float max )
{
//	float max = scale;
//	float min = 0.0;

    int size = iwidth*iheight;
    unsigned char *img = new unsigned char[ 3*size ];

    for( long i=0; i<size; i++ )
    {
        float p;
        if( image[i] >= max )
            p = 255.0;
        else if( image[i] <= min )
            p = 0.0;
        else
            p = 256.0 * (image[i]-min) / (max-min);


        img[3*i] = p;
        img[3*i+1] = p;
        img[3*i+2] = p;
    }

    glmWritePPM( path, img, iwidth, iheight );

    delete [] img;
}

/*
void prepareSmooth( vector<GLMpoint2d> *pq_space, int *imgcoord, float sigma )
{
	int width = imgcoord[2] - imgcoord[0] +1;
	int height = imgcoord[3] - imgcoord[1] +1;
	long size = width * height;

	float *image_p = new float[size];
	float *image_q = new float[size];
	GLMpoint2d point;
	long count=0;
	for ( vector_pq::iterator iter=pq_space->begin(); iter!=pq_space->end(); iter++ )
	{
		point = (*iter);

		image_p[count] = point.x;
		image_q[count] = point.y;
		count++;
	}
	GaussSmoothImg( image_p, width, height, sigma );
	GaussSmoothImg( image_q, width, height, sigma );

	count = 0;
	for ( iter=pq_space->begin(); iter!=pq_space->end(); iter++ )
	{
		point.x = image_p[count];
		point.y = image_q[count];
		(*iter) = point;
		count++;
	}


	delete [] image_p;
	delete [] image_q;
}
*/

/*
void prepareSmooth_mag( vector<GLMpoint2d> *pq_space, int *imgcoord, float sigma )
{
	int width = imgcoord[2] - imgcoord[0] +1;
	int height = imgcoord[3] - imgcoord[1] +1;
	long size = width * height;

	float *image_mag = new float[size];
	GLMpoint2d point;
	long count=0;
	vector_pq::iterator iter_rand = pq_space->begin();
	for ( vector_pq::iterator iter=pq_space->begin(); iter!=pq_space->end(); iter++ )
	{
		point = iter_rand[count];

		if( Myabs(point.x)>1.0 || Myabs(point.y)>1.0 )
			point.x = point.x;

		image_mag[count] = sqrt( point.x*point.x + point.y*point.y );
		count++;
	}
	GaussSmoothImg( image_mag, width, height, sigma );

	count = 0;
	for ( iter=pq_space->begin(); iter!=pq_space->end(); iter++ )
	{
		point = iter_rand[count];
		if( Myabs(point.x)>1.0 || Myabs(point.y)>1.0 )
			point.x = point.x;

		float norm = sqrt( point.x*point.x + point.y*point.y );
		if( norm>0.0001 )
		{
			point.x = point.x * image_mag[count] / norm;
			point.y = point.y * image_mag[count] / norm;
		}
		else if( count )
		{
			point = iter_rand[count-1];
		}
		else
		{
			point.x = 0;
			point.y = 0;
		}

		iter_rand[count] = point;
		count++;
	}

	delete [] image_mag;
}
*/

/*
void GaussSmoothImg( float *image, int iwidth, int iheight, float sigma)
{
	Gan_Mask1D *pMask;
	// create symmetric 1D convolution mask 
	pMask = gan_gauss_mask_new ( GAN_FLOAT,
                                   sigma,	// standard deviation of Gaussian 
                                   9,		// size of mask 
                                   1.0,		// scaling of values 
                                   NULL );

	Gan_Image pOriginalImage; // declare original image 
    Gan_Image *pXSmoothedImage = NULL; // declare image smoothed in x-direction 
    Gan_Image *pXYSmoothedImage = NULL; // declare image smoothed in x & y directions 

    gan_image_form_data_gl_f( &pOriginalImage, iheight, iwidth, iwidth*sizeof(float),
								 image, iheight*iwidth, NULL, 0 );

    
	//apply smoothing in the x direction 
    pXSmoothedImage = gan_image_convolve1Dx_s ( &pOriginalImage, GAN_ALL_CHANNELS,
                                                 pMask );

    // apply smoothing in the y direction 
    pXYSmoothedImage = gan_image_convolve1Dy_s ( pXSmoothedImage, GAN_ALL_CHANNELS,
                                                  pMask );

	int wid = pXYSmoothedImage->width;
	int hei = pXYSmoothedImage->height;

	for( int i=4; i<hei+4; i++ )
	{
		for( int j=4; j<wid+4; j++ )
		{
			
			image[(i)*iwidth+j] = gan_image_get_pix_gl_f( pXYSmoothedImage, i-4, j-4 );
		}
	}


	unsigned char *test = new unsigned char[3*iheight*iwidth];
	for( i=0; i<iheight*iwidth; i++ )
	{
		test[3*i] = 255.0*image[i];
		test[3*i+1] = 255.0*image[i];
		test[3*i+2] = 255.0*image[i];
	}
	glmWritePPM("C:\\doNTdelete\\project\\data\\output\\smooth.ppm", test, iwidth, iheight );
	delete [] test;


	//free convolution mask;
	gan_mask1D_free ( pMask );
	//free images
	gan_image_free( &pOriginalImage );
	gan_image_free( pXSmoothedImage );
	gan_image_free( pXYSmoothedImage );
}
*/

//////	shape from shading - deal with deformation ///////////
float find_deform( vector<GLMpoint> *pq_space1, vector<GLMpoint> *pq_space2, vector<GLMpoint2d> *pq_deform)
{
    long sp1_siz = pq_space1->size();
    long sp2_siz = pq_space2->size();

    if( sp1_siz != sp2_siz || !sp1_siz )
    {
        printf("find_deform: sp1_siz and sp2_siz not compatible\n" );
        return 0.0;
    }

    vector<float> vect_temp;

    float dot_product;
    float mean_angle = 0;
    vector_obj::iterator pq_iter_sp1 = pq_space1->begin();
    vector_obj::iterator pq_iter_sp2 = pq_space2->begin();
    for( long i=0; i<sp1_siz; i++ )
    {
        GLMpoint temp1 = pq_iter_sp1[i];
        GLMpoint temp2 = pq_iter_sp2[i];

        //normalize pq_vectors
        float temp = sqrt( temp1.x*temp1.x + temp1.y*temp1.y + 1.0 );
        temp1.x /= temp;	temp1.y /= -temp; temp1.z /= temp;

        temp2.x /= 100.0; temp2.y /= 100.0;
        temp = sqrt( temp2.x*temp2.x + temp2.y*temp2.y + 1.0 );
        temp2.x /= temp;	temp2.y /= -temp; temp2.z /= temp;

        dot_product = Myabs(temp1.x*temp2.x + temp1.y*temp2.y + temp1.z*temp2.z);
        float norm = sqrt( ( temp1.x*temp1.x+temp1.y*temp1.y + temp1.z*temp1.z) *
                           ( temp2.x*temp2.x+temp2.y*temp2.y + temp2.z*temp2.z ) );

        if( norm )
            dot_product /= norm;
        else
            dot_product = 1;

        vect_temp.push_back( dot_product );

        mean_angle += dot_product;
    }
    mean_angle /= sp1_siz;

    for( vector_float::iterator temp_iter=vect_temp.begin(); temp_iter!=vect_temp.end(); temp_iter++ )
    {
        dot_product = *temp_iter;
        GLMpoint2d deform;
        deform.x = dot_product;

        float deviation = sqrt( (mean_angle-dot_product)*(mean_angle-dot_product) );

        deform.y = deviation;

        pq_deform->push_back( deform );
    }

    return mean_angle;
}

float sim_cross_corr( vector<GLMpoint> *pq_obj, vector<GLMpoint> *pq_img, vector<long> *inf_vector)
{
    long sp1_siz = pq_obj->size();
    long sp2_siz = pq_img->size();

    if( sp1_siz != sp2_siz || !sp1_siz )
    {
        printf("sim_cross_correlation: sp1_siz and sp2_siz not compatible\n" );
        return 0.0;
    }

    vector_obj::iterator sp1_iter = pq_obj->begin();
    vector_obj::iterator sp2_iter = pq_img->begin();

    vector<float> *sp1_angle = new vector<float>;
    vector<float> *sp2_angle = new vector<float>;

    float mean_square_error = 0.0;
    //calculate cos(theta) = abs(p)/sqrt(p*p+q*q)
    GLMpoint sp1, sp2;
    for( long i=0; i<sp1_siz; i++ )
    {
        sp1 = sp1_iter[i];
        sp2 = sp2_iter[i];

        float temp = sqrt( sp1.x*sp1.x + sp1.y*sp1.y );
        float sp1_angle_tmp;
        if (temp )
            sp1_angle_tmp = Myabs( sp1.x ) / temp;
        else
            sp1_angle_tmp = 0.0;

        temp = sqrt( sp2.x*sp2.x +sp2.y*sp2.y );
        float sp2_angle_tmp;
        if( temp )
            sp2_angle_tmp = Myabs( sp2.x ) / temp;
        else
            sp2_angle_tmp = 0.0;

        sp1_angle->push_back( sp1_angle_tmp );
        sp2_angle->push_back( sp2_angle_tmp );
    }

    //use cross correlation
    float similarity = cross_correlation( sp1_angle, sp2_angle, inf_vector );
    if( similarity )
        similarity = 1.0 / similarity;
    else
        similarity = MAX;

    //de-allocate memory
    delete sp1_angle;
    delete sp2_angle;

    return similarity;
}

float cross_correlation( vector<float> *image_obj, vector<float> *image_img, vector<long> *inf_vector )
{
    long obj_siz = image_obj->size();
    long img_siz = image_img->size();

    if( obj_siz != img_siz || !obj_siz )
    {
        printf("cross_correlation: sp1_siz and sp2_siz not compatible\n" );
        return 0.0;
    }


    vector_float::iterator obj_iter = image_obj->begin();
    vector_float::iterator img_iter = image_img->begin();

    float meanSqErr = 0;
    float mean_obj = 0;
    float mean_img = 0;
    for( long i=0; i<obj_siz; i++ )
    {
        float temp_obj = obj_iter[i];
        float temp_img = img_iter[i];

        meanSqErr += (temp_obj-temp_img)*(temp_obj-temp_img);
        mean_obj += temp_obj;
        mean_img += temp_img;
    }
    meanSqErr /= obj_siz;
    mean_obj /= obj_siz;
    mean_img /= obj_siz;

    long inf_siz = inf_vector->size();
    vector_long::iterator inf_iter = inf_vector->begin();
    long inf_count;
    if( inf_siz )
        inf_count = inf_iter[0];
    long index = 0;

    float dev_obj = 0;
    float dev_img = 0;
    float corr = 0;
    int i;
    for( i=0; i<obj_siz; i++ )
    {
        float temp_obj = obj_iter[i];
        float temp_img = img_iter[i];

        dev_obj += (temp_obj-mean_obj)*(temp_obj-mean_obj);
        dev_img += (temp_img-mean_img)*(temp_img-mean_img);

        if( i == inf_count && inf_siz )
        {
            index++;
            inf_count = inf_iter[index];
        }
        else
            corr += (temp_obj-mean_obj)*(temp_img-mean_img);
    }
    dev_obj /= obj_siz;
    dev_img /= obj_siz;
    corr /= obj_siz;

    float measure;

    float temp = sqrt( dev_obj*dev_img );
    if( temp )
        measure = Myabs(corr) / temp;
    else
        measure = MAX;

    return measure;
}

void pq_scale( vector<GLMpoint_d> *pq_target, vector<GLMpoint_d> *pq_new, double *min_, double *max_ )
{
    long pq_size = pq_target->size();
    if( !pq_size ){
        printf( "pq_scale: pq_space is empty\n" );
        return;
    }

    const double scl1 = -5.0;
    const double scl2 = 5.0;

    vector_dobj::iterator pq_iter = pq_target->begin();

    GLMpoint_d point;
    point = pq_iter[0];
    double max = point.x;
    double min = point.y;
    for( long i=0; i<pq_size; i++ )
    {
        point = pq_iter[i];

        if( max<point.x )
            max = point.x;
        if( min> point.x )
            min = point.x;

        if( max<point.y )
            max = point.y;
        if( min> point.y )
            min = point.y;
    }

    *min_ = min;
    *max_ = max;
    if( max == min )
    {
        printf( "pq_scale: max=min\n" );
        return;
    }

    max = 0.9;
    min = -0.9;
    GLMpoint_d point_new;
    int i;
    for( i=0; i<pq_size; i++ )
    {
        point = pq_iter[i];

        double temp = sqrt( point.x*point.x + point.y*point.y + 1.0 );
        point.x /= temp; point.y /= -temp;

        if( point.x < min )
            point.x = scl1;
        if( point.x > max )
            point.x = scl2;
        if( point.y < min )
            point.y = scl1;
        if( point.y > max )
            point.y = scl2;
        point_new.x = scl1+(point.x-min)*(scl2-scl1)/(max-min);
        point_new.y = scl1+(point.y-min)*(scl2-scl1)/(max-min);
        point_new.z = point.z;

        pq_new->push_back( point_new );
    }
}


double sim_rigid( vector<GLMpoint_d> *pq_obj, vector<GLMpoint_d> *pq_img, vector<long> *inf_vector )
{
    long sp1_siz = pq_obj->size();
    long sp2_siz = pq_img->size();

    if( sp1_siz != sp2_siz || !sp1_siz )
    {
        printf("sim_rigid: sp1_siz and sp2_siz not compatible\n" );
        return 0.0;
    }

    //scale z-buffer
/*	vector<GLMpoint_d> *pq_objn = new vector<GLMpoint_d>;
	double min_obj, max_obj;
	pq_scale( pq_obj, pq_objn, &min_obj, &max_obj );
	vector_dobj::iterator spw_iter = pq_objn->begin();
*/
    vector_dobj::iterator sp1_iter = pq_obj->begin();
    vector_dobj::iterator sp2_iter = pq_img->begin();

    double mean_angle = 0.0;

    long count = 0;
    long inf_siz = inf_vector->size();
    vector_long::iterator inf_iter = inf_vector->begin();
    long inf_count;
    if( inf_siz )
        inf_count = inf_iter[0];

    long index = 0;

    for( long i=0; i<sp1_siz; i++ )
    {
        GLMpoint_d sp1_p = sp1_iter[i];
        GLMpoint_d sp2_p = sp2_iter[i];

        //normalize pq_vectors
        sp1_p.x *= 100.0;	sp1_p.y *= 100.0;
        double temp = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y + 1.0 );
        sp1_p.x /= temp;	sp1_p.y /= -temp;

        temp = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y + 1.0 );
        sp2_p.x /= temp;	sp2_p.y /= -temp;

        double dot_product = Myabs( sp1_p.x*sp2_p.x + sp1_p.y*sp2_p.y );
//		GLMpoint_d sp_w = spw_iter[i];
//		double weight_zbuf = sqrt( sp_w.x*sp_w.x + sp_w.y*sp_w.y );
        double weight_zbuf = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y );
//		double weight_zbuf = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y );
        double norm = sqrt( ( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y )*( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y ) );

        if( norm )
            dot_product /= norm;
        else
            dot_product = 1.0;

        if( weight_zbuf )
            dot_product *= weight_zbuf;

        if( count == inf_count && inf_siz )
        {
            dot_product = 0.0;
            index++;
            inf_count = inf_iter[index];
        }

        mean_angle += dot_product;

        count++;
    }
    mean_angle /= sp1_siz;

    double deviation = 0.0;
    count = 0;
    index = 0;
    if( inf_siz )
        inf_count = inf_iter[0];
    int i;
    for( i=0; i<sp1_siz; i++ )
    {
        GLMpoint_d sp1_p = sp1_iter[i];
        GLMpoint_d sp2_p = sp2_iter[i];

        //normalize pq_vectors
        sp1_p.x *= 100.0;	sp1_p.y *= 100.0;
        double temp = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y + 1.0 );
        sp1_p.x /= temp;	sp1_p.y /= -temp;

        temp = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y + 1.0 );
        sp2_p.x /= temp;	sp2_p.y /= -temp;

        double dot_product = Myabs( sp1_p.x*sp2_p.x + sp1_p.y*sp2_p.y );
//		GLMpoint_d sp_w = spw_iter[i];
//		double weight_zbuf = sqrt( sp_w.x*sp_w.x + sp_w.y*sp_w.y );
        double weight_zbuf = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y );
//		double weight_zbuf = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y );
        double norm = sqrt( ( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y )*( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y ) );

        if(norm)
            dot_product /= norm;
        else
            dot_product = 1.0;

        dot_product = (1.0-dot_product);

        if( weight_zbuf )
            dot_product *= weight_zbuf;
        else
            weight_zbuf = 1.0;

        if( count == inf_count  && inf_siz )
        {
            dot_product = 0.0;
            index++;
            inf_count = inf_iter[index];
        }

        deviation += Myabs(dot_product-mean_angle) * weight_zbuf;

    }

//	delete pq_objn;

    deviation /= sp1_siz;

    deviation *= mean_angle;

    if( deviation )
        return 10.0/deviation;
    else
        return MAX;
}


float sim_deform( vector<GLMpoint> *pq_obj, vector<GLMpoint> *pq_img,
                  vector<GLMpoint2d> *pq_deform, vector<long> *inf_vector )
{
    float thres = 0.003;
    long sp1_siz = pq_obj->size();
    long sp2_siz = pq_img->size();
    long def_siz = pq_deform->size();

    if( sp1_siz != sp2_siz || (def_siz != sp1_siz && def_siz) || !sp1_siz )
    {
        printf("sim_deform: sp1_siz and sp2_siz not compatible\n" );
        return 0.0;
    }

    vector_obj::iterator sp1_iter = pq_obj->begin();
    vector_obj::iterator sp2_iter = pq_img->begin();
    vector_pq::iterator def_iter = pq_deform->begin();

    float mean_angle = 0.0;

    long count = 0;
    long inf_siz = inf_vector->size();
    vector_long::iterator inf_iter = inf_vector->begin();
    long inf_count;
    if( inf_siz )
        inf_count = inf_iter[0];
    long index = 0;
    for( long i=0; i<sp1_siz; i++ )
    {
        GLMpoint sp1_p = sp1_iter[i];
        GLMpoint sp2_p = sp2_iter[i];

        //normalize pq_vectors
        sp1_p.x *= 100.0;	sp1_p.y *= 100.0;
        float temp = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y + 1.0 );
        sp1_p.x /= temp;	sp1_p.y /= -temp;

        temp = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y + 1.0 );
        sp2_p.x /= temp;	sp2_p.y /= -temp;


        GLMpoint2d def_p;
        if( def_siz )
            def_p = def_iter[i];

        float dot_product = Myabs( sp1_p.x*sp2_p.x + sp1_p.y*sp2_p.y );
        float weight_zbuf = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y );
        float norm = sqrt( ( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y )*( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y ) );

        if( norm )
            dot_product /= norm;
        else
            dot_product = 1.0;

        dot_product *= weight_zbuf;


        //deal with deformation
        if( def_p.y>thres && def_siz )
            dot_product /= def_p.y*100.0;

        if( count == inf_count && inf_siz )
        {
            dot_product = 0;
            index++;
            inf_count = inf_iter[index];
        }

        mean_angle += dot_product;
        count++;
    }
    mean_angle /= sp1_siz;

    float deviation = 0;
    count = 0;
    index = 0;
    if( inf_siz )
        inf_count = inf_iter[0];
    for( int i=0; i<sp1_siz; i++ )
    {
        GLMpoint sp1_p = sp1_iter[i];
        GLMpoint sp2_p = sp2_iter[i];

        //normalize pq_vectors
        sp1_p.x *= 100.0;	sp1_p.y *= 100.0;
        float temp = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y + 1.0 );
        sp1_p.x /= temp;	sp1_p.y /= -temp;

        temp = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y + 1.0 );
        sp2_p.x /= temp;	sp2_p.y /= -temp;

        GLMpoint2d def_p;
        if( def_siz )
            def_p = def_iter[i];

        float dot_product = Myabs( sp1_p.x*sp2_p.x + sp1_p.y*sp2_p.y );
        float weight_zbuf = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y );
        float norm = sqrt( ( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y )*( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y ) );

        if(norm)
            dot_product /= norm;
        else
            dot_product = 1.0;

        dot_product = (1.0-dot_product);

        dot_product *= weight_zbuf;

        //deal with deformation
        if( def_p.y>thres && def_siz )
            dot_product /= def_p.y*100.0;
        else
            def_p.y = 1.0;

        if( count == inf_count  && inf_siz )
        {
            dot_product = 0;
            index++;
            inf_count = inf_iter[index];
        }

        deviation += sqrt((dot_product-mean_angle)*(dot_product-mean_angle)) * weight_zbuf /(def_p.y*10.0);

    }

    deviation /= sp1_siz;

    deviation *= mean_angle;

    if( deviation )
        return 10.0/deviation;
    else
        return MAX;
}


float sim_deform_OF( vector<GLMpoint_d> *pq_obj, vector<GLMpoint_d> *pq_img,
                     vector<float> *deform, vector<long> *inf_vector )
{
    float thres = 0.001;
    long sp1_siz = pq_obj->size();
    long sp2_siz = pq_img->size();
    long def_siz = deform->size();

    if( sp1_siz != sp2_siz || (def_siz != sp1_siz && def_siz) || !sp1_siz )
    {
        printf("sim_deform: sp1_siz and sp2_siz not compatible\n" );
        return 0.0;
    }

    vector_dobj::iterator sp1_iter = pq_obj->begin();
    vector_dobj::iterator sp2_iter = pq_img->begin();
    vector_float::iterator def_iter = deform->begin();

    float mean_angle = 0.0;

    long count = 0;
    long inf_siz = inf_vector->size();
    vector_long::iterator inf_iter = inf_vector->begin();
    long inf_count;
    if( inf_siz )
        inf_count = inf_iter[0];
    long index = 0;
    for( long i=0; i<sp1_siz; i++ )
    {
        GLMpoint_d sp1_p = sp1_iter[i];
        GLMpoint_d sp2_p = sp2_iter[i];

        //normalize pq_vectors
        sp1_p.x *= 100.0;	sp1_p.y *= 100.0;
        float temp = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y + 1.0 );
        sp1_p.x /= temp;	sp1_p.y /= -temp;

        temp = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y + 1.0 );
        sp2_p.x /= temp;	sp2_p.y /= -temp;

        float def_p;
        if( def_siz )
            def_p = def_iter[i];

        float dot_product = Myabs( sp1_p.x*sp2_p.x + sp1_p.y*sp2_p.y );
        float weight_zbuf = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y );
        float norm = sqrt( ( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y )*( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y ) );

        if( norm )
            dot_product /= norm;
        else
            dot_product = 1.0;

        dot_product *= weight_zbuf;

        //deal with deformation
        if( def_p>thres && def_siz )
            dot_product *= (1.0f-def_p);

        if( count == inf_count && inf_siz )
        {
            dot_product = 0;
            index++;
            inf_count = inf_iter[index];
        }

        mean_angle += dot_product;
        count++;
    }
    mean_angle /= sp1_siz;

    float deviation = 0;
    count = 0;
    index = 0;
    if( inf_siz )
        inf_count = inf_iter[0];
    for( int i=0; i<sp1_siz; i++ )
    {
        GLMpoint_d sp1_p = sp1_iter[i];
        GLMpoint_d sp2_p = sp2_iter[i];

        //normalize pq_vectors
        sp1_p.x *= 100.0;	sp1_p.y *= 100.0;
        float temp = sqrt( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y + 1.0 );
        sp1_p.x /= temp;	sp1_p.y /= -temp;

        temp = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y + 1.0 );
        sp2_p.x /= temp;	sp2_p.y /= -temp;

        float def_p;
        if( def_siz )
            def_p = def_iter[i];

        float dot_product = Myabs( sp1_p.x*sp2_p.x + sp1_p.y*sp2_p.y );
        float weight_zbuf = sqrt( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y );
        float norm = sqrt( ( sp1_p.x*sp1_p.x + sp1_p.y*sp1_p.y )*( sp2_p.x*sp2_p.x + sp2_p.y*sp2_p.y ) );

        if(norm)
            dot_product /= norm;
        else
            dot_product = 1.0;

        dot_product = (1.0-dot_product);

        dot_product *= weight_zbuf;

        //deal with deformation
        float def;
        if( def_p>thres && def_siz )
        {
            def = 1.0f-def_p;
            dot_product *= def;
        }
        else
            def = 1.0;

        if( count == inf_count  && inf_siz )
        {
            dot_product = 0.0;
            index++;
            inf_count = inf_iter[index];
        }

        deviation += sqrt((dot_product-mean_angle)*(dot_product-mean_angle)) * weight_zbuf * def;

    }

    deviation /= sp1_siz;

    deviation *= mean_angle;

    if( deviation )
        return 10.0/deviation;
    else
        return MAX;
}

/* save float image to ppm , normalise grayscale image to 0-255 */
void store_float_image( vector<float> *image_float, int *imgcoord, char *path )
{
    int width = imgcoord[2] - imgcoord[0] +1;
    int height = imgcoord[3] - imgcoord[1] +1;
    long img_size = width * height;

    long size = image_float->size();

    if( img_size != size )
    {
        printf("store_float_image: vector size not compatible with image width-height\n" );
        return;
    }

    vector_float::iterator iter = image_float->begin();
    float max = -MAX;
    float min = MAX;
    for( long i=0; i<size; i++ )
    {
        float temp = iter[i];
        if( temp > max )
            max = temp;
        if( temp < min )
            min = temp;
    }

    unsigned char *img = new unsigned char[3*size];
    for( int i=0; i<size; i++ )
    {
        float temp = iter[i];

        if( temp >= max )
            temp = 255.0;
        else if( temp <= min )
            temp = 0.0;
        else
            temp = 255.0 * (temp-min) / (max-min);


        img[3*i] = temp;
        img[3*i+1] = temp;
        img[3*i+2] = temp;
    }

    glmWritePPM( path, img, width, height );

    delete [] img;
}

/* save float image to ppm, normalise, input: image, width, height, path */
void store_float_image( vector<float> *image_float, int width, int height, char *path )
{
    // int width = imgcoord[2] - imgcoord[0] +1;
    // int height = imgcoord[3] - imgcoord[1] +1;
    long img_size = width * height;

    long size = image_float->size();

    if( img_size != size )
    {
        printf("store_float_image: vector size not compatible with image width-height\n" );
        return;
    }

    vector_float::iterator iter = image_float->begin();
    float max = -MAX;
    float min = MAX;
    for( long i=0; i<size; i++ )
    {
        float temp = iter[i];
        if( temp > max )
            max = temp;
        if( temp < min )
            min = temp;
    }

    unsigned char *img = new unsigned char[3*size];
    for( int i=0; i<size; i++ )
    {
        float temp = iter[i];

        if( temp >= max )
            temp = 255.0;
        else if( temp <= min )
            temp = 0.0;
        else
            temp = 255.0 * (temp-min) / (max-min);


        img[3*i] = temp;
        img[3*i+1] = temp;
        img[3*i+2] = temp;
    }

    glmWritePPM( path, img, width, height );

    delete [] img;
}

///////////////////////////////////////////////////////////////////////////////////////////////
bool store_2D_img( vector<GLMpoint_d> *pq_space, int *wid_heig, char *path1, char *path2 )
{
    int width = wid_heig[0]-2;
    int height = wid_heig[1]-2;
    long img_size = width * height;

    long size = pq_space->size();

    if( img_size != size ){
        printf("store_2D_img: vector size not compatible with image width-height.\n" );
        return false;
    }

    ofstream fout1(path1);
    if (!fout1) return false;

    ofstream fout2(path2);
    if (!fout2) return false;

    vector_dobj::iterator iter = pq_space->begin();
    for( int j=0; j<height; j++ )
    {
        for( int i=0; i<width; i++ )
        {
            GLMpoint_d temp = iter[j*width+i];
            fout1<< temp.x<<" ";
            fout2<< temp.y<<" ";
        }
        fout1<<endl;
        fout2<<endl;
    }
    fout1.close();
    fout2.close();

    printf("width=%d, height=%d",width,height);
    return true;
}

bool store_2D_img( vector<GLMpoint_d> *pq_space, int *wid_heig, char *path1, char *path2, char *path3 )
{
    int width = wid_heig[0]-2;
    int height = wid_heig[1]-2;
    long img_size = width * height;

    long size = pq_space->size();

    if( img_size != size ){
        printf("store_2D_img: vector size not compatible with image width-height.\n" );
        return false;
    }

    ofstream fout1(path1);
    if (!fout1) return false;

    ofstream fout2(path2);
    if (!fout2) return false;

    ofstream fout3(path3);
    if (!fout3) return false;

    vector_dobj::iterator iter = pq_space->begin();
    for( int j=0; j<height; j++ )
    {
        for( int i=0; i<width; i++ )
        {
            GLMpoint_d temp = iter[j*width+i];
            fout1<< temp.x<<" ";
            fout2<< temp.y<<" ";
            fout3<< temp.z<<" ";
        }
        fout1<<endl;
        fout2<<endl;
        fout3<<endl;
    }
    fout1.close();
    fout2.close();
    fout3.close();

    return true;
}

bool store_2D_img( vector<GLMpoint_d> *pq_space, int *wid_heig, int choose, char *path )
{
    int width = wid_heig[0]-2;
    int height = wid_heig[1]-2;
    long img_size = width * height;

    long size = pq_space->size();

    if( img_size != size ){
        printf("store_2D_img: vector size not compatible with image width-height.\n" );
        return false;
    }

    if( choose!=1 && choose!=2 && choose!=3 )
    {
        printf("store_2D_img: choose should be 1 or 2 or 3.\n" );
        return false;
    }

    ofstream fout1(path);
    if (!fout1) return false;

    vector_dobj::iterator iter = pq_space->begin();
    for( int j=0; j<height; j++ )
    {
        for( int i=0; i<width; i++ )
        {
            GLMpoint_d temp = iter[j*width+i];
            if( choose == 1 )
                fout1<< temp.x<<" ";
            else if( choose == 2 )
                fout1<< temp.y<<" ";
            else
                //fout1<< temp.z<<" ";
                fout1<< temp.z<<endl;
        }
        fout1<<endl;
    }
    fout1.close();

    return true;
}



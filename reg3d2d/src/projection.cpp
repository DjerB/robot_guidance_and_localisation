/*
projection.c


Tool for teaching about OpenGL projections.

*/


#include <math.h>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
//#include <vector>
#include <string>
//using std::vector;

#include "main.h"
#include "glm.h"
#include "registration.h"
#include "similarity.h"
//#include "edge_features.h"
#include "praxis.h"
#include "shapeshade.h"
#include "projection.h"
#include "deformation.h"

#include "define_test.h"
#include "nrutil.h"
#include "powell.h"
#include "linmin.h"
#include <time.h>
//using namespace std;
/*
//debug
#include <malloc.h>

/////////
#define CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
//debug
*/

static char GlobalfnamePPM[200];
static char GlobalPoseFile[200];
static char GlobalfnameDepth[200];
static int GlobalNoOfFrames;
static int GlobalFrameNo;
static int RegistrationMode;
static bool RecordTime = true;
static bool saveTrack = true;
//static GLfloat centreline[661*3];
static GLMmodel** pmodel = NULL;

static GLfloat view_x = 0.0, view_y = 0.0, view_z = 2.0;
static GLfloat spin_xso = 0.0;
static GLfloat spin_yso = 0.0;
static GLfloat spin_zso = 0.0;

static GLfloat bounds[6];

//PPM image overlay
static int iheight, iwidth;
static unsigned char* image = NULL;
static unsigned char* image2 = NULL;
static float* depthImage = NULL;
float _maxDepth; // raw max value of video depth
float _minDepth; // raw min value of video depth

static GLfloat zoomFactorx = 1.0;
static GLfloat zoomFactory = 1.0;
static unsigned char trans = 40;

//silhouette detection parameters
static int edgstrlen = 2;
//static float edgoffset = 0;

//camera parameters
static float aspect = 1.0;
static float focal = 1.2;
//static float near_clip = 1.0;
// static float far_clip = 100.0;  // in vivo
//static float far_clip = 200.0;  // phantom

// phantom symposium 2018
//static float near_clip = 3.0;
//static float far_clip = 300.0;
// in vivo case 1
static float near_clip = 1.0;
static float far_clip = 200.0;

void set_nearfar_clip( float h, float y ) {
    near_clip = h;
    far_clip = y;
}

//image grab
static int oglwidth = 0;
static int oglheight = 0;
static unsigned char* view_image = 0;

//bool variables control edge and silhouette display
static bool disp_silh = false;
static bool display_edges = true;

//lighting conditions
static int illumType = 1;	//{0->LAmbient, 1->LDiffuse, 2->LSpecular, 3->MAmbient, 4->MDiffuse, 5->MSpecular} 
static float RGBAcolor[24] = {0.0, 0.0, 0.0, 1.0,	//RGBA - LAmbient
                              1.0, 1.0, 1.0, 1.0,	//RGBA - LDiffuse
                              1.0, 1.0, 1.0, 1.0,	//RGBA - LSpecular
                              0.2, 0.2, 0.2, 1.0,	//RGBA - MAmbient
                              0.8, 0.8, 0.8, 1.0,	//RGBA - MDiffuse
                              0.0, 0.0, 0.0, 1.0 };	//RGBA - MSpecular

static float spotangle = 180.0;	//GL_SPOT_CUTOFF
static float spotexponent = 0.0; //GL_SPOT_EXPONENT
static float shininess = 0.0; //GL_SHININESS
static float att_0 = 1.0;	//GL_CONSTANT_ATTENUATION
static float att_1 = 0.0;	//GL_LINEAR_ATTENUATION
static float att_2 = 0.0;	//GL_QUADRATIC_ATTENUATION

//mouse coordinates: select area [w0,h0,w1,h1]
static int mouse[4] = {0, 0 ,0 ,0};

//shape from shade
static bool shshade = false;
static vector<GLMpoint_d> *pq_model = NULL;
static vector<GLMpoint_d> *pq_img = NULL;

//read zbuffer
static float *zbuffer = NULL;
static bool zbuffer_read = false;

//registration of video frames
static Reg_vid_fram_in_out RegVidPar;
typedef vector<float*> vector_float_point;
static vector<float*> *reg_res_vect = NULL;
//store 2D images from the 3D model
static StoreFrame Store3DFrames;
//deformation
static bool deformation = false;
static vector<GLMpoint_d> *pq_img_bef = NULL;
static float def_threshold = 0.0;
static 	vector<GLMpoint2d_d> deform_vector;
//draw sphere
static float radius = 0.1;
static float sphere_centre[3] = { 0.0, 0.0, 0.0};
static float deform_amount = 0.0;
static bool draw_sphere = false;

//get camera transformation
static GLdouble modelviewmatrix[16];
static GLdouble projectionmatrix[16];
static GLint viewport[4];
static bool GetTransf = false;

//initialize pmodel
void init_models( int n ) {
    pmodel = new GLMmodel*[n];
    for( int i = 0; i<n; i++ ) pmodel[i] = 0;
}

/*void load_centreline(char* fname) {
int Type = 1;
std::ifstream fin(fname);
if( !fin )
//return Type;
exit(1);

//read header
char header[30];
fin.getline( header, 30);
cout << header<<endl;
//cout << header[0]<<" "<<header[1]<<" "<<header[2]<<" "<<header[3]<<" "<<header[4]<<" "<<header[5]<<" "<<header[6]<<" "<<header[7]<<" "<<header[8]<<" "<<header[9]<<" "<<header[10]<<" "<<endl;
// if( !strcmp( header, "NOFF" ) )	{
// 		Type = 1;
// 		fprintf(stderr,"bad NOFF header: %s\n",header);
// 		}
// 	else if( !strcmp( header, "NOFF - STRIPS" ) )
// 		Type = 2;
// 	else{
// 		//display error - file format does not agree with the specifications
// 		fprintf(stderr,"bad NOFF header: %s\n",header);
// 		fin.close();
// 		return Type;
// 	}

if( Type == 1 )		//NOFF: read triangles
{
// //read number of vertexes, number of triangles, edges
// 		fin >> NumVertexes >> NumPolygons >> NumEdges;
// 
// 		//allocate memory - initialize
// 		TDGInit();
// 
// 		//read vertexes
// 		std::cout<<"reading "<<NumVertexes<<" vertices..."<<std::endl;
for( int i=0; i<661; i++ )
{
float temp[3];
//cout << temp[0]<<" "<<temp[1]<<" "<<temp[2]<<" ";
fin >> temp[0] >> temp[1] >> temp[2];
// interception, voxel dimensions(already scaled in .txt file), -x, -y z
// skeleton.txt generated in matlab
centreline[3*i] = -(temp[0]-142.75); 
centreline[3*i+1] = -(temp[1]-248.25); 
centreline[3*i+2] = temp[2]+805.5; 
//cout << temp[0]<<" "<<temp[1]<<" "<<temp[2]<<" ";
}

}

fin.close();
}
*/
void load_model( int mx, char* fname ) {
// read in vertices and triangles
    pmodel[mx] = glmReadOBJ(fname);
    if (!pmodel[mx]) exit(0);
// calculate max and min of x,y,z, extra padding to z
    glmBound(pmodel[mx], bounds);
// set camera view point to the middle of bound for x and y, set view_z to minz
    view_x = 0.5 * (bounds[0] + bounds[1]);
    view_y = 0.5 * (bounds[2] + bounds[3]);
    view_z = bounds[4];
//cout << "before glmFacetNormals"<<endl;
    glmFacetNormals( pmodel[mx] );
    glmVertexNormals( pmodel[mx], 90.0 );
// need to write glmWriteNOFF(), tdgeometry object was declared as local. 
// create a local tdgeometry object from glm or write directly from glm
//TDGWriteNOFF( char * path )
//glmWriteNOFF(pmodel[mx],"/Users/aapple/Documents/linuxCode/reg3d2d/D97193NOFF.noff" );
}

// void delete_models() {
// 	if (pmodel != NULL) {
// 	glmDelete(pmodel[0]);
// 	glmDelete(pmodel[1]);
// 	 cout << "pmodel deleted" << endl;
// 	}
// }
/******** image overlay operators ********/

void load_image( char* fname){
    if (image) {
        delete[] image;
        delete[] image2;
    }
// iwidth and iheight are only declared not initialised.
    image = glmReadPPM( fname, &iwidth, &iheight);
    if (!image) exit(0);
// camera parameter ???
    aspect = iwidth/float(iheight);
    printf("iwidth=%d, iheight=%d\n",iwidth,iheight);
//copy image to image2
    image2 = new unsigned char[iwidth*iheight*4];
    for( int i = 0; i<iwidth*iheight*4; i++ )
        image2[i] = image[i];

//clean pq_space calculated from the previous image
    if( pq_img )
    {
        vector<GLMpoint_d> *temp;
        temp = pq_img_bef;
        pq_img_bef = pq_img;
        if( temp )
            delete temp;
        pq_img = NULL;

        if( !deformation )
        {
            delete pq_img_bef;
            pq_img_bef = NULL;
        }
    }

}

void save_pose(void) {
    char fname[200]; // filename depth ppm
    snprintf(fname, sizeof(fname), "/Users/malishen/Desktop/Invivo1/pose/%d.pose", GlobalFrameNo);
//    char* fname = "/Users/malishen/Documents/reg3d2d/D97193_1186_1237_pose/1235newpose.pose";
    ofstream fout(fname);
    if (!fout) return;

    fout<< view_x<<"\n";
    fout<< view_y<<"\n";
    fout<< view_z<<"\n";
    fout<< spin_xso<<"\n";
    fout<< spin_yso<<"\n";
    fout<< spin_zso<<"\n";

    fout.close();
    zbuffer_read = true;
    grab_model_image(1);
    if( zbuffer_read )
    {
        int x0 = 0; int y0 = 0;
        int x1 = oglwidth-1; int y1 = oglheight-1;

        int tw = x1 - x0+1; int th = y1 - y0+1;
        if( zbuffer ) {
//            cout << "trying to delete zbuffer" << endl;
            delete [] zbuffer;
        }
        zbuffer = new float[tw*th];
// update zbuffer !!!!
//printf("glReadPixels() to update zbuffer\n");
//printf("x0=%d, y0=%d, tw=%d, th=%d",x0,y0,tw,th);
        glReadPixels( x0, y0, tw, th, GL_DEPTH_COMPONENT, GL_FLOAT, zbuffer );
//        zbuffer_read = false;
        bool allone = true;
        int count = 0;
        for(int i=0;i<(tw*th);i++)
        {
            if (zbuffer[i]>0.1){
                std::cout << zbuffer[i] << std::endl;
            }
            if (zbuffer[i]!=1) {
                allone=false;
//cout <<zbuffer[i]<<" ";
                count++;
            }
        }
        if (allone==true)
            cout << "zbuffer all one"<<endl;
//        else
//            cout << "zbuffer not all one "<<"have "<<count<<"non-one points"<<endl;

    }
    if( !zbuffer ) // input to ShapeFromShading()
    {
        printf( "ShapeFromShading: zbuffer has not been initialized\n" );
    }
    else {
        // scaled_buffer is a float image
        float *scaled_buffer = new float[iheight * iwidth];
        bool have_del_z = false;
        if (oglwidth != iwidth || oglheight != iheight)
            gluScaleImage(GL_DEPTH_COMPONENT, oglwidth, oglheight, GL_FLOAT,
                          zbuffer, iwidth, iheight, GL_FLOAT, scaled_buffer);
        else {
            delete[] scaled_buffer;
            scaled_buffer = zbuffer;
            have_del_z = true;
        }
        printf("Updating z-buffer\n");
        zbuffer2TrueDepth(near_clip, far_clip, scaled_buffer);
        // output CT depth map as PPM image
        char depthfilenameppm[200]; // filename depth ppm
        snprintf(depthfilenameppm, sizeof(depthfilenameppm), "/Users/malishen/Desktop/Invivo1/CTDepth/depthppm_%d.ppm",
                 GlobalFrameNo);
//        test_fun4(scaled_buffer, depthfilenameppm, iwidth, iheight, near_clip, far_clip);// rescale between near_clip and far_clip
        test_fun4(scaled_buffer, depthfilenameppm, iwidth, iheight, 0.0, far_clip);// rescale between 0 and far_clip

        // output Video depth map as PPM image
        char Vdepthfilenameppm[200]; // filename depth ppm
        snprintf(Vdepthfilenameppm, sizeof(Vdepthfilenameppm),
                 "/Users/malishen/Desktop/Invivo1/VideoDepth/depthVppm_%d.ppm", GlobalFrameNo);
//        test_fun4(depthImage, Vdepthfilenameppm, iwidth, iheight, near_clip, far_clip); // rescale between near_clip and far_clip
        test_fun4(depthImage, Vdepthfilenameppm, iwidth, iheight, 0.0, far_clip); // rescale between 0 and far_clip
        cout << "min video depth=" << _minDepth << ", max video depth=" << _maxDepth << std::endl;
    }
}
void load_pose(char* fname) {
    std::ifstream fin(fname);
    if( !fin )
        return;

    float x, y, z,rx, ry, rz;
//read in pose parameters
    fin >> x >> y >> z >> rx >> ry >> rz;
    set_pose_values(x,y,z,rx,ry,rz);
//generateProjections(x,y,z,rx,ry,rz);
}
void delete_image() {
    if (image) {
        delete[] image;
        delete[] image2;
//cout << "image and image2 deleted" << endl;
    }
    if (depthImage) {
        delete [] depthImage;
//cout << "depthImage deleted" << endl;
    }
}

//////////////////////
void readInRegistrationFileNames(void) {
    bool error = true;
    cout << "  Please enter the file name containing a series of poses:\n";

    cin.getline ( GlobalPoseFile, sizeof ( GlobalPoseFile) );

    error = cin.eof();

    if ( error )
    {	cout << "cant open pose file" << endl;
        return;
    }

    cout << GlobalPoseFile << " read in" << endl;

    cout << "  Please enter the first ppm file name:\n";

    cin.getline ( GlobalfnamePPM, sizeof (GlobalfnamePPM ) );

    error = cin.eof();

    if ( error )
    {	cout << "cant open PPM file" << endl;
        return;
    }

    cout << "  Please enter the first depth file name:\n";

    cin.getline ( GlobalfnameDepth, sizeof (GlobalfnameDepth ) );

    error = cin.eof();

    if ( error )
    {	cout << "cant open depth file" << endl;
        return;
    }

    cout << "Enter the number of poses" << endl;
    cin >> GlobalNoOfFrames;
    cout << GlobalNoOfFrames;

    cout << "Enter the starting frame number" << endl;
    cin >> GlobalFrameNo;
//    GlobalFrameNo = 1;
//    cout << GlobalfnamePPM << "read in" << endl;

}
void displayRegistrationResults(void) {
// double fArray[36];
//     int i=0;
//     FILE *fptr;
// 
//     fptr = fopen ("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/trans_track/tracker_horn.txt", "r");
// 
//     if (fptr == NULL)
// 
//         printf ("Can't Find File to open\n");
// 
//     else
//     {
//         for (i=0; i <= 36; i++)
//             {
//             if (fscanf(fptr, "%lf", &fArray[i]) == EOF) break;  /* stop if we ran out of values */
//             printf ("%le\n", fArray[i]);  /* use le since you're printing a double not a float */
//             }
//     }
    float x ,y ,z,rx,ry,rz;

    int FileLength = findFileLengthPPM(GlobalfnamePPM);
    int FileLengthDepth = findFileLengthTXT(GlobalfnameDepth);
    ifstream myfile;
    myfile.open(GlobalPoseFile);
    if((GlobalNoOfFrames-GlobalFrameNo)>0) {
        for (int i = 1; i <=GlobalFrameNo; i++) {
            myfile >> x >> y >> z >> rx >> ry >> rz;
        }
        cout << "Frame="<< GlobalFrameNo << "; Pose = " << x << y << z << rx << ry << rz << endl;
        cout << GlobalfnamePPM << endl;
        cout << GlobalfnameDepth << endl;
        load_image(GlobalfnamePPM);
        loadDepthFile(GlobalfnameDepth,true);
//        char poseFilename[200]; // filename depth ppm
//        snprintf(poseFilename, sizeof(poseFilename), "/Users/malishen/Desktop/phantomWrong/pose/%d.pose",
//                 GlobalFrameNo);
//        load_pose(poseFilename);
        set_pose_values(view_x,view_y,view_z,spin_xso,spin_yso,spin_zso);

//        set_pose_values(x,y,z,rx,ry,rz);

//next30Fname(GlobalfnamePPM,FileLength);
        nextFname(GlobalfnamePPM,FileLength);
        nextFname(GlobalfnameDepth,FileLengthDepth);
        GlobalFrameNo++;
    }
    else {
        cout << "The end of pose file reached!!!!" << endl;
    }

    myfile.close();
}
void nextFname(char* file_in_name,int InputFileLength) {
    if (file_in_name[InputFileLength]=='9'){
        cout << "1st reached 9" << endl;
        file_in_name[InputFileLength]='0';

        if (file_in_name[InputFileLength-1]=='9'){
            cout << "2nd reached 9" << endl;
            file_in_name[InputFileLength-1]='0';

            if (file_in_name[InputFileLength-2]=='9'){

                file_in_name[InputFileLength-2]='0';

                if (file_in_name[InputFileLength-3]=='9'){
                    file_in_name[InputFileLength-4]++;
                    file_in_name[InputFileLength-3]='0';

                }
                else {
                    file_in_name[InputFileLength-3]++;

                }
            }
            else {
                cout << "reached 100" << endl;
                file_in_name[InputFileLength-2]++;

            }
        }
        else {
            file_in_name[InputFileLength-1]++;

        }
    }
    else {
        file_in_name[InputFileLength]++;

    }
}

void next30Fname(char* file_in_name,int InputFileLength) {
    for (int i = 1; i <2 ; i++) {
        if (file_in_name[InputFileLength-1]=='9'){
            cout << "2nd reached 9" << endl;
            file_in_name[InputFileLength-1]='0';

            if (file_in_name[InputFileLength-2]=='9'){

                file_in_name[InputFileLength-2]='0';

                if (file_in_name[InputFileLength-3]=='9'){
                    file_in_name[InputFileLength-4]++;
                    file_in_name[InputFileLength-3]='0';

                }
                else {
                    file_in_name[InputFileLength-3]++;

                }
            }
            else {
                cout << "reached 100" << endl;
                file_in_name[InputFileLength-2]++;

            }
        }
        else {
            file_in_name[InputFileLength-1]++;

        }
    }
}
int findFileLengthPPM(char* file_in_name) {
    int pointer = 0;
    int InputFileLength = 0;
    while(file_in_name[pointer]!='.'||file_in_name[pointer+1]!='p'||file_in_name[pointer+2]!='p'||file_in_name[pointer+3]!='m') {
        pointer++;
    }
    while (!isdigit(file_in_name[pointer])) {
//   	cout << file_in_name[pointer]<<endl;
        pointer--;
    }
    InputFileLength = pointer;
//   cout << InputFileLength << endl;
//   cout << file_in_name[pointer]<<endl;
    return InputFileLength;
}

int findFileLengthTXT(char* file_in_name) {
    int pointer = 0;
    int InputFileLength = 0;
    while(file_in_name[pointer]!='.'||file_in_name[pointer+1]!='t'||file_in_name[pointer+2]!='x'||file_in_name[pointer+3]!='t') {
        pointer++;
    }
    while (!isdigit(file_in_name[pointer])) {
        cout << file_in_name[pointer]<<endl;
        pointer--;
    }
    InputFileLength = pointer;
    cout << InputFileLength << endl;
    cout << file_in_name[pointer]<<endl;
    return InputFileLength;
}


float func(float p[])
{
// float sum = p[1]*p[1]+p[2]*p[2]+p[3]*p[3]+p[4]*p[4]+p[5]*p[5]+p[6]*p[6];
// 	return sum;
    float IntenTemp = MAX;
    set_pose_values(p[1],p[2],p[3],p[4],p[5],p[6]);
    if (RegistrationMode==0)
        IntenTemp = eval_similarity(0)/10;   // Intensity

    if (RegistrationMode==3)
        IntenTemp = eval_similarity(3)/10;   // PQ

    if (RegistrationMode==4)
        IntenTemp = eval_similarity(4);   // Depth


    return IntenTemp;
}

void MinimisationPowell(void) {
    int iter;
    float ftol, *p, **xi, fret;
    int nGlobal = 6;
    p=fvector(1,nGlobal);
    xi = matrix(1,nGlobal,1,nGlobal);
    p[1]=view_x;p[2]=view_y;p[3]=view_z;p[4]=spin_xso;p[5]=spin_yso;p[6]=spin_zso;
    xi[1][1]=1;xi[2][1]=0;xi[3][1]=0;xi[4][1]=0;xi[5][1]=0;xi[6][1]=0;
    xi[1][2]=0;xi[2][2]=1;xi[3][2]=0;xi[4][2]=0;xi[5][2]=0;xi[6][2]=0;
    xi[1][3]=0;xi[2][3]=0;xi[3][3]=1;xi[4][3]=0;xi[5][3]=0;xi[6][3]=0;
    xi[1][4]=0;xi[2][4]=0;xi[3][4]=0;xi[4][4]=1;xi[5][4]=0;xi[6][4]=0;
    xi[1][5]=0;xi[2][5]=0;xi[3][5]=0;xi[4][5]=0;xi[5][5]=1;xi[6][5]=0;
    xi[1][6]=0;xi[2][6]=0;xi[3][6]=0;xi[4][6]=0;xi[5][6]=0;xi[6][6]=1;


//ftol = 0.0000000001;
    ftol = 0.01;
    fret = func(p);
//	printf("p=[%f,%f,%f,%f,%f,%f], xi=[%f,%f;%f,%f], iter=%d, fret=%f\n",p[1],p[2],p[3],p[4],p[5],p[6],xi[1][1],xi[1][2],xi[2][1],xi[2][2],iter,fret);
    powell(p, xi, nGlobal, ftol, &iter, &fret,func);
    set_pose_values(p[1],p[2],p[3],p[4],p[5],p[6]);
//	printf("p=[%f,%f,%f,%f,%f,%f], xi=[%f,%f;%f,%f], iter=%d, fret=%f\n",p[1],p[2],p[3],p[4],p[5],p[6],xi[1][1],xi[1][2],xi[2][1],xi[2][2],iter,fret);
    free_vector(p,1,nGlobal);
    free_matrix(xi,1,nGlobal,1,nGlobal);
}

// void generateProjectionsIntensity(void) {
// 
// 	char fname[200];
// 	bool error;
// 	cout << "  Please enter the first ppm file name:\n";
// 
//       cin.getline ( fname, sizeof ( fname ) );
// 
//       error = cin.eof();
// 
//       if ( error )
// 	{
// 	  return;
// 	}
// 	
// 	
//    ofstream outFile;         // Step #2 - Declare outstream file.
//    outFile.open("/Users/malishen/Desktop/IntensityPosesCase3MICCAIWithRecog.txt");        // Step #3 - Open outFile.
//  if (outFile.fail())          // Check for file creation and return error.
//    {
//       cout << "Error opening \"outFile.txt\" for output.\n";
// 
//       exit(1);
//    }
//    // ofstream TimeRecord;
// //    TimeRecord.open("/Users/malishen/Desktop/IntensityPosesInVivo429_every1_1000calibratedTime2.txt");
// //     if (TimeRecord.fail())          // Check for file creation and return error.
// //    {
// //       cout << "Error opening \"TimeRecord.txt\" for output.\n";
// // 
// //       exit(1);
// //    }
//    
// 	int FileLength = findFileLengthPPM(fname);
// 	double IntensityScore;
// 	double IntenTemp;
// 	float x ;
// 	float y ;
// 	float z ;
// 	float rx ;
// 	float ry ;
// 	float rz ;
// 	
// 	float Anglestep=2;
// 	float Diststep = 1;
// 	//float step=2.5;
// 	
// 	float Intensitypose[6];
// 	// IntensityScore = eval_similarity(0);
// 	// 	IntenTemp = IntensityScore;
// 	int frameNo = 1;
// 	
// 	/* Timing */
// 	clock_t t;
// 
// 	while (frameNo < 120) {
//      t = clock();
// 	load_image(fname);
// 	IntensityScore = INT_MAX;
// 	IntenTemp = INT_MAX;
// 	if (frameNo ==85) {
// 		set_pose_values(-305.92,-327.09,299.17,-23.62,101.68,143.01);
// 	}
// 	x = view_x;
// 	y = view_y;
// 	z = view_z;
// 	rx = spin_xso;
// 	ry = spin_yso;
// 	rz = spin_zso;
// 	
// 	
// 	Intensitypose[0] = x;
// 	Intensitypose[1] = y;
// 	Intensitypose[2] = z;
// 	Intensitypose[3] = rx;
// 	Intensitypose[4] = ry;
// 	Intensitypose[5] = rz;
// 	
// 	for (int xi = -1; xi < 2; xi++ ) {
// 		for (int yi = -1; yi < 2; yi++ ) {
// 			for (int zi = -1; zi < 2; zi++ ) {
// 				for (int rxi = -1; rxi < 2; rxi++ ) {
// 					for (int ryi = -1; ryi < 2; ryi++ ) {
// 						for (int rzi = -1; rzi < 2; rzi++ ) {
// 	// for (int xi = -2; xi < 3; xi++ ) {
// // 		for (int yi = -2; yi < 3; yi++ ) {
// // 			for (int zi = -2; zi < 3; zi++ ) {
// // 				for (int rxi = -2; rxi < 3; rxi++ ) {
// // 					for (int ryi = -2; ryi < 3; ryi++ ) {
// // 						for (int rzi = -2; rzi < 3; rzi++ ) {
// 							set_pose_values(x+xi*Diststep,y+yi*Diststep,z+zi*Diststep,rx+rxi*Anglestep,ry+ryi*Anglestep,rz+rzi*Anglestep);
// 							
// 							IntenTemp = eval_similarity(0);
// 							if (IntenTemp<IntensityScore) {
// 								IntensityScore = IntenTemp;
// 								Intensitypose[0] = x+xi*Diststep;
// 								Intensitypose[1] = y+yi*Diststep;
// 								Intensitypose[2] = z+zi*Diststep;
// 								Intensitypose[3] = rx+rxi*Anglestep;
// 								Intensitypose[4] = ry+ryi*Anglestep;
// 								Intensitypose[5] = rz+rzi*Anglestep;
// 								}
// 								//reinit_display();
// 								//redraw_display(1);
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// 	//printf("IntensityBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f), Intensitysimilarity =%f ",Intensitypose[0],Intensitypose[1],Intensitypose[2],Intensitypose[3],Intensitypose[4],Intensitypose[5],IntensityScore);
// 	set_pose_values(Intensitypose[0],Intensitypose[1],Intensitypose[2],Intensitypose[3],Intensitypose[4],Intensitypose[5]);
// 	outFile << Intensitypose[0] <<endl<< Intensitypose[1]<<endl      // Step #4 - Write to output file.
//            << Intensitypose[2] <<endl<< Intensitypose[3]<<endl
//            << Intensitypose[4] <<endl<< Intensitypose[5]<<endl;
//     // t = clock() - t;
// //     TimeRecord << ((float)t)/CLOCKS_PER_SEC << endl;
// 	//next30Fname(fname,FileLength);
// 	nextFname(fname,FileLength);
// 	frameNo++;
// 	}
// 	outFile.close();
// // 	TimeRecord.close();
// }
void generateProjectionsIntensity(void) {

    char fname[200];
    bool error;
    cout << "  Please enter the first ppm file name:\n";

    cin.getline ( fname, sizeof ( fname ) );

    error = cin.eof();

    if ( error )
    {
        return;
    }


    ofstream outFile;         // Step #2 - Declare outstream file.

    outFile.open("/Users/malishen/Desktop/IntensityPosesInVivo429every1Powell.txt");        // Step #3 - Open outFile.

    if (outFile.fail())          // Check for file creation and return error.
    {
        cout << "Error opening \"numbers.txt\" for output.\n";

        exit(1);
    }

    ofstream TimeRecord;
    TimeRecord.open("/Users/malishen/Desktop/IntensityPosesInVivo429every1PowellTime.txt");
    if (TimeRecord.fail())          // Check for file creation and return error.
    {
        cout << "Error opening \"TimeRecord.txt\" for output.\n";

        exit(1);
    }

    int FileLength = findFileLengthPPM(fname);
    int frameNo = 1;
    RegistrationMode = 0;
/* Timing */
    clock_t t;
    while (frameNo < 51) {
        if (RecordTime) {
            t = clock();
        }
        load_image(fname);
        MinimisationPowell();
        outFile << view_x <<endl<< view_y<<endl      // Step #4 - Write to output file.
                << view_z <<endl<< spin_xso<<endl
                << spin_yso<<endl<< spin_zso<<endl;
        if (RecordTime) {
            t = clock() - t;
            TimeRecord << ((float)t)/CLOCKS_PER_SEC << endl;
        }
//next30Fname(fname,FileLength);
        nextFname(fname,FileLength);
        frameNo++;
        reinit_display();
        redraw_display(1);
    }
    outFile.close();
    TimeRecord.close();
}

// void generateProjectionsPQ(void) {
// 	
// 	char fname[200];
// 	bool error;
// 	cout << "  Please enter the next ppm file name:\n";
// 
//       cin.getline ( fname, sizeof ( fname ) );
// 
//       error = cin.eof();
// 
//       if ( error )
// 	{
// 	  return;
// 	}
// 	
// 	ofstream outFile;         // Step #2 - Declare outstream file.
// 
// //    outFile.open("/Users/malishen/Desktop/PQPosesInVivoPallavCase1_2776_2834_every1.txt");        // Step #3 - Open outFile.
// 
// 	outFile.open("/Users/malishen/Desktop/PQPosesCase3MICCAI.txt");     
//    if (outFile.fail())          // Check for file creation and return error.
//    {
//       cout << "Error opening \"PQPoses.txt\" for output.\n";
// 
//       exit(1);
//    }
// 	// ofstream TimeRecord;
// //    TimeRecord.open("/Users/malishen/Desktop/PQPosesInVivo429_every1_1000calibratedTime.txt");
// //     if (TimeRecord.fail())          // Check for file creation and return error.
// //    {
// //       cout << "Error opening \"TimeRecord.txt\" for output.\n";
// // 
// //       exit(1);
// //    }
//    
// 	int FileLength = findFileLengthPPM(fname);
// 	double PQScore;
// 	double PQTemp;
// 	float x ;
// 	float y ;
// 	float z ;
// 	float rx ;
// 	float ry ;
// 	float rz ;
// 	
// 	float Anglestep=2;
// 	float Diststep = 1;
// 	
// 	float PQpose[6];
// 	// IntensityScore = eval_similarity(0);
// 	// 	IntenTemp = IntensityScore;
// 	int frameNo = 1;
// 	/* Timing */
// 	clock_t t;
// 	while (frameNo < (120)) {
//       t = clock();
// 	load_image(fname);
// 	PQScore = INT_MAX;
// 	PQTemp = INT_MAX;
// 	x = view_x;
// 	y = view_y;
// 	z = view_z;
// 	rx = spin_xso;
// 	ry = spin_yso;
// 	rz = spin_zso;
// 	
// 	PQpose[0] = x;
// 	PQpose[1] = y;
// 	PQpose[2] = z;
// 	PQpose[3] = rx;
// 	PQpose[4] = ry;
// 	PQpose[5] = rz;
// 	for (int xi = -1; xi < 2; xi++ ) {
// 		for (int yi = -1; yi < 2; yi++ ) {
// 			for (int zi = -1; zi < 2; zi++ ) {
// 				for (int rxi = -1; rxi < 2; rxi++ ) {
// 					for (int ryi = -1; ryi < 2; ryi++ ) {
// 						for (int rzi = -1; rzi < 2; rzi++ ) {
// 	// for (int xi = -2; xi < 3; xi++ ) {
// // 		for (int yi = -2; yi < 3; yi++ ) {
// // 			for (int zi = -2; zi < 3; zi++ ) {
// // 				for (int rxi = -2; rxi < 3; rxi++ ) {
// // 					for (int ryi = -2; ryi < 3; ryi++ ) {
// // 						for (int rzi = -2; rzi < 3; rzi++ ) {
// 							set_pose_values(x+xi*Diststep,y+yi*Diststep,z+zi*Diststep,rx+rxi*Anglestep,ry+ryi*Anglestep,rz+rzi*Anglestep);
// 							
// 							PQTemp = eval_similarity(3);
// 							if (PQTemp<PQScore) {
// 								PQScore = PQTemp;
// 								PQpose[0] = x+xi*Diststep;
// 								PQpose[1] = y+yi*Diststep;
// 								PQpose[2] = z+zi*Diststep;
// 								PQpose[3] = rx+rxi*Anglestep;
// 								PQpose[4] = ry+ryi*Anglestep;
// 								PQpose[5] = rz+rzi*Anglestep;
// 								}
// 								//reinit_display();
// 								//redraw_display(1);
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// 	//printf("PQBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f), PQsimilarity =%f ",PQpose[0],PQpose[1],PQpose[2],PQpose[3],PQpose[4],PQpose[5],PQScore);
// 	set_pose_values(PQpose[0],PQpose[1],PQpose[2],PQpose[3],PQpose[4],PQpose[5]);
// 	outFile << PQpose[0] <<endl<< PQpose[1]<<endl      // Step #4 - Write to output file.
//            << PQpose[2] <<endl<< PQpose[3]<<endl
//            << PQpose[4] <<endl<< PQpose[5]<<endl;
// 	// t = clock() - t;
// //     TimeRecord << ((float)t)/CLOCKS_PER_SEC << endl;
// 	nextFname(fname,FileLength);
// 	//next30Fname(fname,FileLength);
// 	//cout << "Frame NO !!!!!!!" << frameNo << endl;
// 	frameNo++;
// 	}
// 	outFile.close();
// // 	TimeRecord.close();
// }
void generateProjectionsPQ(void) {

    char fname[200];
    bool error;
    cout << "  Please enter the next ppm file name:\n";

    cin.getline ( fname, sizeof ( fname ) );

    error = cin.eof();

    if ( error )
    {
        return;
    }

    ofstream outFile;         // Step #2 - Declare outstream file.

    outFile.open("/Users/malishen/Desktop/PQPosesInVivo429every1Powell.txt");        // Step #3 - Open outFile.

    if (outFile.fail())          // Check for file creation and return error.
    {
        cout << "Error opening \"PQPoses.txt\" for output.\n";

        exit(1);
    }

    ofstream TimeRecord;
    TimeRecord.open("/Users/malishen/Desktop/PQPosesInVivo429every1PowellTime.txt");
    if (TimeRecord.fail())          // Check for file creation and return error.
    {
        cout << "Error opening \"TimeRecord.txt\" for output.\n";

        exit(1);
    }

    int FileLength = findFileLengthPPM(fname);
    int frameNo = 1;
    RegistrationMode = 3;
/* Timing */
    clock_t t;
    while (frameNo < 51) {
        if (RecordTime) {
            t = clock();
        }
        load_image(fname);

        MinimisationPowell();
        outFile << view_x <<endl<< view_y<<endl      // Step #4 - Write to output file.
                << view_z <<endl<< spin_xso<<endl
                << spin_yso<<endl<< spin_zso<<endl;
        if (RecordTime) {
            t = clock() - t;
            TimeRecord << ((float)t)/CLOCKS_PER_SEC << endl;
        }
        nextFname(fname,FileLength);
//next30Fname(fname,FileLength);
        cout << "Frame NO !!!!!!!" << frameNo << endl;
        frameNo++;
    }
    outFile.close();
    TimeRecord.close();
}
void loadDepthFile(char *fileName, bool flip) {
    // if the depth file is a text file generated using sfs code
    if (depthImage) {
        delete [] depthImage;
    }
    cout << fileName << endl;
    FILE *fileIn;
    fileIn = fopen(fileName,"r");
    if (!fileIn) {
        cout << "can't open depthFile" << endl;
        exit(1);
    }
    float entry =0;
    _maxDepth = INT_MIN;
    _minDepth = INT_MAX;
    int _height = iheight;
    int _width = iwidth;
    depthImage = new float[_width*_height];

// read from bottom to up = the same as PPM orientation
    if (flip) {
        int j = _height;
        while(j--) {
            for( int i=0; i<_width; i++ )
            {
                fscanf(fileIn,"%f\n",&entry);
                depthImage[j*_width + i] = entry;
                _maxDepth = (entry > _maxDepth) ? entry : _maxDepth;
                _minDepth = (entry < _minDepth) ? entry : _minDepth;
            }
        }
    }
// read from top to bottom = flip vertically respect to PPM orientation
    else {
        for (int i = 0; i < _height; i++) {
            for (int j = 0; j < _width; j++) {
                fscanf(fileIn,"%f\n",&entry);
//entry /= _f; //u(x) = r(x)/f
                depthImage[i*_width + j] = entry;
                _maxDepth = (entry > _maxDepth) ? entry : _maxDepth;
                _minDepth = (entry < _minDepth) ? entry : _minDepth;
            }
        }
    }
    fclose(fileIn);
    cout << "depth file read in !!!!!!!" << endl;
    cout << "_maxDepth=" << _maxDepth << ";" << "_minDepth=" << _minDepth << endl;
    // normalise depth within [near_clip,far_clip]
    for( int i=0; i<iheight*iwidth; i++ )
    {
        depthImage[i] = (depthImage[i]-_minDepth)*(far_clip-near_clip)/(_maxDepth-_minDepth)+near_clip;
    }

// ofstream fout1("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-c/zbuffer_depth/frame_0000readin.txt");
//  	if (!fout1) return;
//  
//  	for( int i=0; i<iheight*iwidth; i++ )
//      	{
//        		depthImage[i] = (depthImage[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;
//        		fout1<< depthImage[i]<<endl;
//  		}
//  		fout1.close();
}
//// default normalise with [0,1]
//void Normalise2DImage(float *image,int width, int height) {
//    // normalise depth within [1,100]
//    for( int i=0; i<height*width; i++ )
//    {
//        image[i] = (image[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;
//    }
//}
// void generateProjectionsDepth(void) {
// 	
// 	char fname[200];
// 	char fnameDepth[200];
// 	bool error;
// 	
// 	cout << "  Please enter the next PPM file name:\n";
// 
//       cin.getline ( fname, sizeof ( fname ) );
// 
//       error = cin.eof();
// 
//       if ( error )
// 	{
// 	  return;
// 	}
// 	
// 	cout << "  Please enter the next Depth file name:\n";
// 
//       cin.getline ( fnameDepth, sizeof ( fnameDepth ) );
// 
//       error = cin.eof();
// 
//       if ( error )
// 	{
// 		cout << "Can't open Depth File" << endl;
// 	  	return;
// 	}
// 	ofstream outFile;         // Step #2 - Declare outstream file.
// 
// //    outFile.open("/Users/malishen/Desktop/DepthPosesInVivoPallavCase1_2776_2834_every1CC.txt");        // Step #3 - Open outFile.
// 	outFile.open("/Users/malishen/Desktop/DepthPosesCase3MICCAI.txt");
//    if (outFile.fail())          // Check for file creation and return error.
//    {
//       cout << "Error opening \"PQPoses.txt\" for output.\n";
// 
//       exit(1);
//    }
//    // ofstream TimeRecord;
// //    TimeRecord.open("/Users/malishen/Desktop/DepthMIPosesInVivo429_every1_1000calibratedTime.txt");
// //     if (TimeRecord.fail())          // Check for file creation and return error.
// //    {
// //       cout << "Error opening \"TimeRecord.txt\" for output.\n";
// // 
// //       exit(1);
// //    }
// 	int FileLength = findFileLengthPPM(fname);
// 	int FileLengthDepth = findFileLengthTXT(fnameDepth);
// 	
// 	double DepthScore;
// 	double DepthTemp;
// 	float x ;
// 	float y ;
// 	float z ;
// 	float rx ;
// 	float ry ;
// 	float rz ;
// 	
// 	float Anglestep=2;
// 	float Diststep = 1;
// 	
// 	float Depthpose[6];
// 
// 	int frameNo = 1;
// 	/* Timing */
// 	clock_t t;
// 	while (frameNo < 1001) {
//        t = clock();
// 	load_image(fname);
// 	loadDepthFile(fnameDepth,true);
// 	
// 	DepthScore = INT_MAX;
// 	DepthTemp = INT_MAX;
// 	
// 	x = view_x;
// 	y = view_y;
// 	z = view_z;
// 	rx = spin_xso;
// 	ry = spin_yso;
// 	rz = spin_zso;
// 	
// 	Depthpose[0] = x;
// 	Depthpose[1] = y;
// 	Depthpose[2] = z;
// 	Depthpose[3] = rx;
// 	Depthpose[4] = ry;
// 	Depthpose[5] = rz;
// 
// 	// for (int xi = -2; xi < 3; xi++ ) {
// // 		for (int yi = -2; yi < 3; yi++ ) {
// // 			for (int zi = -2; zi < 3; zi++ ) {
// // 				for (int rxi = -2; rxi < 3; rxi++ ) {
// // 					for (int ryi = -2; ryi < 3; ryi++ ) {
// // 						for (int rzi = -2; rzi < 3; rzi++ ) {
// for (int xi = -1; xi < 2; xi++ ) {
// 		for (int yi = -1; yi < 2; yi++ ) {
// 			for (int zi = -1; zi < 2; zi++ ) {
// 				for (int rxi = -1; rxi < 2; rxi++ ) {
// 					for (int ryi = -1; ryi < 2; ryi++ ) {
// 						for (int rzi = -1; rzi < 2; rzi++ ) {
// 							set_pose_values(x+xi*Diststep,y+yi*Diststep,z+zi*Diststep,rx+rxi*Anglestep,ry+ryi*Anglestep,rz+rzi*Anglestep);
// 							cout << "new pose set" << endl;
// 							DepthTemp = eval_similarity(4);
// 							if (DepthTemp<DepthScore) {
// 								DepthScore = DepthTemp;
// 								Depthpose[0] = x+xi*Diststep;
// 								Depthpose[1] = y+yi*Diststep;
// 								Depthpose[2] = z+zi*Diststep;
// 								Depthpose[3] = rx+rxi*Anglestep;
// 								Depthpose[4] = ry+ryi*Anglestep;
// 								Depthpose[5] = rz+rzi*Anglestep;
// 								}
// 								//reinit_display();
// 								//redraw_display(1);
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// 	//printf("DepthBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f), Depthsimilarity =%f ",Depthpose[0],Depthpose[1],Depthpose[2],Depthpose[3],Depthpose[4],Depthpose[5],DepthScore);
// 	set_pose_values(Depthpose[0],Depthpose[1],Depthpose[2],Depthpose[3],Depthpose[4],Depthpose[5]);
// 	outFile << Depthpose[0] <<endl<< Depthpose[1]<<endl      // Step #4 - Write to output file.
//            << Depthpose[2] <<endl<< Depthpose[3]<<endl
//            << Depthpose[4] <<endl<< Depthpose[5]<<endl;
// 
// 	// next30Fname(fname,FileLength);
// // 	next30Fname(fnameDepth,FileLengthDepth);
// 	// t = clock() - t;
// //     TimeRecord << ((float)t)/CLOCKS_PER_SEC << endl;
// 	nextFname(fname,FileLength);
// 	nextFname(fnameDepth,FileLengthDepth);
// 	//cout << "!!!!!!frameNo=" << frameNo << endl;
// 	frameNo++;
// 	}
// 	outFile.close();
// // 	TimeRecord.close();
// }

/* function to compute pose using depth based 2D/3D registration */
void generateProjectionsDepth(void) {

    char fname[200];
    char fnameDepth[200];
    bool error;
    ofstream outFile;         // Declare outstream file for saving tracking  poses.
    ofstream TimeRecordFile;     // declare outstream file for saving time record.
    clock_t t; // /* Timing */
    cout << "  Please enter the next PPM file name:\n";

    cin.getline ( fname, sizeof ( fname ) );

    error = cin.eof();

    if ( error )
    {
        return;
    }

    cout << "  Please enter the next Depth file name:\n";

    cin.getline ( fnameDepth, sizeof ( fnameDepth ) );

    error = cin.eof();

    if ( error )
    {
        cout << "Can't open Depth File" << endl;
        return;
    }
    if (saveTrack)
    {
        //    outFile.open("/Users/malishen/Documents/MICCAI2016DesktopFiles/DepthPosesCase1Seq1_F1_F795_every1CCPowell.txt");        // Step #3 - Open outFile.
        // 	outFile.open("/Users/malishen/Dropbox/MICCAI/MICCAI2017PhantomDepthResults/DepthPosesphantomSeq1_F1077_F2245_every1CCPowell.txt");
        // 	outFile.open("/Users/malishen/Dropbox/MICCAI/MICCAI2017InvivoDepthResults/DepthPosesCase1Seq1_F352_F592_every1CCPowell.txt");        // Step #3 - Open outFile.
        // outFile.open("/Users/malishen/Dropbox/MICCAI/MICCAI2017InvivoDepthResults/DepthPosesCase1Seq2_F51_F393_every1CCPowell.txt");        // case 1 seq2.
        outFile.open("/Users/malishen/Desktop/testReg/DepthPosesPhantom_F109_F393_every1CCPowellTest.txt");        // Test File.

        if (outFile.fail())          // Check for file creation and return error.
        {
            cout << "Error opening \"PQPoses.txt\" for output.\n";

            exit(1);
        }
    }
    if (RecordTime)	{
        // TimeRecord.open("/Users/malishen/Dropbox/MICCAI/MICCAI2017InvivoDepthResults/DepthPosesCase1Seq2_F51_F393_every1CCPowellTime.txt");
        TimeRecordFile.open("/Users/malishen/Desktop/testReg/DepthPosesPhantom_F109_F393_every1CCPowellTimeTest.txt");

        if (TimeRecordFile.fail())          // Check for file creation and return error.
        {
            cout << "Error opening \"TimeRecord.txt\" for output.\n";

            exit(1);
        }
    }
    int FileLength = findFileLengthPPM(fname);
    int FileLengthDepth = findFileLengthTXT(fnameDepth);

    int frameNo = 109;
    RegistrationMode = 4;

    while (frameNo < 394) {  //394
        if (RecordTime) {
            t = clock();
        }
        load_image(fname);
        loadDepthFile(fnameDepth,true);

        MinimisationPowell();
        if (saveTrack) {
            outFile << view_x <<endl<< view_y<<endl      // Step #4 - Write to output file.
                    << view_z <<endl<< spin_xso<<endl
                    << spin_yso<<endl<< spin_zso<<endl;
        }
        if (RecordTime) {
            t = clock() - t;
            TimeRecordFile << ((float)t)/CLOCKS_PER_SEC << endl;
        }
        if( !zbuffer ) // input to ShapeFromShading()
        {
            printf( "ShapeFromShading: zbuffer has not been initialized\n" );
        }
        else
        {
            // scaled_buffer is a float image
            float *scaled_buffer = new float[iheight*iwidth];
            bool have_del_z = false;
            if( oglwidth != iwidth || oglheight != iheight )
                gluScaleImage( GL_DEPTH_COMPONENT, oglwidth, oglheight, GL_FLOAT,
                               zbuffer, iwidth, iheight, GL_FLOAT, scaled_buffer );
            else
            {
                delete [] scaled_buffer;
                scaled_buffer = zbuffer;
                have_del_z = true;
            }
            printf("Updating z-buffer\n");
            zbuffer2TrueDepth(near_clip,far_clip,scaled_buffer);
            // output CT depth map as PPM image
            char depthfilenameppm[200]; // filename depth ppm
            snprintf(depthfilenameppm, sizeof(depthfilenameppm), "/Users/malishen/Desktop/testReg/CTDepth/depthppm_%d.ppm", frameNo);
            test_fun4( scaled_buffer, depthfilenameppm,iwidth,iheight, near_clip,far_clip);
            // output Video depth map as PPM image
            char Vdepthfilenameppm[200]; // filename depth ppm
            snprintf(Vdepthfilenameppm, sizeof(Vdepthfilenameppm), "/Users/malishen/Desktop/testReg/VideoDepth/depthVppm_%d.ppm", frameNo);
            test_fun4( depthImage, Vdepthfilenameppm,iwidth,iheight, near_clip,far_clip);
            cout << "min video depth=" << _minDepth << ", max video depth=" << _maxDepth << endl;
            // store_float_image( scaled_buffer, iwidth, iheight, depthfilenameppm);
            // zbuffer2Depthmap(scaled_buffer, depthMap,mousecoord, info,, focal, iwidth, near_clip, far_clip );
            // store_2D_img( depthMap, wid_heig, 3,depthfilename );
        }
        // next30Fname(fname,FileLength);
        // 	next30Fname(fnameDepth,FileLengthDepth);
        nextFname(fname,FileLength);
        nextFname(fnameDepth,FileLengthDepth);
//		cout << "!!!!!!frameNo=" << frameNo << endl;
        frameNo++;
    }
    if (saveTrack){
        outFile.close();
    }
    if (RecordTime) {
        TimeRecordFile.close();
    }
}


void generateProjectionsPQDepth(void) {

    char fname[200];
    char fnameDepth[200];
    bool error;

    cout << "  Please enter the next PPM file name:\n";

    cin.getline ( fname, sizeof ( fname ) );

    error = cin.eof();

    if ( error )
    {
        return;
    }

    cout << "  Please enter the next Depth file name:\n";

    cin.getline ( fnameDepth, sizeof ( fnameDepth ) );

    error = cin.eof();

    if ( error )
    {
        cout << "Can't open Depth File" << endl;
        return;
    }
    ofstream outFile;         // Step #2 - Declare outstream file.

    outFile.open("/Users/malishen/Desktop/DepthPQPoses429CC.txt");        // Step #3 - Open outFile.

    if (outFile.fail())          // Check for file creation and return error.
    {
        cout << "Error opening \"DepthPQPoses.txt\" for output.\n";

        exit(1);
    }

    int FileLength = findFileLengthPPM(fname);
    int FileLengthDepth = findFileLengthTXT(fnameDepth);

    double DepthScore;
    double DepthTemp;
    double PQScore;
    double PQTemp;
    float x ;
    float y ;
    float z ;
    float rx ;
    float ry ;
    float rz ;

    float Anglestep=2;
    float Diststep = 1;

    float Depthpose[6];

    int frameNo = 1;
    while (frameNo < 101) {

        load_image(fname);
        loadDepthFile(fnameDepth,true);

        DepthScore = INT_MAX;
        DepthTemp = INT_MAX;
        PQScore = INT_MAX;
        PQTemp = INT_MAX;
        x = view_x;
        y = view_y;
        z = view_z;
        rx = spin_xso;
        ry = spin_yso;
        rz = spin_zso;

        Depthpose[0] = x;
        Depthpose[1] = y;
        Depthpose[2] = z;
        Depthpose[3] = rx;
        Depthpose[4] = ry;
        Depthpose[5] = rz;

// for (int xi = -2; xi < 3; xi++ ) {
// 		for (int yi = -2; yi < 3; yi++ ) {
// 			for (int zi = -2; zi < 3; zi++ ) {
// 				for (int rxi = -2; rxi < 3; rxi++ ) {
// 					for (int ryi = -2; ryi < 3; ryi++ ) {
// 						for (int rzi = -2; rzi < 3; rzi++ ) {
        for (int xi = -1; xi < 2; xi++ ) {
            for (int yi = -1; yi < 2; yi++ ) {
                for (int zi = -1; zi < 2; zi++ ) {
                    for (int rxi = -1; rxi < 2; rxi++ ) {
                        for (int ryi = -1; ryi < 2; ryi++ ) {
                            for (int rzi = -1; rzi < 2; rzi++ ) {
                                set_pose_values(x+xi*Diststep,y+yi*Diststep,z+zi*Diststep,rx+rxi*Anglestep,ry+ryi*Anglestep,rz+rzi*Anglestep);
                                cout << "new pose set" << endl;
                                DepthTemp = eval_similarity(4);
                                PQTemp = eval_similarity(3);
                                if (DepthTemp<DepthScore && PQTemp<PQScore) {
                                    DepthScore = DepthTemp;
                                    PQScore = PQTemp;
                                    Depthpose[0] = x+xi*Diststep;
                                    Depthpose[1] = y+yi*Diststep;
                                    Depthpose[2] = z+zi*Diststep;
                                    Depthpose[3] = rx+rxi*Anglestep;
                                    Depthpose[4] = ry+ryi*Anglestep;
                                    Depthpose[5] = rz+rzi*Anglestep;
                                }
//reinit_display();
//redraw_display(1);
                            }
                        }
                    }
                }
            }
        }
        printf("DepthBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f), Depthsimilarity =%f ",Depthpose[0],Depthpose[1],Depthpose[2],Depthpose[3],Depthpose[4],Depthpose[5],DepthScore);
        set_pose_values(Depthpose[0],Depthpose[1],Depthpose[2],Depthpose[3],Depthpose[4],Depthpose[5]);
        outFile << Depthpose[0] <<endl<< Depthpose[1]<<endl      // Step #4 - Write to output file.
                << Depthpose[2] <<endl<< Depthpose[3]<<endl
                << Depthpose[4] <<endl<< Depthpose[5]<<endl;

// next30Fname(fname,FileLength);
// 	next30Fname(fnameDepth,FileLengthDepth);

        nextFname(fname,FileLength);
        nextFname(fnameDepth,FileLengthDepth);

        frameNo++;
        cout << "!!!!!!frameNo=" << frameNo << endl;
    }
    outFile.close();
}
void generateProjections(void) {
    float x = view_x;
    float y = view_y;
    float z = view_z;
    float rx = spin_xso;
    float ry = spin_yso;
    float rz = spin_zso;

    float step=2;

    double IntensityScore = 0;
    double IntenTemp = 0;

    IntensityScore = eval_similarity(0);
    IntenTemp = IntensityScore;
    load_image("/Users/malishen/Documents/MATLAB/OldMac/TOOLBOX_calib/rect_ppm/D97193_1186_0001_rect.ppm");
}
//void generateProjections(float x,float y, float z, float rx, float ry, float rz) {
// void generateProjections(void) {
// 	float x = view_x;
// 	float y = view_y;
// 	float z = view_z;
// 	float rx = spin_xso;
// 	float ry = spin_yso;
// 	float rz = spin_zso;
// 	
// 	float step=2;
// 	//float step=2.5;
// 	double SFSPQScore = 0;
// 	double SFSTemp = 0;
// 	double IntensityScore = 0;
// 	double IntenTemp = 0;
// 	SFSPQScore = eval_similarity(3);
// 	SFSTemp = SFSPQScore;
// 	IntensityScore = eval_similarity(0);
// 	IntenTemp = IntensityScore;
// 	
// 	//set_pose_values(x*(1),y+step,z,rx,ry,rz);
// 	
// 	float SFSpose[6] = {x,y,z,rx,ry,rz};
// 	float Intensitypose[6] = {x,y,z,rx,ry,rz};
// 	for (int xi = -1; xi < 2; xi++ ) {
// 		for (int yi = -1; yi < 2; yi++ ) {
// 			for (int zi = -1; zi < 2; zi++ ) {
// 				for (int rxi = -1; rxi < 2; rxi++ ) {
// 					for (int ryi = -1; ryi < 2; ryi++ ) {
// 						for (int rzi = -1; rzi < 2; rzi++ ) {
// 							set_pose_values(x+xi*step,y+yi*step,z+zi*step,rx+rxi*step,ry+ryi*step,rz+rzi*step);
// 							
// 							SFSTemp = eval_similarity(3);
// 							if (SFSTemp<SFSPQScore) {
// 								SFSPQScore = SFSTemp;
// 								SFSpose[0] = x+xi*step;
// 								SFSpose[1] = y+yi*step;
// 								SFSpose[2] = z+zi*step;
// 								SFSpose[3] = rx+rxi*step;
// 								SFSpose[4] = ry+ryi*step;
// 								SFSpose[5] = rz+rzi*step;
// 								}
// 							IntenTemp = eval_similarity(0);
// 							if (IntenTemp<IntensityScore) {
// 								IntensityScore = IntenTemp;
// 								Intensitypose[0] = x+xi*step;
// 								Intensitypose[1] = y+yi*step;
// 								Intensitypose[2] = z+zi*step;
// 								Intensitypose[3] = rx+rxi*step;
// 								Intensitypose[4] = ry+ryi*step;
// 								Intensitypose[5] = rz+rzi*step;
// 								}
// 								//reinit_display();
// 								//redraw_display(1);
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// 	printf("SFSBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f), similarity =%f ",SFSpose[0],SFSpose[1],SFSpose[2],SFSpose[3],SFSpose[4],SFSpose[5],SFSPQScore);
// 	cout << endl;
// 	printf("InputBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f)",x,y,z,rx,ry,rz);
// 	cout << endl;
// 	printf("IntensityBestPose(x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f), similarity =%f ",Intensitypose[0],Intensitypose[1],Intensitypose[2],Intensitypose[3],Intensitypose[4],Intensitypose[5],IntensityScore);
// 	set_pose_values(SFSpose[0],SFSpose[1],SFSpose[2],SFSpose[3],SFSpose[4],SFSpose[5]);
// 	
// }
/*
void zbuffer2Depthmap(float *zbuf, vector<GLMpoint_d> *depthMap,int *mousecoord, 
float *info, float focal, int zbuf_width, float fnear, float ffar ){

//copy mouse coordinates to (x0,y0)->lower left corner, (x1,y1)->upper right corner
int x0 = mousecoord[0];	int y0 = mousecoord[1];
int x1 = mousecoord[2];	int y1 = mousecoord[3];
int width = x1 - x0 +1;
int height = y1 - y0 +1;

double iwidth = (double)(info[0]);		double iheight = (double)(info[1]);
double imgwidth = (double)(info[2]);	double imgheight = (double)(info[3]);
printf("x0=%d,y0=%d,x1=%d,y1=%d,width=%d,height=%d,iwidth=%f,iheight=%f,imgwidth=%f,imgheight=%f",x0,y0,x1,y1,width,height,iwidth,iheight,imgwidth,imgheight);
bool flag = false;	// flag=false if every z-buffer pixel == 1 =>object is not in the FOV

//vector<GLMpoint_d> lets_test;
GLMpoint_d lets_temp;

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
else
cout << "zbuffer not all one "<<"have "<<countZ<<"non-one points, in pqmodel"<<endl;

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

lets_temp.x = zx;
lets_temp.y = zy;
lets_temp.z = zdepth[0];
depthMap->push_back( lets_temp );


}

void generateDepths (float x,float y, float z, float rx, float ry, float rz) {
mousecoordtr( mouse );
if( !mouse[0] && !mouse[1] && !mouse[2] && !mouse[3] )
{
mouse[0] = 0; mouse[1] = 0; mouse[2] = oglwidth-1; mouse[3] = oglheight-1;
update_mouse( mouse );
}
int mousecoord[4];
mousecoord[0] = (int)( (float)(mouse[0])*(float)(iwidth)/(float)(oglwidth) ) +1;
mousecoord[1] = (int)( (float)(mouse[1])*(float)(iheight)/(float)(oglheight) ) +1;
mousecoord[2] = (int)( (float)(mouse[2])*(float)(iwidth)/(float)(oglwidth) ) -1;
mousecoord[3] = (int)( (float)(mouse[3])*(float)(iheight)/(float)(oglheight) ) -1;

float step[2] = { zoomFactorx, zoomFactory };

if( !zbuffer ) // input to ShapeFromShading()
{
printf( "ShapeFromShading: zbuffer has not been initialized\n" );
return 0.0;
}

float *scaled_buffer = new float[iheight*iwidth];
bool have_del_z = false;
if( oglwidth != iwidth || oglheight != iheight )
gluScaleImage( GL_DEPTH_COMPONENT, oglwidth, oglheight, GL_FLOAT,
zbuffer, iwidth, iheight, GL_FLOAT, scaled_buffer );
else
{
delete [] scaled_buffer;
scaled_buffer = zbuffer;
have_del_z = true;
}
printf("Updating z-buffer\n");
//vector<long> inf_vector;
// camera parameters how info calculated??
float info[4] = {2.0*near_clip*aspect/focal, 2.0*near_clip/focal, iwidth, iheight };
cout << "mouse="<<mouse[0]<<" "<<mouse[1]<<" "<<mouse[2]<<" "<<mouse[3]<<endl;
cout << "mousecoord="<< mousecoord[0]<<" "<<mousecoord[1]<<" "<<mousecoord[2]<<" "<<mousecoord[3]<<endl;
// calculate pq_model calling shapeshade.cpp
vector<GLMpoint_d> depthMap;
int x1 = mousecoord[2];	int y1 = mousecoord[3];
int wid_heig[2] = { x1+2, y1+2};
zbuffer2Depthmap(scaled_buffer, depthMap,mousecoord, info,, focal, iwidth, near_clip, far_clip );

store_2D_img( depthMap, wid_heig, 3,depthfilename );
printf("zbuffer saved to file\n");
}
*/
static void image_transparency( unsigned char trans ){
    for( int i=0; i<iwidth*iheight; i++ )
    {
        image[i*4 + 3] = trans;
        image2[i*4 +3] = trans;
    }
}

/* restore image from backup copy, image2 is the backup*/
void restore_image() {
    for( int i = 0; i<iwidth*iheight*4; i++ )
        image[i] = image2[i];
}


void get_bounds( float b[] ) {
    for( int i = 0; i<6; i++ )
        b[i] = bounds[i];
}

void set_near_clip( float n ) { near_clip = n; }
void set_far_clip( float n ) { far_clip = n; }

// in vivo case 1
// BF260
// Focal Length:          fc = [ 213.17558   217.16548 ] ± [ 3.49401   3.63654 ]
// Principal point:       cc = [ 161.53330   154.13610 ] ± [ 1.70835   1.05239 ]
 static double ccx = 161.53330;	//principal point x-pixel coordinate
 static double ccy = 154.13610;	//principal point y-pixle coordinate
 static double fcx = 213.17558;	//focal length in x-pixels
 static double fcy = 217.16548;	//focal length in y-pixels

// in vivo case 2 &3
// BF260F
// Focal Length:          fc = [ 208.46507   205.99039 ] ± [ 3.84765   3.83482 ]
// Principal point:       cc = [ 159.08973   154.38814 ] ± [ 2.10345   1.59137 ]
// static double ccx = 159.08973;	//principal point x-pixel coordinate
// static double ccy = 154.38814;	//principal point y-pixle coordinate
// static double fcx = 208.46507;	//focal length in x-pixels
// static double fcy = 205.99039;	//focal length in y-pixels

// silicon phantom and pig lung 
//    %-- Focal length:
//    fc = [ 321.893878980221980 ; 352.081239611292630 ];

//    %-- Principal point:
//    cc = [ 113.905868037321770 ; 122.528073440898110 ];
//static double ccx = 113.905868037321770;	//principal point x-pixel coordinate
//static double ccy = 122.528073440898110;	//principal point y-pixle coordinate
//static double fcx = 321.893878980221980;	//focal length in x-pixels
//static double fcy = 352.081239611292630;	//focal length in y-pixels

// in vivo Case 3
// Focal Length:          fc = [ 218.64957   222.13512 ] ± [ 7.38013   7.79117 ]
// Principal point:       cc = [ 161.39231   156.01091 ] ± [ 3.83117   2.08797 ]

// static double ccx = 161.39231;	//principal point x-pixel coordinate
// static double ccy = 156.01091;	//principal point y-pixle coordinate
// static double fcx = 218.64957;	//focal length in x-pixels
// static double fcy = 222.13512;	//focal length in y-pixels

// in vivo Fani
// static double ccx = 231.736545717156390;	//principal point x-pixel coordinate
// static double ccy = 249.275600598035110;	//principal point y-pixle coordinate
// static double fcx = 301.692071832502220;	//focal length in x-pixels
// static double fcy = 325.299924499268170;	//focal length in y-pixels

// in vivo Pallav
// static double ccx = 156.6086;	//principal point x-pixel coordinate
// static double ccy = 156.7669;	//principal point y-pixle coordinate
// static double fcx = 301.692071832502220;	//focal length in x-pixels
// static double fcy = 325.299924499268170;	//focal length in y-pixels
// static double fcx = 211.9159;	//focal length in x-pixels
// static double fcy = 214.9643;	//focal length in y-pixels


// phantom
// static double ccx = 158.44569397;	//principal point x-pixel coordinate
// static double ccy = 126.848937988;	//principal point y-pixle coordinate
// static double fcx = 355.70211792;	//focal length in x-pixels
// static double fcy = 352.75958252;	//focal length in y-pixels

void screen_reshape(int width, int height)
{
//printf("in screen_reshape(), projection.cpp\n");
//store them for later use
    oglwidth = width;
    oglheight = height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//  glFrustum(-near_clip*aspect/focal, near_clip*aspect/focal,
//	    -near_clip/focal, near_clip/focal, near_clip, far_clip);

    GLdouble left, right, bottom, top, near1, far1;
    GLdouble width_d = (double)(width);	GLdouble height_d = (double)(height);
    near1 = near_clip;
    far1 = far_clip;
    left = near1*(-ccx/fcx);
    bottom = near1*(ccy-height_d+1.0)/fcy;
    right = near1*(width_d-1.0-ccx)/fcx;
    top = near1*ccy/fcy;
    glFrustum(left, right, bottom, top, near1, far1 );
//printf("glFrunstum(left=%f,right=%f,bottom=%f,top=%f,near1=%f,far1=%f)",left,right,bottom,top,near1,far1);
    glRotatef( 180.0, 0.0, 1.0, 0.0 ); // face camera along +ve z

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnable(GL_LIGHTING);

// light will be at the viewpoint
    GLfloat light_pos[] = { 0.0, 0.0, 0.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

//  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_difcol);
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, att_0);
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, att_1);
    glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, att_2);

    glLightfv(GL_LIGHT0, GL_AMBIENT, &RGBAcolor[0]);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, &RGBAcolor[4]);
    glLightfv(GL_LIGHT0, GL_SPECULAR, &RGBAcolor[8]);

    GLfloat spotdirection[] = {0.0, 0.0, 1.0};
    glLightf( GL_LIGHT0, GL_SPOT_CUTOFF, spotangle );
    glLightfv( GL_LIGHT0, GL_SPOT_DIRECTION, spotdirection );
    glLightf( GL_LIGHT0, GL_SPOT_EXPONENT, spotexponent );

    glEnable(GL_LIGHT0);

    glClearColor(0.0, 0.0, 0.5, 0.0);
    glEnable(GL_DEPTH_TEST);

//calculate image scaling factors
    zoomFactorx = (GLfloat)width / (GLfloat)iwidth;
    zoomFactory = (GLfloat)height / (GLfloat)iheight;
    glPixelZoom( zoomFactorx, zoomFactory);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


/**** main view display update ****/

void
screen_display(int mx, bool grab)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


//camera transform
    glRotatef(spin_zso, 0.0, 0.0, 1.0);
    glRotatef(spin_xso, 1.0, 0.0, 0.0);
    glRotatef(spin_yso, 0.0, 1.0, 0.0);

    GLdouble model_view[16];
    glGetDoublev( GL_MODELVIEW_MATRIX, model_view);

    glTranslatef( -view_x, -view_y, -view_z );

    if( GetTransf )
    {
        glGetDoublev( GL_MODELVIEW_MATRIX, modelviewmatrix );
        glGetDoublev( GL_PROJECTION_MATRIX, projectionmatrix );
        glGetIntegerv( GL_VIEWPORT, viewport );
        GetTransf = false;
    }

//model transform
    if (pmodel[mx])
    {
        pmodel[mx]->materials[0].ambient[0] = RGBAcolor[12];
        pmodel[mx]->materials[0].ambient[1] = RGBAcolor[13];
        pmodel[mx]->materials[0].ambient[2] = RGBAcolor[14];
        pmodel[mx]->materials[0].ambient[3] = RGBAcolor[15];
        pmodel[mx]->materials[0].diffuse[0] = RGBAcolor[16];
        pmodel[mx]->materials[0].diffuse[1] = RGBAcolor[17];
        pmodel[mx]->materials[0].diffuse[2] = RGBAcolor[18];
        pmodel[mx]->materials[0].diffuse[3] = (grab) ? 1.0 : RGBAcolor[19];
        pmodel[mx]->materials[0].specular[0] = RGBAcolor[20];
        pmodel[mx]->materials[0].specular[1] = RGBAcolor[21];
        pmodel[mx]->materials[0].specular[2] = RGBAcolor[22];
        pmodel[mx]->materials[0].specular[3] = RGBAcolor[23];
        pmodel[mx]->materials[0].shininess = shininess;

        glPolygonMode(GL_BACK, GL_LINE);
//      glmDraw(pmodel[mx], GLM_SMOOTH | GLM_MATERIAL | GLM_STRIP );
        glmDraw(pmodel[mx], GLM_SMOOTH | GLM_MATERIAL);
//  glmDraw(pmodel[mx], GLM_SMOOTH | GLM_MATERIAL, centreline);

//glmDraw(pmodel[mx], GLM_SMOOTH | GLM_COLOR );
    }
    glDisable(GL_BLEND);

    if( draw_sphere )
    {
        GLUquadricObj *qobj;
        qobj = gluNewQuadric();
        gluQuadricDrawStyle( qobj, GLU_FILL );
        gluQuadricNormals( qobj, GLU_SMOOTH );

        glTranslatef( sphere_centre[0], sphere_centre[1], sphere_centre[2] );

        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glColor4f( 0.8, 0.0, 0.0, 0.5 );
        gluSphere( qobj, radius, 15, 10 );
        glEnable(GL_LIGHTING);
        glDisable(GL_BLEND);
        gluDeleteQuadric(qobj);
    }

    if( zbuffer_read )
    {
        int x0 = 0; int y0 = 0;
        int x1 = oglwidth-1; int y1 = oglheight-1;

        int tw = x1 - x0+1; int th = y1 - y0+1;
        if( zbuffer ) {
//            cout << "trying to delete zbuffer" << endl;
            delete [] zbuffer;
        }
        zbuffer = new float[tw*th];
// update zbuffer !!!!
//printf("glReadPixels() to update zbuffer\n");
//printf("x0=%d, y0=%d, tw=%d, th=%d",x0,y0,tw,th);
        glReadPixels( x0, y0, tw, th, GL_DEPTH_COMPONENT, GL_FLOAT, zbuffer );
        zbuffer_read = false;
        bool allone = true;
        int count = 0;
        for(int i=0;i<(tw*th);i++)
        {
            if (zbuffer[i]!=1) {
                allone=false;
//cout <<zbuffer[i]<<" ";
                count++;
            }
        }
        if (allone==true)
            cout << "zbuffer all one"<<endl;
//        else
//            cout << "zbuffer not all one "<<"have "<<count<<"non-one points"<<endl;

    }

    if (grab) {
//read frame buffer
        if (view_image){
//printf("trying to delete view_image\n");
            delete[] view_image;
//printf("image deleted....\n");
        }
        view_image = new unsigned char[oglheight*oglwidth*4];
// update view_image !!!!
        glReadPixels( 0, 0, oglwidth, oglheight, GL_RGBA, GL_UNSIGNED_BYTE,
                      view_image );
    }

// define image transparency 
    image_transparency(trans);
//display image
    glEnable(GL_BLEND);

    glDrawPixels( iwidth, iheight, GL_RGBA, GL_UNSIGNED_BYTE, image);

    glDisable(GL_BLEND);

}


void set_vx( float x ) { view_x = (GLfloat)x; }
void set_vy( float y ) { view_y = (GLfloat)y; }
void set_vz( float z ) { view_z = (GLfloat)z; }

void set_rx( float a ) { spin_xso = a; }
void set_ry( float a ) { spin_yso = a; }
void set_rz( float a ) { spin_zso = a; }

void set_trans( unsigned char a) { trans = a; }
unsigned char get_trans() { return trans; }

//set lighting conditions
void set_illumtype( int a ){ illumType = a; }
void get_illumtype( int *a ){ *a = illumType; }
void set_redcomp( float a ){
    switch(illumType)
    {
        case 0:
            RGBAcolor[0] = a;
            break;
        case 1:
            RGBAcolor[4] = a;
            break;
        case 2:
            RGBAcolor[8] = a;
            break;
        case 3:
            RGBAcolor[12] = a;
            break;
        case 4:
            RGBAcolor[16] = a;
            break;
        case 5:
            RGBAcolor[20] = a;
            break;
        default:
            printf("set_redcomp: wrong illumType");
            break;
    }
}

void set_greencomp( float a ){
    switch(illumType)
    {
        case 0:
            RGBAcolor[1] = a;
            break;
        case 1:
            RGBAcolor[5] = a;
            break;
        case 2:
            RGBAcolor[9] = a;
            break;
        case 3:
            RGBAcolor[13] = a;
            break;
        case 4:
            RGBAcolor[17] = a;
            break;
        case 5:
            RGBAcolor[21] = a;
            break;
        default:
            printf("set_greencomp: wrong illumType");
            break;
    }
}

void set_bluecomp( float a ){
    switch(illumType)
    {
        case 0:
            RGBAcolor[2] = a;
            break;
        case 1:
            RGBAcolor[6] = a;
            break;
        case 2:
            RGBAcolor[10] = a;
            break;
        case 3:
            RGBAcolor[14] = a;
            break;
        case 4:
            RGBAcolor[18] = a;
            break;
        case 5:
            RGBAcolor[22] = a;
            break;
        default:
            printf("set_bluecomp: wrong illumType");
            break;
    }
}

void set_alphacomp( float a ){
    switch(illumType)
    {
        case 0:
            RGBAcolor[3] = a;
            break;
        case 1:
            RGBAcolor[7] = a;
            break;
        case 2:
            RGBAcolor[11] = a;
            break;
        case 3:
            RGBAcolor[15] = a;
            break;
        case 4:
            RGBAcolor[19] = a;
            break;
        case 5:
            RGBAcolor[23] = a;
            break;
        default:
            printf("set_alphacomp: wrong illumType");
            break;
    }
}

float* get_RGBAvalues( void )
{
    switch(illumType)
    {
        case 0:
            return &RGBAcolor[0];
        case 1:
            return &RGBAcolor[4];
        case 2:
            return &RGBAcolor[8];
        case 3:
            return &RGBAcolor[12];
        case 4:
            return &RGBAcolor[16];
        case 5:
            return &RGBAcolor[20];
        default:
            printf("set_alphacomp: wrong illumType");
            return NULL;
    }

}

void set_spotlight( float angle, float exp ) { spotangle = angle; spotexponent = exp;}
void get_spotlight( float *angle, float *exp ) { *angle = spotangle; *exp = spotexponent;}

void set_shininess( float a ) { shininess = a; }
void get_shininess( float *a ) { *a = shininess; }

void set_attenuation( float c0, float c1, float c2 ) {
    att_0 = c0; att_1 = c1;	att_2 = c2;
    reinit_display();
}
void get_attenuation( float *c0, float *c1, float *c2 ){
    *c0 = att_0; *c1 = att_1; *c2 = att_2;
}


//allow changes to focal length at any time
void set_focal_length(float f) {
    focal = f;
    reinit_display();
    redraw_display(1);
}

void get_pose_values( float *x, float *y, float *z,
                      float *rx, float *ry, float *rz ) {
    *x = view_x;
    *y = view_y;
    *z = view_z;
    *rx = spin_xso;
    *ry = spin_yso;
    *rz = spin_zso;
}

void set_pose_values( float x, float y, float z,
                      float rx, float ry, float rz ) {
    view_x = x;
    view_y = y;
    view_z = z;
    spin_xso = rx;
    spin_yso = ry;
    spin_zso = rz;
// update sliders
    set_sliders(0);
    reinit_display();
    redraw_display(1);
// for (int i = 0; i<100; i++) {
// 	view_x = centreline[3*i];
// 	view_y = centreline[3*i+1];
// 	view_z = centreline[3*i+2];
// 	cout << view_x << view_y << view_z<<endl;
// 	// update sliders
//   set_sliders(0);
//   redraw_display(1);
// 	}
}


//use deformation code --- only implemented for shape from shading
void set_deform( bool a ) { deformation = a; }
void set_def_threshold( float a) { def_threshold = a; }

//get-set input/output parameters for registering a series of video frames
void get_info_start( int *init_frame, int *final_frame, int *step, char **input_path, char **output_path )
{
    *init_frame = RegVidPar.init_frame;
    *final_frame = RegVidPar.final_frame;
    *step = RegVidPar.step;
    *input_path = RegVidPar.input_path;
    *output_path = RegVidPar.output_path;
}

void set_info_start( int init_frame, int final_frame, int step, const char *input_path, const char *output_path )
{
    RegVidPar.init_frame = init_frame;
    RegVidPar.final_frame = final_frame;
    RegVidPar.step = step;

    if( RegVidPar.input_path )
        delete [] RegVidPar.input_path;
    if( input_path )
    {
        int len = strlen ( input_path );
        RegVidPar.input_path = new char [len+1];
        strcpy( RegVidPar.input_path, input_path );
    }

    if( RegVidPar.output_path )
        delete [] RegVidPar.output_path;
    if( output_path )
    {
        int len = strlen ( output_path );
        RegVidPar.output_path = new char [len+1];
        strcpy( RegVidPar.output_path, output_path );
    }
}

//get-set input/output parameters for storing a series of the 3D model images
void get_store_info( int *init_frame, int *final_frame, int *step, char **output_path, int *imgwidth, int *imgheight )
{
    *init_frame = Store3DFrames.init_frame;;
    *final_frame = Store3DFrames.final_frame;
    *step = Store3DFrames.step;
    *output_path = Store3DFrames.output_path;
    *imgwidth = Store3DFrames.imgwidth;
    *imgheight = Store3DFrames.imgheight;
}

void set_store_info( int init_frame, int final_frame, int step,
                     const char *output_path, int imgwidth, int imgheight )
{
    Store3DFrames.init_frame = init_frame;
    Store3DFrames.final_frame = final_frame;
    Store3DFrames.step = step;
    Store3DFrames.imgwidth = imgwidth;
    Store3DFrames.imgheight = imgheight;

    if( Store3DFrames.output_path )
        delete [] Store3DFrames.output_path;
    int len = strlen ( output_path );
    Store3DFrames.output_path = new char [len+1];
    strcpy( Store3DFrames.output_path, output_path );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//set sphere parameters
void set_radius( float a ) { radius = a; }
void set_centre( float ax, float ay, float az ) {
    sphere_centre[0] = ax;
    sphere_centre[1] = ay;
    sphere_centre[2] = az;
}

void set_draw_sphere( bool a ){ draw_sphere = a; }

void set_deform_amount( float a ) { deform_amount = a; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double eval_similarity(int method) {
    double sim_m = 0.0;

    if( method == 3 )
    {
        shshade = true;	//shape from shade is on
        zbuffer_read = true;	//read z-buffer
    }
    if( method == 4 )
    {
        zbuffer_read = true;	//read z-buffer
    }

    if (!grab_model_image(1)) return 1e12;

    //find similarity measure
    unsigned char *scaled_image = 0;
    bool scale_yes = false;
    if (method<2) {
        if( oglwidth != iwidth || oglheight != iheight )
        {
            scaled_image = new unsigned char[iheight*iwidth*4];
            gluScaleImage( GL_RGBA, oglwidth, oglheight, GL_UNSIGNED_BYTE,
                           view_image, iwidth, iheight, GL_UNSIGNED_BYTE, scaled_image);
            scale_yes = true;
        }
        else
            //cout << "scaled image = view image" << endl;
            scaled_image = view_image;

        //image to the video image
    }

    if (method == 0) {
        Similarity similarity;
        //define size
        similarity.init(iheight, iwidth );
        //define mode: 1-> correlation criteria
        similarity.init( 1 );
        //define mode: 2-> correlation criteria
        //similarity.init( 2 );
        //define RGB factors
        similarity.init( 0.4, 0.4, 0.2 );

        sim_m = similarity.findsimilarity( scaled_image, image );
//		cout << "Using Intensity Correlation" << endl;
        //calculate similarity measure
    }
    else if (method == 3 ){		//use shape from shade information: p_q space
        sim_m = ShapeFromShading();
//		cout << "Using PQ" << endl;
    }

    if (scaled_image && scale_yes ) {
        //printf("trying to delete scaled_image\n");
        delete [] scaled_image;
    }
    // calculate similarity using depth information
    if (method == 4) {
//		cout << "in similarity(4)" << endl;
        if( !zbuffer ) // input to ShapeFromShading()
        {
            printf( "ShapeFromShading: zbuffer has not been initialized\n" );
            return 0.0;
        }

        float *scaled_buffer = new float[iheight*iwidth];
        bool have_del_z = false;
        if( oglwidth != iwidth || oglheight != iheight )
            gluScaleImage( GL_DEPTH_COMPONENT, oglwidth, oglheight, GL_FLOAT,
                           zbuffer, iwidth, iheight, GL_FLOAT, scaled_buffer );
        else
        {
            delete [] scaled_buffer;
            scaled_buffer = zbuffer;
            have_del_z = true;
        }
        //printf("Updating z-buffer\n");

        Similarity similarity;
        //define size
        similarity.init(iheight, iwidth );
        //define mode: 1-> correlation criteria
        // int zNear = 1;
        // int zFar = 100;
        // convert zbuffer to true depth using near and far clipping planes
        zbuffer2TrueDepth(near_clip, far_clip, scaled_buffer);

        sim_m = similarity.findsimilarityDepth( depthImage, _minDepth, _maxDepth, scaled_buffer);
        //cout << "_minDepth=" << _minDepth << " _maxDepth=" << _maxDepth << endl;
        // 			sim_m = similarity.findsimilarityDepthMI( depthImage, _minDepth, _maxDepth, scaled_buffer );
        //cout << "Using Depth MI" << endl;
        if (!have_del_z)
            delete [] scaled_buffer;
    }
//	printf( "similarity measure = %f\n", sim_m);

    return sim_m;
}
void zbuffer2TrueDepth(float zNear, float zFar, float *scaled_depth){
    for( int i=0; i<iheight*iwidth; i++ )
    {
        //       		depthImage = (depthimage[i]-_minDepth)*99/(_maxDepth-_minDepth)+1;

        scaled_depth[i] = 2.0* zNear*zFar/(zFar+zNear-(2.0*scaled_depth[i]-1.0)*(zFar-zNear)); // opengl zbuffer range [-1,1]

        // scaling for better matching between sfs depth map, CT depth more shallow,
        // if want to output truedepth, ignore the scaling below:
        // D_CT = ((z_e<50).*z_e + (z_e>=50)*50)*2;
//        if (scaled_depth[i]<(zFar/2))
//            scaled_depth[i] = scaled_depth[i]*2;
//        else
//            scaled_depth[i] = zFar;
    }
}

/******* Access procedures for frame buffer and zbuffer used in Python *****/

/* return dimensions of rendered image and copy image to grey-scale frame buffer */
void get_view_image( int which_model, int *wd, int *ht, char **dat, int *sz ) {
    cout <<"in get_view_image(), which model = "<<which_model<<endl;
    if (!grab_model_image(which_model)) {
        *wd = 0;
        *ht = 0;
        *dat = 0;
        *sz = 0;
        return;
    }

    *wd = oglwidth;
    *ht = oglheight;
    char *buf = new char[oglwidth*oglheight];
    for (int y = 0; y<oglheight; y++)
        for ( int x = 0; x<oglwidth; x++) {
            buf[y*oglwidth+x] = view_image[4*((oglheight-1-y)*oglwidth+x)];
        }
    *dat = buf;
    *sz = oglwidth*oglheight;
}

/* return z buffer as a packed array of floats */
void get_zbuffer( int which_model, int *wd, int *ht, char **dat, int *sz ) {
    cout << "in get_zbuffer(), which model="<<which_model<<endl;
    if (!grab_model_image(which_model)) {
        *wd = 0;
        *ht = 0;
        *dat = 0;
        *sz = 0;
        return;
    }

    *wd = oglwidth;
    *ht = oglheight;
    float *zbuf = new float[oglwidth*oglheight];
    glReadPixels( 0, 0, oglwidth, oglheight, GL_DEPTH_COMPONENT, GL_FLOAT, zbuf );

//flip y-axis
    for(int y = 0; y<oglheight; y++) {
        int y1 = oglheight - 1 - y;
        if (y>=y1) break;
        for( int x = 0; x<oglwidth; x++ ) {
            float tmp = zbuf[y*oglwidth+x];
            zbuf[y*oglwidth+x] = zbuf[y1*oglwidth+x];
            zbuf[y1*oglwidth+x] = tmp;
        }
    }

    *dat = (char*)zbuf;
    *sz = sizeof(float)*oglwidth*oglheight;
}


//transform mouse coordinates so that
//result: (x0,y0) is the top left corner and the (x1,y1) is the bottom right corner
//mousecoord[0] ->x0, mousecoord[1]->y0, mousecoord[2]->x1, mousecoord[3]->y1
//x->width, y->height
void mousecoordtr( int *mousecoord )
{
    float temp;
    if ( mousecoord[0] > mousecoord[2] )
    {
        temp = mousecoord[0];
        mousecoord[0] = mousecoord[2];
        mousecoord[2] = temp;
    }

    if ( mousecoord[1] > mousecoord[3] )
    {
        temp = mousecoord[1];
        mousecoord[1] = mousecoord[3];
        mousecoord[3] = temp;
    }

}

//transform from windows to opengl mouse coordinates
//transform from windows to opengl mouse coordinates
void WinToGl( int *mousecoord, int *oglcoord )
{
    int height = oglheight;

    oglcoord[1] = height - mousecoord[1] - 1;
    oglcoord[3] = height - mousecoord[3] - 1;
    oglcoord[0] = mousecoord[0];
    oglcoord[2] = mousecoord[2];

    mousecoordtr( mousecoord );
}

//take mouse coordinates ???
void setmousecoord( int *mousecoord )
{
    mouse[0] = mousecoord[0];	mouse[1] = mousecoord[1];
    mouse[2] = mousecoord[2];	mouse[3] = mousecoord[3];
}

void getmousecoord( int *mousecoord )
{
    mousecoord[0] = mouse[0];	mousecoord[1] = mouse[1];
    mousecoord[2] = mouse[2];	mousecoord[3] = mouse[3];
}

//get 2D image from the 3D model
//scaled_image should have been allocated as:
// 	unsigned char scaled_image = new unsigned char[iheight*iwidth*3];
bool getRGBimage( unsigned char *scaled_image, int img_width, int img_height )
{
    if (!grab_model_image(1)) return false;
    unsigned char *temp = new unsigned char[oglwidth*oglheight*4];

    gluScaleImage(GL_RGBA, oglwidth, oglheight, GL_UNSIGNED_BYTE,
                  view_image, img_width, img_height, GL_UNSIGNED_BYTE, temp);

    for( int i=0; i<img_height; i++ )
    {
        for( int j=0; j<img_width; j++ )
        {
            scaled_image[(i*img_width+j)*3] = temp[((img_height-i-1)*img_width+j)*4];
            scaled_image[(i*img_width+j)*3+1] = temp[((img_height-i-1)*img_width+j)*4+1];
            scaled_image[(i*img_width+j)*3+2] = temp[((img_height-i-1)*img_width+j)*4+2];
        }
    }

    delete [] temp;

    return true;
}

float ShapeFromShading( void )
{
    printf("In ShapeFromShading()\n");
//scale mouse coordinates to image dimensions
    mousecoordtr( mouse );
    if( !mouse[0] && !mouse[1] && !mouse[2] && !mouse[3] )
    {
        mouse[0] = 0; mouse[1] = 0; mouse[2] = oglwidth-1; mouse[3] = oglheight-1;
        update_mouse( mouse );
    }
    int mousecoord[4];
    mousecoord[0] = (int)( (float)(mouse[0])*(float)(iwidth)/(float)(oglwidth) ) +1;
    mousecoord[1] = (int)( (float)(mouse[1])*(float)(iheight)/(float)(oglheight) ) +1;
    mousecoord[2] = (int)( (float)(mouse[2])*(float)(iwidth)/(float)(oglwidth) ) -1;
    mousecoord[3] = (int)( (float)(mouse[3])*(float)(iheight)/(float)(oglheight) ) -1;

    float step[2] = { zoomFactorx, zoomFactory };

    if( pq_model )
        pq_model->clear();
    else
        pq_model = new vector<GLMpoint_d>;

    if( !zbuffer ) // input to ShapeFromShading()
    {
        printf( "ShapeFromShading: zbuffer has not been initialized\n" );
        return 0.0;
    }

    float *scaled_buffer = new float[iheight*iwidth];
    bool have_del_z = false;
    if( oglwidth != iwidth || oglheight != iheight )
        gluScaleImage( GL_DEPTH_COMPONENT, oglwidth, oglheight, GL_FLOAT,
                       zbuffer, iwidth, iheight, GL_FLOAT, scaled_buffer );
    else
    {
        delete [] scaled_buffer;
        scaled_buffer = zbuffer;
        have_del_z = true;
    }
//printf("Updating z-buffer\n");
    vector<long> inf_vector;
// camera parameters how info calculated??
    float info[4] = {2.0*near_clip*aspect/focal, 2.0*near_clip/focal, iwidth, iheight };
//cout << "mouse="<<mouse[0]<<" "<<mouse[1]<<" "<<mouse[2]<<" "<<mouse[3]<<endl;
//cout << "mousecoord="<< mousecoord[0]<<" "<<mousecoord[1]<<" "<<mousecoord[2]<<" "<<mousecoord[3]<<endl;
// calculate pq_model calling shapeshade.cpp function
// input: scaled_buffer, output: pq_model
    float x,y,z,rx,ry,rz;
    get_pose_values( &x, &y, &z, &rx, &ry,&rz );
    char filename[200]; // make sure it's big enough

    snprintf(filename, sizeof(filename), "/Users/malishen/Desktop/zbuffer_%f_%f_%f_%f_%f_%f.txt", x,y,z,rx,ry,rz);
    bool flag = pqmodel( scaled_buffer, pq_model, &inf_vector, mousecoord, info, focal, iwidth, near_clip, far_clip, filename);

// #ifdef PRINT_MODE		
// 	int wid_heigtmp[2] = { iwidth, iheight};
// 	// save pq_model to file
// 	store_2D_img( pq_model, wid_heigtmp, "/Users/malishen/Documents/reg3d2d/SFS_pq_model_x.txt",
// 								 	   "/Users/malishen/Documents/reg3d2d/SFS_pq_model_y.txt" );
// #endif

    if( !flag )
        return MAX;

/*	vector<GLMpoint_d> *pq_modeln = new vector<GLMpoint_d>;
double min_obj, max_obj;
pq_scale( pq_model, pq_modeln, &min_obj, &max_obj );
delete pq_model;
pq_model = pq_modeln;
*/
//	PQtoFile( "C:\\doNTdelete\\project\\data\\output\\pq_space\\model.txt", pq_model, mousecoord );

    delete [] scaled_buffer;
    if( have_del_z )
        zbuffer = NULL;

    if( !pq_img )
    {
        pq_img = new vector<GLMpoint_d>;
        pq_image( image, pq_img, mousecoord,  iwidth, iheight, step, focal );

// #ifdef PRINT_MODE		
// 		int wid_heig[2] = { iwidth, iheight};
// 		// save pq_img to file
// 		store_2D_img( pq_img, wid_heig, "/Users/malishen/Documents/reg3d2d/SFS_pq_img_x.txt",
// 								 	"/Users/malishen/Documents/reg3d2d/SFS_pq_img_y.txt" );
// #endif
/*		vector<GLMpoint_d> *pq_imgn = new vector<GLMpoint_d>;
//		double min_img, max_img;
//		pq_scale( pq_img, pq_imgn, &min_img, &max_img );
//		delete pq_img;
pq_img = pq_imgn;
*/
    }

//	PQtoFile( "C:\\doNTdelete\\project\\data\\output\\pq_space\\photo.txt", pq_img, mousecoord );


    vector<float> deform_vector;
    if( deformation )
    {
        static int frame = 0;
        frame++;
        char *bas_path1 = "C:\\doNTdelete\\project\\data\\output\\disparity\\deform";
        char tmp_path[10];
//_itoa( frame, tmp_path, 10 );
        int len1 = strlen( bas_path1 );
        int len_tmp = strlen( tmp_path );
        char *path1 = new char[len1+len_tmp+4+1];
        strcpy( path1, bas_path1 );
        strcat( path1, tmp_path );
        strcat( path1, ".txt" );
        FloatfromFile( path1, &deform_vector );
        delete [] path1;
    }
    double similarity;
    if( deformation ) {
        printf("sim_deform_OF\n");
        similarity = sim_deform_OF( pq_model, pq_img, &deform_vector, &inf_vector );
    }
    else {
        printf("sim_rigid\n");
        similarity = sim_rigid( pq_model, pq_img, &inf_vector );
    }

    inf_vector.clear();
    deform_vector.clear();

    return similarity;
}

void clear_shape_info( void )
{
    if( pq_model )
    {
        delete pq_model;
        pq_model = NULL;
    }
    if( pq_img )
    {
        delete pq_img;
        pq_img = NULL;
    }
    if( pq_img_bef )
    {
        delete pq_img_bef;
        pq_img_bef = NULL;
    }

    deform_vector.clear();

}

void DefineSphereCenter()
{
//define the sphere center
    int oglmousecoord[4];
    WinToGl( mouse, oglmousecoord );
    zbuffer_read = true;
    GetTransf = true;
    reinit_display();
    not_grab_model_image(0);
    float zvalue = zbuffer[oglwidth*oglmousecoord[1]+oglmousecoord[0]];

    GLdouble obj[3];
    gluUnProject( oglmousecoord[0], oglmousecoord[1], zvalue,
                  modelviewmatrix, projectionmatrix, viewport,
                  &obj[0], &obj[1], &obj[2] );

    sphere_centre[0] = obj[0]; sphere_centre[1] = obj[1]; sphere_centre[2] = obj[2];
}

void ApplyDeformation() {
    RestoreDef( pmodel[0], 0 );
    ApplyDeformationSphere( pmodel[0], radius, sphere_centre, deform_amount );
    RestoreDef( pmodel[1], 1 );
    ApplyDeformationSphere( pmodel[1], radius, sphere_centre, deform_amount );
}

void RestoreOriginal(){
    RestoreDef( pmodel[0], 0 );
    RestoreDef( pmodel[1], 1 );
}

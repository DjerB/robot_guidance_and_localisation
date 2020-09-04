#include <stdio.h>
#include <fstream>
#include "FL/Fl_File_Chooser.h"

//#include "python_d.h"
#include "scene_nodes.h"
#include "scenegraph.h"
#include "projection.h"
#include "registration.h"
#include "praxis.h"
#include "main.h"
#include "glm.h"

//extern "C" void init_reg2d3d();

typedef vector<float*> vector_float_point;
typedef vector<int> vector_int_point;


static Registration2D3D* reg_gui; // pointer to the interface
static const int dig = 4;

//registration of video frames ??
static Reg_vid_fram_in_out RegVidPar;
static vector<float*> *reg_res_vect = NULL;

//registration of video frames with 1 DOF
static vector<float*> *track_data = NULL;
static vector<int> *num_frame_tr = NULL;
static float current_pose[6] = { 0, 0, 0, 0, 0, 0 }; // x y z rx ry rz

//store 2D images from the 3D model
static StoreFrame Store3DFrames;  // ??

int main( int argc, char* argv[] ) {
    // Py_Initialize();

//   init_scenegraph();
//   init_reg2d3d();

//   FILE* fp = fopen("reginit.py","r");
//   if (!fp) {
// 	  fprintf( stderr, "python initialisation file failed\n");
// 	  exit(-1);
//   }
// //  if (PyRun_SimpleFile(fp,"reginit.py")) exit(-1);
// 	PyRun_SimpleString("import sys");
// 	PyRun_SimpleString("sys.path.append('.')");
// 	if (PyRun_SimpleString("from reginit import *")) exit(-1);
// 	fclose(fp);
    set_focal_length(2.9); // projection.h
    //cout << "set focal length"<<endl;
    set_attenuation( .41, .00073, .000022 );
    //cout << "set attenuation"<<endl;

    init_models(2);
    /******** Hamlyn Synposium 2018 ********/
    /* silicon lung phantom validation */
    // load_model(0,"/Users/malishen/Dropbox/Symposium2018/PhantominEM/MeshPhantom_20151113Smooth.off");
    // load_model(1,"/Users/malishen/Dropbox/Symposium2018/PhantominEM/MeshPhantom_20151113Smooth.off");
//    load_model(0,"/Users/malishen/Documents/MICCAI2016DesktopFiles/Airway_Phantom_AdjustSmooth.off");
//    load_model(1,"/Users/malishen/Documents/MICCAI2016DesktopFiles/Airway_Phantom_AdjustSmooth.off");
//    load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Undistorted_board2_enhanced_ppm/00109_rect.ppm"); // sequence 1 from 109, when Reg, load from 110_rect
////    load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Undistorted_board2_enhanced_ppm/00394_rect.ppm"); // sequence 1 from 394, when Reg, load from 395_rect
//    loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/depth_board2_enhanced/00109_rect.txt",true);
    /* case 1*/
      load_model(0,"/Users/malishen/Documents/MICCAI2016DesktopFiles/BronchiLastYearNoSmoothingadjusted4.off");
     load_model(1,"/Users/malishen/Documents/MICCAI2016DesktopFiles/BronchiLastYearNoSmoothingadjusted4.off");
//     load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Sequence2Sorted_ppm/051_rect.ppm"); // case 1 seq2 miccai 2017 new validation
//     loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/DepthSequence2Sorted/00051_rect.txt",true);
     load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Sequence1Sorted_ppm/00352_rect.ppm"); // case 1 seq1 miccai 2017 new validation
   loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/DepthSequence1Sorted/00352_rect.txt",true);

    /*********** case 1 *******************/
    // BF260
    // Focal Length:          fc = [ 213.17558   217.16548 ] ± [ 3.49401   3.63654 ]
    // Principal point:       cc = [ 161.53330   154.13610 ] ± [ 1.70835   1.05239 ]
    //  load_model(0,"/Users/malishen/Documents/MICCAI2016DesktopFiles/BronchiLastYearNoSmoothingadjusted4.off");
    // load_model(1,"/Users/malishen/Documents/MICCAI2016DesktopFiles/BronchiLastYearNoSmoothingadjusted4.off");
    // load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Sequence2Sorted_ppm/051_rect.ppm"); // case 1 seq2 miccai 2017 new validation
    // loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/DepthSequence2Sorted/00051_rect.txt",true);
    // load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Sequence1Sorted_ppm/00352_rect.ppm"); // case 1 seq1 miccai 2017 new validation
//   loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/DepthSequence1Sorted/00352_rect.txt",true); 
    // load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Sequence1Sorted_ppm/00001_rect.ppm");
//   loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/DepthSequence1Sorted/00001_rect.txt",true); 
// 
//   load_image("/Users/malishen/Desktop/Sequence2Sorted_ppm/119_rect.ppm");
//   loadDepthFile("/Users/malishen/Desktop/DepthSequence2SortedREG/119_rect.txt",true); 
//   set_pose_values(-267.7,-210.4,131.39,19.51,35.95,-134.84); 
    /*************** case 2 **************/
    // BF260F
    // load_model(0,"/Users/malishen/OneDriveMali/OneDrive\ -\ Imperial\ College\ London/DrShahNavigationCases/2/BronchiMesh2.off");
//   load_model(1,"/Users/malishen/OneDriveMali/OneDrive\ -\ Imperial\ College\ London/DrShahNavigationCases/2/BronchiMesh2.off");
//   load_image("/Users/malishen/Desktop/Case2/Sequence1Sorted_ppm/001_rect.ppm");
//    loadDepthFile("/Users/malishen/Desktop/Case2/DepthSequence1SortedREG/001_rect.txt",true); 
    // load_image("/Users/malishen/Desktop/Sequence2Sorted_ppm/001_rect.ppm");
//   loadDepthFile("/Users/malishen/Desktop/DepthSequence2SortedREG/001_rect.txt",true); 


    /********** silicon lung phantom validation ***********/
    // load_model(0,"/Users/malishen/Documents/MICCAI2016DesktopFiles/Airway_Phantom_AdjustSmooth.off");
    // load_model(1,"/Users/malishen/Documents/MICCAI2016DesktopFiles/Airway_Phantom_AdjustSmooth.off");
    // load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Undistorted_board2_enhanced_ppm/01077_rect.ppm"); // seq1 part 2 new
    //  loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/depth_board2_enhanced/01077_rect.txt",true);
    // load_image("/Users/malishen/Documents/MICCAI2016DesktopFiles/Undistorted_board2_enhanced_ppm/00159_rect.ppm"); // seq1 part 1 new
//    loadDepthFile("/Users/malishen/Documents/MICCAI2016DesktopFiles/depth_board2_enhanced/00159_rect.txt",true); 
//  

    /*      Phantom validation     */
    // load_model(0,"/Users/malishen/Documents/Fani_PhantomData/model_phantom/newmodel/phantom0-0.5-deskin.noff");
//   load_model(1,"/Users/malishen/Documents/Fani_PhantomData/model_phantom/newmodel/phantom0-0.5-deskin.noff");
//   load_image("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-c/150-1150/frame_0000.ppm");
//   loadDepthFile("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-c/150-1150depth_corrected_ccx_ccy/frame_0000.txt",false); 
//   

    /*      in vivo validation  Fani    */
    // load_model(0,"/Users/malishen/Documents/reg3d2d/D97193.noff");
//   load_model(1,"/Users/malishen/Documents/reg3d2d/D97193.noff");
//   load_image("/Volumes/MALISHEN/Fani/D97193_frames_ppm/D97193_00429_rect.ppm");
//   loadDepthFile("/Volumes/MALISHEN/Fani/D97193_frames_Depth/D97193_00429_rect.txt",true); // false=no flip
//   
    /*      in vivo validation 2  (case 1)*/
    // load_model(0,"/Users/malishen/Documents/reg3d2d/DICOMBronchiMesh.off");
//   load_model(1,"/Users/malishen/Documents/reg3d2d/DICOMBronchiMesh.off");

    // load_model(0,"/Users/malishen/Documents/reg3d2d/BronchiLastYearNoSmoothingadjusted4.off");
//   load_model(1,"/Users/malishen/Documents/reg3d2d/BronchiLastYearNoSmoothingadjusted4.off");

//   load_image("/Volumes/MALISHEN/Pallav/Case1/2776_2834_ppm_rect/0000_rect.ppm");
// load_image("/Volumes/MALISHEN/DrShahNavigationCases/1/RB123/PPM/00_rect.ppm");
// loadDepthFile("/Volumes/MALISHEN/DrShahNavigationCases/1/RB123/SFS/00_rect.txt",true);

/***** case 3 *****/
// load_model(0,"/Volumes/MALISHEN/DrShahNavigationCases/3/Bronchisnap.off");
//   load_model(1,"/Volumes/MALISHEN/DrShahNavigationCases/3/Bronchisnap.off");
//   
// //   load_image("/Volumes/MALISHEN/Pallav/Case1/2776_2834_ppm_rect/0000_rect.ppm");
// // load_image("/Volumes/MALISHEN/DrShahNavigationCases/3/RB3abi/UndistortFrames_ppm/00_rect.ppm");
// // loadDepthFile("/Volumes/MALISHEN/DrShahNavigationCases/3/RB3abi/SFS/00_rect.txt",true);
// 
// // load_image("/Volumes/MALISHEN/DrShahNavigationCases/3/LB6abc/UndistortFrames_ppm/00_rect.ppm");
// // loadDepthFile("/Volumes/MALISHEN/DrShahNavigationCases/3/LB6abc/SFS/00_rect.txt",true);
// //   
//   load_image("/Volumes/MALISHEN/DrShahNavigationCases/3/UndistortValidationPPM/000_rect.ppm");
// loadDepthFile("/Volumes/MALISHEN/DrShahNavigationCases/3/LB6abc/SFS/00_rect.txt",true);
//  

/****** Pig Lung Case *******/
    //  load_model(0,"/Users/malishen/Documents/PigLungDataForReg2d3d/LungSegMatlab.off");
//   load_model(1,"/Users/malishen/Documents/PigLungDataForReg2d3d/LungSegMatlab.off");
//   load_image("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-c/150-1150/frame_0000.ppm");
//   loadDepthFile("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-c/150-1150depth_corrected_ccx_ccy/frame_0000.txt",false); 
//   


    //load_model(0,"/Users/aapple/Documents/linuxCode/projection/src/m54.off");
    //load_model(0,"/Users/aapple/Documents/linuxCode/reg3d2d/RE__lighting_and_camera_calibration_GroundTruthPose\&VideoFrame/d97193_strips.noff");
    //cout << "first model loaded.."<<endl;
    //load_model(1,"/Users/aapple/Documents/linuxCode/projection/src/m54.off");
    //load_model(1,"/Users/aapple/Documents/linuxCode/reg3d2d/RE__lighting_and_camera_calibration_GroundTruthPose\&VideoFrame/d97193_strips.noff");
    //cout << "second model loaded.."<<endl;
    //load_image("/Users/aapple/Documents/linuxCode/reg3d2d/D97193_photos/frame_00002.ppm");
    //load_image("/Users/aapple/Documents/linuxCode/reg3d2d/2776_2834_ppm/0020.ppm");
    //load_image("/Users/malishen/Documents/reg3d2d/D97193_1186_1237/D97193_1186_0021_rect.ppm");
    //load_image("/Users/malishen/Documents/MATLAB/OldMac/TOOLBOX_calib/rect_ppm/D97193_1186_0000_rect.ppm");
    //load_image("/Users/malishen/Documents/Fani_PhantomData/video-journal/vid20030801/photos-b/465-1500/frame_0000.ppm");
    //loadDepthFile("/Users/malishen/Documents/MATLAB/OldMac/TOOLBOX_calib/rect_depth/D97193_1186_0000depth_rectChangedccxccy.txt");
    //load_centreline("/Users/aapple/Documents/linuxCode/reg3d2d/skeleton_scaled.txt");




    reg_gui = new Registration2D3D();
    set_sliders(0);
    reg_gui->param1->value(1);
    reg_gui->show(argc,argv);
    reg_gui->viewer->show();
    //reg_gui->plotter->show();

    /********* Hamlyn Symposium 2018 **************/
    /*********** silicon lung phantom **********/
//    set_pose_values(39.94,-8.65,-148.48,27.73,105.78,-32.69); // phantom sequence 1, starting from 109 (wrong orientation at main bronchi bifurcation, do not use)
//    set_pose_values(37.56,-5.12,-142.54,-19.51,97.57,151.18); // phantom sequence 1, starting from 109
//     set_pose_values(-261.757,-186.9760,147.1610,25.68,70.86,-91.94); // case 1: seq 2 from frame 51 (wrong)
//    set_pose_values(-260.63,-186.38,146.78,48.27,1.03,-112.37); // case 1: seq 2 from frame 51

//   set_pose_values(-224.29,-261.76,59.91,25.68,70.86,-91.94); // case 1: seq 1 from frame 352 (wrong old value)
    set_pose_values(-222.25,-255.43,54.46,25.68,58.54,-77.63); // case 1: seq 1 from frame 352 (new value)


    /*      Phantom validation     */
    //set_pose_values(16.13,-7.01,-81.36,-1.03,-21.57,34.73);    // phantom-c
    //set_pose_values(-30.24,6.40,-94.06,-7.19,21.57,-149.14);

    /*      in vivo validation      */
//   set_pose_values(136.22,189.12,-2.69,-185.89,-5.14,165.48);   // D97193.noff pose 429
    //set_pose_values(129.31,185.15,-19.47,167.41,-11.3,187.96);   // D97193.noff pose 529
    //set_pose_values(132.38,189.4,-19.47,169.46,-13.35,-175.7);   // D97193.noff pose 629
    //set_pose_values(129.31,185.15,-20.87,169.46,-15.41,-177.74);   // D97193.noff pose 729
    //set_pose_values(128.22,185.12,-22.69,-187.89,-19.14,179.48);   // D97193.noff pose 899
    //set_pose_values(133.81,187.413,-9.31716,-177.043,-52.5955,195.42);   // D97193.noff pose 806 MI
    //set_pose_values(135.947,180.779,0.326607,-188.326,-15.6751,253.701);   // D97193.noff pose 742 CC

/********* Pig Lung *********/
// set_pose_values(215.63,143.24,322.41,3.08,177.68,-4.09);   // entry of the tube


/*  case 1 */
// Camerafocalpoint x = -341.932 y = -167.568 z = 216.132
// CameraLocation x = -224.223 y = -261.796 z = 59.8417
// CameraViewUp x = -0.760434 y = 0.524938 z = -0.382334
// set_pose_values(-224.223,-261.796,59.8417,29.78,64.70,-81.72);   // case 1 sequence 1 initial pose








// 	set_pose_values(132.220001,192.119995,-30.690002,-171.889999,-27.139999,201.479996);// D97193.noff pose1420

    /*      in vivo validation Pallav   */

// set_pose_values(11.71,140.21,994.27,-146.86,9.24,44.95);   // DICOMBronchiMesh.off 2776_2834 pose 0000 Fani's calibration
// 	set_pose_values(10.57,138.87,996.65,-150.97,33.89,49.03);   // DICOMBronchiMesh.off 2776_2834 pose 0000 Fani's Fc Pallav's cc
// 	


    // generateProjectionsIntensityPowell();

    /**************** New three cases *********/
//   set_pose_values(-224.29,-261.76,59.91,25.68,70.86,-91.94); // case 1: seq 1 from frame 352
    // set_pose_values(-261.757,-186.9760,147.1610,25.68,70.86,-91.94); // case 1: seq 2 from frame 51
//   set_pose_values(-262.65,-199.89,111.61,83.19,-165.35,-173.66); // case 1: seq 1 RB123
//   set_pose_values(-267.7,-210.4,131.39,19.51,35.95,-134.84); // case 1 seq 2
// case 1 seq 2 resume tracking 
// set_pose_values(-263.276,-221.737,124.684,168.572,-81.7972,-333.26); // from frame 340
// set_pose_values(-267.657,-196.575,142.38,168.572,-81.7972,-333.26); // from frame 120
// set_pose_values(-269.763,-191.658,142.004,259.341,-113.022,274.067); // reverse from 190 to 1
//   set_pose_values(-279.989,-221.966,45.5259,19.51,35.95,-134.84);  // case 2: seq1 
/*********** silicon lung phantom **********/
    // set_pose_values(12.1,-1.25,-145.85,-153.03,-72.92,-28.60); // seq1 section 1 new validation miccai 2017
    // set_pose_values(10.04,-5.49,-155.81,259.341,-113.022,274.067); // seq1 section 2 new validation miccai 2017

    //set_pose_values(133.1,196,-15,176,12.98,14.84);   // D97193.noff pose
    //set_pose_values(16.27,127.67,980.02,-9.24,190,38.82);   // D97193.noff pose
    //set_pose_values(127.160004,185.149994,-22.270000,175.619995,-7.190000,-132.490005);
    //load_pose("/Users/aapple/Documents/linuxCode/reg3d2d/RE__lighting_and_camera_calibration_GroundTruthPose\&VideoFrame/poses/pose_1196.pose");
//load_pose("/Users/aapple/Documents/linuxCode/reg3d2d/1196newPose.pose");
// 	eval_similarity(3);
// 	eval_similarity(0);
// 	//load_pose("/Users/aapple/Documents/linuxCode/reg3d2d/RE__lighting_and_camera_calibration_GroundTruthPose\&VideoFrame/poses/pose_1190.pose");
// 	load_pose("/Users/aapple/Documents/linuxCode/reg3d2d/1190newPose.pose");
// 	eval_similarity(3);
// 	eval_similarity(0);
    //load_pose("/Users/malishen/Documents/reg3d2d/D97193_1186_1237_pose/1186newpose.pose");
    // eval_similarity(3);
// 	eval_similarity(0);
    //set_pose_values(131.61,187.70,-29.260000,177.68,-21.570000,-130.750000); // pose_1196
    //set_pose_values(133.15,186.85,-27.860000,177.68,-21.570000,-130.750000);
    //set_pose_values(131.61,191.10,-11.08,-185.89,-11.30,181.83);
// 	cout << "set pose-initialise"<<endl;
    //eval_similarity(3);
    //eval_similarity(0);


    // case 3 validation start pose
// 	set_pose_values(-292.74,-320.54,301.50,-66.76,70.86,75.59);



    int value = Fl::run();
    cout << "return value" << value << endl;
    //delete_models();
    delete_image();
    return value;
}


/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////--------Optimization--------//////////////////////////////////
static bool abort_praxis = false;
void set_abort_praxis( bool a ){ abort_praxis = a; }
/////////////////////////////////////////////////////////////////////////////////////////////

/**** Minimisation for pose estimation ****/
const double afac = 1.0;
const double opt_dfac = 8.0;		//positional displacement
const double opt_afac = 2.0;	//angle orientation
const double opt_zfac = 10.0;	//z-axis rotation
double test_function(double *x, int n) {

//	set_pose_values( x[0], x[1], x[2], afac*x[3], afac*x[4], afac*x[5] );
//	set_pose_values( afac*x[0], afac*x[1], afac*x[2], x[3], x[4], x[5] );
    set_pose_values( opt_dfac*x[0], opt_dfac*x[1], opt_dfac*x[2],
                     opt_afac*x[3], opt_afac*x[4], opt_zfac*x[5] );
    int method = get_method();
    double sim_m = (double)eval_similarity( method );
    Fl::check();
    return sim_m;
}

static bool optimising = false;
void optimise( )
{
    if (optimising) return;
    optimising = true;
    int n = 6;
    float temp[6];
    get_pose_values( &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5] );
/*	double x[6] = { (double)temp[0], (double)temp[1], (double)temp[2],
		(double)temp[3]/afac, (double)temp[4]/afac, (double)temp[5]/afac };
*/
//	double x[6] = { (double)temp[0]/afac, (double)temp[1]/afac, (double)temp[2]/afac,
//	   	(double)temp[3], (double)temp[4], (double)temp[5] };
    double x[6] = { (double)temp[0]/opt_dfac, (double)temp[1]/opt_dfac, (double)temp[2]/opt_dfac,
                    (double)temp[3]/opt_afac, (double)temp[4]/opt_afac, (double)temp[5]/opt_zfac };
    (void)praxis( test_function, x, n);
//	set_pose_values( x[0], x[1], x[2], afac*x[3], afac*x[4], afac*x[5] );
    set_pose_values( opt_dfac*x[0], opt_dfac*x[1], opt_dfac*x[2],
                     opt_afac*x[3], opt_afac*x[4], opt_zfac*x[5] );

    optimising = false;
}


void optimise_video()
{
    char *basic_path;
    char *out_path;
    int init_frame;
    int final_frame;
    int step;
    get_info_start( &init_frame, &final_frame, &step, &basic_path, &out_path );

    float temp[6];
    get_pose_values( &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5] );

    int num_frames = final_frame - init_frame +2;
    //put initial pose parameters to file
    ofstream fout(out_path);
    if (!fout) return;
    fout << temp[0] << ' ' << temp[1] << ' ' << temp[2] << ' ' << temp[3] << ' ' <<temp[4] << ' ' <<temp[5] << endl;

    int len_bas = strlen( basic_path );
    for( int i= init_frame; i<=final_frame; i+=step )
    {
        //construct path name
        char tmp_path[10];
        //_itoa( i, tmp_path, 10 );
        int len_tmp = strlen( tmp_path );
        char *full_path = new char[len_bas+dig+4+1];
        strcpy( full_path, basic_path );
        int dif = dig - len_tmp;
        for( int j=0; j<dif; j++ )
            strcat(full_path, "0");
        strcat( full_path, tmp_path );
        strcat( full_path, ".ppm" );

        //load_image
        load_image( full_path );
        delete [] full_path;

        optimise();
        //eval_similarity(3);

        //put each intermediate optimize pose parameters to a file
        get_pose_values( &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5] );
        fout << temp[0] << ' ' << temp[1] << ' ' << temp[2] << ' ' << temp[3] << ' ' <<temp[4] << ' ' <<temp[5] << endl;
    }
    fout.close();

    clear_shape_info();
}

int load_reg_result( const char *out_path )
{
    ifstream fin(out_path);
    if (!fin) return 0;

    if( reg_res_vect )
    {

        for( vector_float_point::iterator iter=reg_res_vect->begin(); iter!=reg_res_vect->end(); iter++ )
            delete [] *iter;

        reg_res_vect->clear();
    }
    else
        reg_res_vect = new vector<float*>;

    float *pose;
    while( fin.good() )
    {
        pose = new float[6];
        fin >> pose[0] >> pose[1] >> pose[2] >> pose[3] >> pose[4] >> pose[5];
        if( fin.good() )
            reg_res_vect->push_back(pose);
    }
    fin.close();

    int num_frames = reg_res_vect->size();
    return num_frames;
}

//optimisation - 1DOF
bool load_track_data( )
{
    char *path = choose_input_file();

    if( !path )
        return false;

    //read track data
    ifstream fin(path);
    if (!fin) return false;

    if( track_data )
    {
        for( vector_float_point::iterator iter=track_data->begin(); iter!=track_data->end(); iter++ )
            delete [] *iter;

        delete track_data;
    }
    else
        track_data = new vector<float*>;

    if( num_frame_tr )
        delete num_frame_tr;
    else
        num_frame_tr = new vector<int>;

    float *pose;
    while( fin.good() )
    {
        pose = new float[5];	//pose[0-4]->5DOF
        int frame;		//number of frame
        float temp;
        fin >> temp >> pose[0] >> pose[1] >> pose[2] >> pose[3] >> pose[4] ;
        frame = (int)(temp);
        if( fin.good() )
        {
            track_data->push_back(pose);
            num_frame_tr->push_back(frame);
        }
    }
    fin.close();

    return true;
}

char* choose_input_file() {
    char* fc = fl_file_chooser( "Load Track Data", "*.{txt}", "" );
    return fc;
}

void optimise_video1DOF()
{
    char *basic_path;
    char *out_path;
    int init_frame;
    int final_frame;
    int step;
    get_info_start( &init_frame, &final_frame, &step, &basic_path, &out_path );

    float temp[6];
    get_pose_values( &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5] );

    if( !track_data || !num_frame_tr ){
        printf( "optimise_video1DOF: track info is NULL" );
        return;
    }
    int num_frames = track_data->size();

    //put initial pose parameters to file
    ofstream fout(out_path);
    if (!fout) return;
    fout << temp[0] << ' ' << temp[1] << ' ' << temp[2] << ' ' << temp[3] << ' ' <<temp[4] << ' ' <<temp[5] << endl;

    vector_float_point::iterator iter_pose = track_data->begin();
    vector_int_point::iterator iter_frame = num_frame_tr->begin();
    int len_bas = strlen( basic_path );
    for( int i= 0; i<num_frames; i+=step )
    {
        float *pose = iter_pose[i];
        for( int j=0; j<5; j++ )
            current_pose[j] = pose[j];
        current_pose[5] = temp[5];
        int frame = iter_frame[i];
        //construct path name
        char tmp_path[10];
        //_itoa( frame, tmp_path, 10 );
        int len_tmp = strlen( tmp_path );
        char *full_path = new char[len_bas+dig+4+1];
        strcpy( full_path, basic_path );
        int dif = dig - len_tmp;
        for(int j=0; j<dif; j++ )
            strcat(full_path, "0");
        strcat( full_path, tmp_path );
        strcat( full_path, ".ppm" );

        //load_image
        load_image( full_path );
        delete [] full_path;

        optimise1DOF();

        //put each intermediate optimize pose parameters to a file
        get_pose_values( &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5] );
        fout << temp[0] << ' ' << temp[1] << ' ' << temp[2] << ' ' << temp[3] << ' ' <<temp[4] << ' ' <<temp[5] << endl;
    }
    fout.close();

    for(int i=0; i<6; i++ )
        current_pose[i] = 0.0f;
    clear_shape_info();
}

double test_function1DOF(double *x, int n) {
    current_pose[5] = *x;
    set_pose_values( current_pose[0], current_pose[1], current_pose[2],
                     current_pose[3], current_pose[4], current_pose[5] );
    int method = get_method();
    double sim_m = (double)eval_similarity( method );
    Fl::check();
    return sim_m;
}

void optimise1DOF( )
{
    if (optimising) return;
    optimising = true;
    int n = 1;
    float temp[6];
    get_pose_values( &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5] );
    double x[1] = { (double)temp[5] };
    (void)praxis( test_function, x, n);
    set_pose_values( current_pose[0], current_pose[1], current_pose[2],
                     current_pose[3], current_pose[4], current_pose[5] );

    optimising = false;
}

//end of optimisation - 1DOF


void moveslider(int a)
{
    char *basic_path;
    char *out_path;
    int init_frame;
    int final_frame;
    int step;
    get_info_start( &init_frame, &final_frame, &step, &basic_path, &out_path );

    vector_float_point::iterator iter=reg_res_vect->begin();
    float *pose = iter[a-1];
    set_pose_values( pose[0], pose[1], pose[2], pose[3], pose[4], pose[5] );

    int frame;
    if( a == 1 )
        frame = init_frame;
    else
        frame = a+init_frame-2;
    //construct path name
    int len_bas = strlen( basic_path );
    char tmp_path[10];
    //_itoa( frame, tmp_path, 10 );
    int len_tmp = strlen( tmp_path );
    char *full_path = new char[len_bas+dig+4+1];
    strcpy( full_path, basic_path );
    int dif = dig - len_tmp;
    for( int j=0; j<dif; j++ )
        strcat(full_path, "0");
    strcat( full_path, tmp_path );
    strcat( full_path, ".ppm" );

    //load_image
    load_image( full_path );
    delete [] full_path;

    reg_gui->viewer->draw();
}

//display poses....
void next_pose( int a )
{
    vector_float_point::iterator iter=reg_res_vect->begin();
    float *pose = iter[a-1];
    set_pose_values( pose[0], pose[1], pose[2], pose[3], pose[4], pose[5] );
    reg_gui->viewer->draw();
}

//store 2D image from the 3D model
void storePPMimage( char *path, int imgwidth, int imgheight )
{
    unsigned char *scaled_image = new unsigned char[imgwidth*imgheight*3];
    bool ok = getRGBimage( scaled_image, imgwidth, imgheight );
    if ( ok )
        glmWritePPM( path, scaled_image, imgwidth, imgheight );
}

void storeFrame()
{
    //Get parameters
    int init_frame;
    int final_frame;
    int step;
    char *output_path;
    int imgwidth;
    int imgheight;
    get_store_info( &init_frame, &final_frame, &step, &output_path, &imgwidth, &imgheight );

    StoreFrameMain( output_path, init_frame, imgwidth, imgheight );

    //store pose data
    float x, y, z, rx, ry, rz;
    get_pose_values( &x, &y, &z, &rx, &ry, &rz );

    //construct path name
    int len_bas = strlen( output_path );
    char tmp_path[10];
    //_itoa( init_frame, tmp_path, 10 );
    int len_tmp = strlen( tmp_path );
    char *full_path = new char[len_bas+dig+4+1];
    strcpy( full_path, output_path );
    int dif = dig - len_tmp;
    for( int j=0; j<dif; j++ )
        strcat(full_path, "0");
    strcat( full_path, tmp_path );
    strcat( full_path, ".txt" );

    ofstream fout(full_path);
    if (!fout) return;
    fout << x << ' ' << y << ' ' << z << ' ' << rx << ' ' <<ry << ' ' <<rz << endl;
    fout.close();

    delete [] full_path;
}

void StoreAllFrames()
{
    //Get parameters
    int init_frame;
    int final_frame;
    int step;
    char *output_path;
    int imgwidth;
    int imgheight;
    get_store_info( &init_frame, &final_frame, &step, &output_path, &imgwidth, &imgheight );

    if( !reg_res_vect )
        return;

    vector_float_point::iterator iter=reg_res_vect->begin();
    int len_vector = reg_res_vect->size();

    if( init_frame<0 || final_frame<0 || final_frame>=len_vector || final_frame-init_frame+1>len_vector )
        return;

    //construct path name
    int len_bas = strlen( output_path );
    char tmp_path1[4], tmp_path2[4];
    //_itoa( init_frame, tmp_path1, 10 );
    //_itoa( final_frame, tmp_path2, 10 );
    int len_tmp1 = strlen( tmp_path1 );
    int len_tmp2 = strlen( tmp_path2 );
    char *full_path = new char[len_bas+len_tmp1+1+len_tmp2+4+1];
    strcpy( full_path, output_path );
    strcat( full_path, tmp_path1 );
    char *tmp_char = "-";
    strcat( full_path, tmp_char );
    strcat( full_path, tmp_path2 );
    strcat( full_path, ".txt" );

    ofstream fout(full_path);
    if (!fout) return;

    for( int i=init_frame; i<=final_frame; i+=step )
    {
        float *pose = iter[i];
        set_pose_values( pose[0], pose[1], pose[2], pose[3], pose[4], pose[5] );
        Fl::check();
        StoreFrameMain( output_path, i, imgwidth, imgheight );
        get_pose_values( &pose[0], &pose[1], &pose[2], &pose[3], &pose[4], &pose[5] );
        fout << pose[0] << ' ' << pose[1] << ' ' << pose[2] << ' ' << pose[3] << ' ' <<pose[4] << ' ' <<pose[5] << endl;
    }
    fout.close();
}

void StoreFrameMain( char *output_path, int frame, int imgwidth, int imgheight )
{
    //construct path name
    int len_bas = strlen( output_path );
    char tmp_path[10];
    //_itoa( frame, tmp_path, 10 );
    int len_tmp = strlen( tmp_path );
    char *full_path = new char[len_bas+dig+4+1];
    strcpy( full_path, output_path );
    int dif = dig - len_tmp;
    for( int j=0; j<dif; j++ )
        strcat(full_path, "0");
    strcat( full_path, tmp_path );
    strcat( full_path, ".ppm" );

    //store image
    storePPMimage( full_path, imgwidth, imgheight );

    delete [] full_path;
}

int get_method( void )
{
    return reg_gui->reg_method->value();
}

//update mouse coordinates
void update_mouse( int *mousecoord ){
    reg_gui->win_x0->value( mousecoord[0] );
    reg_gui->win_y0->value( mousecoord[1] );
    reg_gui->win_x1->value( mousecoord[2] );
    reg_gui->win_y1->value( mousecoord[3] );
}

//Display updates

void update_display() {
    reg_gui->plotter->redraw();
    Fl::check();
}

int param(int i) {
    if (i) return reg_gui->param1->value();
    return reg_gui->param0->value();
}


/* set limits on GUI sliders to match bounds[] 
   mode = 0 (coarse) = 1 (fine) */
void set_sliders( int mode ) {
    float bounds[6];
    float x,y,z,rx,ry,rz;
    get_pose_values(&x,&y,&z,&rx,&ry,&rz);
//	cout << "pose values x y z rx ry rz="<<x<<" "<<y<<" "<<z<<" "<<rx<<" "<<ry<<" "<<rz<<endl;
    get_bounds( bounds );
//	cout << "bounds"<< bounds[0]<<" "<<bounds[1]<<" "<<bounds[2]<<" "<<bounds[3]<<" "<<bounds[4]<<" "<<bounds[5]<<endl;
    if (mode) {
        float mult = 1.0f;
        float fext = mult * (bounds[1] - bounds[0]);
        reg_gui->slx->minimum( x - fext );
        reg_gui->slx->maximum( x + fext );
        fext = mult * (bounds[3] - bounds[2]);
        reg_gui->sly->minimum( y - fext );
        reg_gui->sly->maximum( y + fext );
        fext = mult * (bounds[5] - bounds[4]);
        reg_gui->slz->minimum( z - fext );
        reg_gui->slz->maximum( z + fext );
    }
    else {
        reg_gui->slx->minimum(bounds[0]);
        reg_gui->slx->maximum(bounds[1]);
        reg_gui->sly->minimum(bounds[2]);
        reg_gui->sly->maximum(bounds[3]);
        reg_gui->slz->minimum(bounds[4]);
        reg_gui->slz->maximum(bounds[5]);
    }

    reg_gui->slx->precision(2);
    reg_gui->sly->precision(2);
    reg_gui->slz->precision(2);
    reg_gui->slx->value( x );
    reg_gui->sly->value( y );
    reg_gui->slz->value( z );

    if (mode) {
        reg_gui->slrx->minimum(rx - 10.0);
        reg_gui->slrx->maximum(rx + 10.0);
        reg_gui->slry->minimum(ry - 10.0);
        reg_gui->slry->maximum(ry + 10.0);
        reg_gui->slrz->minimum(rz - 10.0);
        reg_gui->slrz->maximum(rz + 10.0);
    }
    else {
        reg_gui->slrx->minimum(-190.0);
        reg_gui->slrx->maximum(190.0);

        reg_gui->slry->minimum(-190.0);
        reg_gui->slry->maximum(190.0);

        reg_gui->slrz->minimum(-190.0);
        reg_gui->slrz->maximum(190.0);
    }
    reg_gui->slx->precision(2);
    reg_gui->sly->precision(2);
    reg_gui->slz->precision(2);
    reg_gui->slrx->value(rx);
    reg_gui->slry->value(ry);
    reg_gui->slrz->value(rz);
    reg_gui->trans->minimum(0);
    reg_gui->trans->maximum(255);
    reg_gui->trans->value(get_trans());

    //if range changes need to update slider position
    reg_gui->slx->redraw();
    reg_gui->sly->redraw();
    reg_gui->slz->redraw();
    reg_gui->slrx->redraw();
    reg_gui->slry->redraw();
    reg_gui->slrz->redraw();
}


void reinit_display() {
    if (!reg_gui) return;
    reg_gui->viewer->force_reinit();
}

void redraw_display(int i) {
    if (!reg_gui) return;
    if (i&1) reg_gui->viewer->redraw();
    if (i&2) reg_gui->plotter->redraw();
}

bool grab_model_image( int which ) {
    if (!reg_gui) return false;
    reg_gui->viewer->set_rez(which);
    reg_gui->viewer->force_reinit();
    reg_gui->viewer->grab_image();
    return true;
}

bool not_grab_model_image( int which ){
    if (!reg_gui) return false;
    reg_gui->viewer->set_rez(which);
    reg_gui->viewer->not_grab_image();
    return true;
}

// void rescale_plot( float x ) {
//   ostrstream cmd;
//   cmd << "rescale(" << exp(x) << ")" << ends;
//   PyRun_SimpleString(cmd.str());
//   reg_gui->plotter->redraw();
// }

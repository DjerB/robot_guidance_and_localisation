#ifndef _PROJECTION_
#define _PROJECTION_

extern void init_models( int );
//extern void load_centreline(char* fname);
extern void load_model( int, char* fname );
extern void delete_models();

extern void load_image( char* fname);
extern void loadDepthFile(char *fileName,bool Flip);
void save_pose(void);
//extern void generateProjections(float x,float y, float z, float rx, float ry, float rz);
extern void MinimisationPowell(void);
extern void generateProjectionsIntensity(void);
extern void generateProjectionsPQ(void);
extern void generateProjectionsPQDepth(void);
extern void generateProjectionsDepth(void);
extern void generateProjections(void);
extern int findFileLengthPPM(char* file_in_name);
extern int findFileLengthTXT(char* file_in_name);
extern void nextFname(char* file_in_name,int InputFileLength);
extern void next30Fname(char* file_in_name,int InputFileLength);
extern void readInRegistrationFileNames(void);
extern void displayRegistrationResults(void);
extern void zbuffer2TrueDepth(float zNear, float zFar, float *scaled_depth);
// extern void zbuffer2Depthmap(float *zbuf, vector<GLMpoint_d> *depthMap,int *mousecoord, 
// 			  float *info, float focal, int zbuf_width, float fnear, float ffar );
//extern void generateDepths(void);
//void save_pose(char* fname);
void load_pose(char* fname);
extern void delete_image();
extern void restore_image();

extern void set_nearfar_clip( float, float );
extern void set_near_clip( float );
extern void set_far_clip( float );
extern void set_focal_length( float );
extern void get_bounds( float[] );

extern void screen_reshape(int width, int height);

extern void screen_display(int, bool);


extern void set_sliders( class Registration2D3D*, int );

extern void screen_reshape(int width, int height);

extern void screen_display(int, bool);

extern void set_vx(float);
extern void set_vy(float);
extern void set_vz(float);
extern void set_rx(float);
extern void set_ry(float);
extern void set_rz(float);

extern void set_trans( unsigned char a);
extern unsigned char get_trans();
extern void set_focal_length( float );


extern void get_pose_values( float*, float*, float*, float*, float*, float* );
extern void set_pose_values( float, float, float, float, float, float );

extern double eval_similarity(int);

//control lighting conditions
extern void set_illumtype( int );
extern void get_illumtype( int* );
extern void set_redcomp( float );
extern void set_greencomp( float );
extern void set_bluecomp( float );
extern void set_alphacomp( float );
extern float* get_RGBAvalues( void );
extern void set_spotlight( float, float );
extern void get_spotlight( float*, float* );
extern void set_shininess( float );
extern void get_shininess( float* );
extern void set_attenuation(float, float,float);
extern void get_attenuation( float*, float*, float* );

//Python access to zbuffer and frame buffer
extern void get_view_image(int,int*,int*,char**,int*);
extern void get_zbuffer(int,int*,int*,char**,int*);

//transform mouse coordinates
extern void mousecoordtr( int *mousecoord );
extern void WinToGl( int *mousecoord, int *oglcoord );
extern void setmousecoord( int *mousecoord );
extern void getmousecoord( int *mousecoord );

//shape from shading
float ShapeFromShading( void );

//get-set input/output parameters for registering a series of video frames
class Reg_vid_fram_in_out{
public:
	int init_frame;
	int final_frame;
	int step;
	char *input_path;
	char *output_path;

	Reg_vid_fram_in_out( void ) { init_frame=0; final_frame=10; step=1; input_path=NULL; output_path=NULL; }
	~Reg_vid_fram_in_out(void ) { if(input_path) delete [] input_path; if(output_path) delete [] output_path;}
};
extern void get_info_start( int *init_frame, int *final_frame, int *step, char **input_path, char **output_pat );
extern void set_info_start( int init_frame, int final_frame, int step, const char *input_path, const char *output_path );

class StoreFrame{
public:
	int init_frame;
	int final_frame;
	int step;
	char *output_path;
	int imgwidth;
	int imgheight;

	StoreFrame( void ) { init_frame=0; final_frame=0; step=1; output_path=NULL; imgwidth=320; imgheight=240; }
	~StoreFrame(void ) { if(output_path) delete [] output_path;}
};
extern void set_store_info( int init_frame, int final_frame, int step, 
							const char *output_path, int imgwidth, int imgheight );
extern void get_store_info( int *init_frame, int *final_frame, int *step, 
						    char **output_path, int *imgwidth, int *imgheight );

extern void set_deform( bool );
extern void set_def_threshold( float );
extern void set_radius( float );
extern void set_centre( float, float, float );
extern void set_deform_amount( float );
extern void ApplyDeformation();
extern void DefineSphereCenter();
extern void RestoreOriginal();
extern void set_draw_sphere( bool );

extern void clear_shape_info( void );

extern bool getRGBimage( unsigned char *scaled_image, int img_width, int img_height );
extern void StoreFrameMain( char *output_path, int frame, int imgwidth, int imgheight );

#endif

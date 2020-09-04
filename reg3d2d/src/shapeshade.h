#ifndef SHAPE_SHADE
#define SHAPE_SHADE

typedef struct _GLMpoint_d {
	double x;
	double y;
	double z;
} GLMpoint_d;

typedef struct _GLMpoint2d_d {
	double x;
	double y;
} GLMpoint2d_d;


//shape from shading
bool pqmodel( float *zbuf, vector<GLMpoint_d> *pq_space, vector<long> *inf_vector, int *mousecoord, 
			  float *info, float focal, int zbuf_width, float fnear, float ffar, char* filename);
void pq_image( unsigned char *image, vector<GLMpoint_d> *pq_space, int *mousecoord,  
			   int width, int height, float *step, float focal_ratio );

void pq_scale( vector<GLMpoint_d> *pq_target, vector<GLMpoint_d> *pq_new, double *min, double *max );


float find_deform( vector<GLMpoint_d> *pq_space1, vector<GLMpoint_d> *pq_space2, vector<GLMpoint2d_d> *pq_deform);
double sim_rigid( vector<GLMpoint_d> *pq_obj, vector<GLMpoint_d> *pq_img, vector<long> *inf_vector );
float sim_deform( vector<GLMpoint> *pq_obj, vector<GLMpoint> *pq_img, 
				  vector<GLMpoint2d> *pq_deform, vector<long> *inf_vector );
float sim_deform_OF( vector<GLMpoint_d> *pq_obj, vector<GLMpoint_d> *pq_img, 
				  vector<float> *deform, vector<long> *inf_vector );
float sim_cross_corr( vector<GLMpoint> *pq_obj, vector<GLMpoint> *pq_img, vector<long> *inf_vector );
float cross_correlation( vector<float> *image_obj, vector<float> *image_img, vector<long> *inf_vector );

void stereograph( vector<GLMpoint> *pq_orig, vector<GLMpoint> **pq_stereograph );

void greyscaleimg( unsigned char *origimg, float *greyimg, int width, int height );

bool PQtoFile( char *fname, vector<GLMpoint> *pq_space, int *imgcoord );
bool PQfromFile( char *fname, vector<GLMpoint> *pq_space );
bool FloatfromFile( char *fname, vector<float> *vector_fl );
bool DataToFile( char *fname, vector<long> *data_vect, int *imgcoord );

//test_fun1: save to file the p-q components of the pq_space as ppm images 
void test_fun1( int *mousecoord, vector<GLMpoint2d> *pq_space, char *pathp, char *pathq, float scale );
void test_fun1( int *mousecoord, vector<GLMpoint> *pq_space, char *pathp, char *pathq, float scale );
//test_fun2: save to file the sqrt(p*p+q*q) of the pq_space as ppm image 
void test_fun2( int *mousecoord, vector<GLMpoint2d> *pq_space, char *pathp );
//test_fun3: save image as gray-scale ppm imagee
void test_fun3( float *image, char *path, int iwidth, int iheight, float scale );
void test_fun4( float *image, char *path, int iwidth, int iheight, float min, float max );
//dotprod_testfun2: save as ppm image the dotproduct of pq_obj and pq_img 
void dotprod_testfun2( vector<GLMpoint> *pq_obj, vector<GLMpoint> *pq_img, int *coord, char *path, float scale );

void VectorRestoration( vector<GLMpoint2d_d> **pq_orig, int *imgcoord, int n, float w, float l, float Dt );

void GaussSmoothImg( float *image, int iwidth, int iheight, float sigma);
void prepareSmooth( vector<GLMpoint2d_d> *pq_space, int *imgcoord, float sigma );
void prepareSmooth_mag( vector<GLMpoint2d_d> *pq_space, int *imgcoord, float sigma );

void store_float_image( vector<float> *image_float, int *imgcoord, char *path );
void store_float_image( vector<float> *image_float, int width, int height, char *path );
void clear_shape_info( void );

//store_2D_img: save to file as a 2D format-for matlab use....
bool store_2D_img( vector<GLMpoint_d> *pq_space, int *wid_heig, char *path1, char *path2 );
bool store_2D_img( vector<GLMpoint_d> *pq_space, int *wid_heig, char *path1, char *path2, char *path3 );
bool store_2D_img( vector<GLMpoint_d> *pq_space, int *wid_heig, int choose, char *path );


#endif

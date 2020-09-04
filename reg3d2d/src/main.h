#ifndef __MAIN_GUI__
#define __MAIN_GUI__

extern void set_abort_praxis( bool a );
extern void optimise();
extern void optimise_video();
extern int load_reg_result( const char *out_path );
extern void moveslider(int a);

extern void update_mouse( int* );

extern void update_display();
extern int param(int);

extern void set_sliders( int );
extern void reinit_display();
extern void redraw_display(int);
extern bool grab_model_image( int );
extern bool not_grab_model_image( int );

extern void rescale_plot( float x );
extern int get_method( void );

extern void storeFrame();
extern void StoreAllFrames();

extern void next_pose( int a );

extern char* choose_input_file();
extern bool load_track_data();
extern void optimise_video1DOF();
extern void optimise1DOF();
extern double test_function1DOF(double *x, int n);


#endif

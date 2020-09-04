#ifndef _REG_OGL_
#define _REG_OGL_

#include "scene_ogl.h"
#include "FL/Fl_Gl_Window.h"

class oglView : public Fl_Gl_Window {
  int rez;
  bool do_reinit;
  bool do_grab;
 public:
  oglView( int, int, int, int, char* name );

  void draw();
  void set_rez( int );
  void force_reinit() { do_reinit = true; }
  void grab_image();
  void not_grab_image();
  int handle( int );
};

class oglPlot : public Fl_Gl_Window {
  float angle;
  SG_camera* camera;
  SG_rotate* rot_y;
  SG_rotate* rot_x;
 public:
  oglPlot( int, int, int, int, char* );

  void draw();
  int handle(int);
};

#endif

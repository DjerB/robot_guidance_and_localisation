#include <iostream>
#include <FL/gl.h>

#include "ogl.h"
#include "main.h"
#include "scenegraph.h"
#include "projection.h"
#include "FL/Fl.h"
#include "FL/fl_draw.h"

using namespace std;

oglView::oglView( int x, int y, int w, int h, char* name ) :
Fl_Gl_Window(x,y,w,h,name), rez(0), do_reinit(false), do_grab(false) {
}

void oglView::draw() {
  if (!valid()) do_reinit = true;
  if (do_reinit) {
    screen_reshape( w(), h() );
    do_reinit = false;
  }
  screen_display(rez,do_grab);
  //printf("screen_display() called");
  do_grab = false;
  rez = 0;
}

void oglView::set_rez( int i ) {
  rez = i;
  redraw();
}

void oglView::grab_image() {
  make_current();
  do_grab = true;
  draw();
}

void oglView::not_grab_image() {
  make_current();
  do_grab = false;
  draw();
}

int oglView::handle( int event )
{
	int mousecoord[4] = {0,0,0,0};
	static int w = 0;
	static int h = 0;
	static int x0=0, y0=0, x1=0, y1=0;
	static bool ok = false;


	switch( event )
	{
	case FL_PUSH:
	{
		if( Fl::event_button() == 1 )	//left button
		{
			this->make_current();
			fl_overlay_clear();
			x0 = Fl::event_x();
			y0 = Fl::event_y();
			ok = true;
		}
		return 1;
	}
	case FL_DRAG:
	{
		if( ok )
		{
			x1 = Fl::event_x();
			y1 = Fl::event_y();
			mousecoord[0] = x0;	mousecoord[1] = y0; mousecoord[2] = x1; mousecoord[3] = y1; 
			mousecoordtr( mousecoord );

			update_mouse( mousecoord );			//update interface mouse coordinates
				
			w = mousecoord[3] - mousecoord[1];
			h = mousecoord[2] - mousecoord[0];
			make_current();
			fl_overlay_rect(mousecoord[0], mousecoord[1], h, w );
		}
		return 1;
	}
	case FL_RELEASE:
	{
		if( Fl::event_button() == 1 && ok )	//left button
		{
			x1 = Fl::event_x();
			y1 = Fl::event_y();
			mousecoord[0] = x0;	mousecoord[1] = y0; mousecoord[2] = x1; mousecoord[3] = y1; 
			mousecoordtr( mousecoord );
			setmousecoord( mousecoord );	//pass mouse coordinates to the application
			update_mouse( mousecoord );		//update interface mouse coordinates
			ok = false;
		}
		return 1;
	}
	default:
		return Fl_Widget::handle( event );
	}
	return 1;

}


oglPlot::oglPlot( int x, int y, int w, int h, char* name ) :
  Fl_Gl_Window(x,y,w,h,name) {
}

void oglPlot::draw() {
	printf("in oglPlot::draw()\n");
  // if (!valid()) {
  //   glMatrixMode(GL_PROJECTION);
  //   glLoadIdentity();
  //   glViewport(0,0,w(),h());
  //   glEnable(GL_DEPTH_TEST);

  // 	//set camera reference
  //   camera = (SG_camera*)get_noderef(0);
  //   if ( camera->node_type() != SG_camera::type_id() ) {
  //     cerr << "Not a camera" << endl;
  //     exit(-1);
  //   }
  //   SGogl_camera( camera );

  // 	//set y-axis rotation reference
  //   rot_y = (SG_rotate*)get_noderef(2);
  //   if ( rot_y->node_type() != SG_rotate::type_id() ) {
  //     cerr << "Not a rotation" << endl;
  //     exit(-1);
  //   }

  // 	//set x-axis rotation reference
  //   rot_x = (SG_rotate*)get_noderef(3);
  //   if ( rot_x->node_type() != SG_rotate::type_id() ) {
  //     cerr << "Not a rotation" << endl;
  //     exit(-1);
  //   }

  // }
  glClearColor(0.7f,1.0f,1.0f,1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // SG_mesh* mesh = (SG_mesh*)get_noderef(1);
  // if ( mesh->node_type() != SG_mesh::type_id() ) {
  //   cerr << "Not a mesh" << endl;
  //   exit(-1);
  // }

  // //assemble transform pipeline
  // SGogl_transforms trans;
  // trans.acc_up(camera);
  // trans.acc_down(mesh);
  // SGogl_mesh( mesh );

}

int oglPlot::handle( int ev ) {
	//cerr << "event " << ev << endl;
	int retval = 0;
	int midx, midy;
	switch (ev) {
	case FL_PUSH:
	case FL_RELEASE:
		retval = 1;
		break;
	case FL_DRAG:
		midx = w()/2;
		midy = h()/2;
		rot_y->set_angle((Fl::event_x() - midx)*90.0/midx);
		rot_x->set_angle((Fl::event_y() - midy)*90.0/midy);
		redraw();
		//cerr << Fl::event_x() << ' ' << midx << endl;
		retval = 1;
		break;
	default:
		retval = Fl_Gl_Window::handle(ev);
	}
	return retval;
}	


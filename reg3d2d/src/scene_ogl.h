#ifndef _SCENE_OGL_
#define _SCENE_OGL_

#include "scene_nodes.h"
#include <FL/gl.h>

class SGogl_transforms : public SG_acctrans {
 public:
  void translate( float x, float y, float z ) {
    glTranslatef(x,y,z);
  }
  void rotate( float x, float y, float z, float ang ) {
    glRotatef( ang, x, y, z );
  }
  void scale( float x, float y, float z ) { glScalef( x, y, z ); }

};


extern void SGogl_mesh( SG_mesh* );
extern void SGogl_camera( SG_camera* );

#endif

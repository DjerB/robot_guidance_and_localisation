#include <math.h>
#include <FL/glu.h>
#include "scene_ogl.h"


static void ogl_facet( SG_ifacet* facet, void* cdat ) {
  SG_mesh* mesh = (SG_mesh*) cdat;
  int n = facet->num_points();
  float *coords = new float [3*n];

  //load coordinates of vertices

  int i;
  for( i = 0; i<n; i++ ) {
    int j = facet->index(i,0);
    float *p = coords + 3*i;
    mesh->get_vertex(j, p, p+1, p+2);
  }

  //find normal
  float nx = 0.0, ny = 0.0, nz = 0.0;
  float *p1 = coords + 3*(n - 1);
  for( i = 0; i<n; i++ ) {
    float *p = coords + 3*i;
    nx = nx + p[1]*p1[2] - p[2]*p1[1];
    ny = ny + p[2]*p1[0] - p[0]*p1[2];
    nz = nz + p[0]*p1[1] - p[1]*p1[0];
    p1 = p;
  }

  float d = 1.0/sqrt(nx*nx+ny*ny+nz*nz);
  nx = d*nx;
  ny = d*ny;
  nz = d*nz;

  glBegin(GL_POLYGON);
  printf("ogl_facet:glBegin(GL_POLYGON) drawing vertices....\n");
  glColor3f( 0.5*(1+nx), 0.5*(1+ny),0.5*(1+nz) );
  for( i = 0; i<n; i++ ) {
    float *p = coords + 3*i;
    glVertex3f( p[0], p[1], p[2] );
  }
  glEnd();
  delete[] coords;
}

//render mesh node via opengl
void SGogl_mesh( SG_mesh* mesh ) {
  mesh->map_facets( ogl_facet, (void*)mesh );
}

//set up opengl viewing matrix
void SGogl_camera( SG_camera* camera ) {
  glMatrixMode(GL_PROJECTION);

  float u0, u1, v0, v1, nd, fd;
  camera->get_uv_bounds( &u0, &v0, &u1, &v1 );
  camera->get_nearfar( &nd, &fd );

  glFrustum( u0*nd, u1*nd, v0*nd, v1*nd, nd, fd );

  float cx, cy, cz, dx, dy, dz, upx, upy, upz;
  camera->get_viewpoint( &cx, &cy, &cz );
  camera->get_direction( &dx, &dy, &dz );
  camera->get_vup( &upx, &upy, &upz );
  gluLookAt( cx, cy, cz, cx+dx, cy+dy, cz+dz, upx, upy, upz );
}

#include "scene_nodes.h"


char* SG_node::type_id() { return "node";}
char* SG_group::type_id() { return "group";}

char* SG_transform::type_id() { return "transform";}
char* SG_translate::type_id() { return "translate";}
char* SG_rotate::type_id() { return "rotate";}
char* SG_scale::type_id() { return "scale";}

char* SG_camera::type_id() { return "camera";}
char* SG_primitive::type_id() { return "primitive";}
char* SG_light::type_id() { return "light";}
char* SG_material::type_id() { return "material";}
char* SG_mesh::type_id() { return "mesh";}

bool SG_group::add_child( SG_node* node ) {
  if (node->parent()) return false;
  members.push_back(node);
  node->parent(this);
  return true;
}

bool SG_group::orphan( SG_node* node ) {
  if (node->parent() != this) return false;
  for( memlist::iterator x = members.begin(); x!=members.end(); x++ ) {
    if (*x == node) {
      (*x)->parent(0);
      members.erase(x);
      break;
    }
  }
  return true;
}


SG_camera::SG_camera() : ortho(true), hither(0.01f), yon(1000.0f) {
  //centered at origin
  set_viewpoint(0,0,0);

  //aligned with +ve z-axis
  set_direction(0,0,1);

  set_lower_uv( -1.0, -1.0 );
  set_upper_uv( 1.0, 1.0 );

}

int SG_mesh::add_vertex( float x, float y, float z ) {
  int n = vtx.size();
  SG_v3 pt;
  pt.x = x; pt.y = y; pt.z = z;
  vtx.push_back(pt);
  return n;
}

bool SG_mesh::set_vertex( int ix, float x, float y, float z ) {
  if (ix<0 || ix>=vtx.size()) return false;
  SG_v3 pt;
  pt.x = x; pt.y = y; pt.z = z;
  vtx[ix] = pt;
  return true;
}

int SG_mesh::add_normal( float x, float y, float z ) {
  int n = normal.size();
  SG_v3 pt;
  pt.x = x; pt.y = y; pt.z = z;
  normal.push_back(pt);
  return n;
}

SG_ifacet* SG_mesh::new_facet(int numf) {
  int n = poly.size();
  poly.push_back( SG_ifacet(numf) );
  return &poly[n];
}

void SG_mesh::map_facets( void (*f)(SG_ifacet*, void*), void* cdat ) {
  for( polyarray::iterator ix = poly.begin(); ix!=poly.end(); ix++ ) {
    (*f)(&(*ix), cdat);
  }
}

// set index per field per point
bool SG_ifacet::set_index( int pi, int fld, int ix ) {
  // check that pi and fld are valid
  if (fld<0 || fld>=fld_n || pi<0 || pi*fld_n>=ndx.size()) return false;
  ndx[pi*fld_n + fld] = ix;
  return true;
}

int SG_ifacet::index( int pi, int fld ) {
  if (fld<0 || fld>=fld_n || pi<0 || pi*fld_n>=ndx.size()) return -1;
  return ndx[pi*fld_n + fld];
}

void SG_translate::acc_fwd_trans( SG_acctrans *act ) {
  act->translate( tx, ty, tz );
}

void SG_translate::acc_bck_trans( SG_acctrans *act ) {
  act->translate( -tx, -ty, -tz );
}


void SG_rotate::acc_fwd_trans( SG_acctrans *act ) {
  act->rotate( ax, ay, az, angle );
}

void SG_rotate::acc_bck_trans( SG_acctrans *act ) {
  act->rotate( ax, ay, az, -angle );
}

void SG_scale::acc_fwd_trans( SG_acctrans *act ) {
  act->scale( sx, sy, sz );
}

//WARNING: Scale nodes should not be degenerate
void SG_scale::acc_bck_trans( SG_acctrans *act ) {
  act->scale( 1.0/sx, 1.0/sy, 1.0/sz );
}

//Accummulate translations up or down hierarchy

//usually applied to camera nodes
void SG_acctrans::acc_up( SG_node *node ) {
  while (node) {
    node->acc_bck_trans(this);
    node = node->parent();
  }
}

//usually applied to primitive nodes
//need recursion here
void SG_acctrans::acc_down( SG_node *node ) {
  if (node) {
    acc_down( node->parent() );
    node->acc_fwd_trans(this);
  }
}


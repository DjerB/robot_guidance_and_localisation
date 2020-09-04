#ifndef _SCENE_NODES_
#define _SCENE_NODES_

#include <iostream>

#include <vector>
#include <list>

using namespace std;

struct SG_v3 { // 3d coords, normals and 3d textures
  float x, y, z;
};

struct SG_v2 { // for uv texture coords
  float u, v;
};

// base class of all nodes
class SG_node {
  SG_node* par_node;

  void parent( SG_node* p ) { par_node = p; }

public:
  static char* type_id();
  virtual char* node_type() { return type_id(); }

  SG_node() : par_node(0) {}

  SG_node* parent() { return par_node; }

  virtual void acc_fwd_trans( class SG_acctrans* ) {}
  virtual void acc_bck_trans( class SG_acctrans* ) {}

  friend class SG_group; //only SG_group methods should update parent pointer
};

class SG_group : public SG_node {
  typedef list<class SG_node *> memlist;
  memlist members;

public:
  static char* type_id();
  char* node_type() { return type_id(); }
  bool add_child( SG_node* );
  bool orphan( SG_node* );


};

//base class of transforms
class SG_transform : public SG_group {
  bool changed; // indicates that parameters have changed and any
    // implementation specific states (matrices, rendering pipelines,
    // etc.) need to be updated
public:
  static char* type_id();
  char* node_type() { return type_id(); }
  SG_transform() : changed(true) {}
};

/* Base class for composing transforms but folding up to/ down from
   the root from/to the given node */
class SG_acctrans {
public: 
  SG_acctrans() {}

  void acc_up( SG_node * );
  void acc_down( SG_node * );

  virtual void translate( float, float, float ) =0;
  virtual void rotate( float, float, float, float ) =0;
  virtual void scale( float, float, float ) =0;
};

// Specific transforms
class SG_translate : public SG_transform {
  float tx, ty, tz;
public:
  static char* type_id();
  char* node_type() { return type_id(); }

  SG_translate() : tx(0.0), ty(0.0), tz(0.0) {}
  SG_translate( float x, float y, float z ) : tx(x), ty(y), tz(z) {}

  void acc_fwd_trans( class SG_acctrans* );
  void acc_bck_trans( class SG_acctrans* );

  void set( float x, float y, float z ) {
	  tx = x; ty = y; tz = z;
  }
};

class SG_rotate : public SG_transform {
  float ax, ay, az, angle;
public:
  static char* type_id();
  char* node_type() { return type_id(); }

  SG_rotate() : ax(1.0), ay(0.0), az(0.0), angle(0.0) {}
  SG_rotate( float x, float y, float z, float a ) : ax(x), ay(y), az(z),
						    angle(a) {}

  void acc_fwd_trans( class SG_acctrans* );
  void acc_bck_trans( class SG_acctrans* );

  void set_axis( float x, float y, float z ) {
	ax = x; ay = y; az = z;
  }

  void set_angle( float a ) {
	angle = a;
  }
};

class SG_scale : public SG_transform {
  float sx, sy, sz;
public:
  static char* type_id();
  char* node_type() { return type_id(); }

  SG_scale() : sx(1.0), sy(1.0), sz(1.0) {}
  SG_scale( float x, float y, float z ) : sx(x), sy(y), sz(z) {}

  void acc_fwd_trans( class SG_acctrans* );
  void acc_bck_trans( class SG_acctrans* );

  void set( float x, float y, float z ) {
	  sx = x; sy = y; sz = z;
  }
};


class SG_camera : public SG_node {
  SG_v3 vcen; //center of projection
  SG_v3 vdir; //viewing direction; length is focal length
  SG_v3 vup;  //+ve vertical wrt eye coord system
  bool ortho;
  SG_v2 uvmin, uvmax; //viewing window
  float hither, yon;
public:
  static char* type_id();
  char* node_type() { return type_id(); }

  SG_camera();

  //access methods

  void set_viewpoint( float x, float y, float z ) {
    vcen.x = x;
    vcen.y = y;
    vcen.z = z;
  }

  void get_viewpoint( float *x, float *y, float *z ) {
    *x = vcen.x;
    *y = vcen.y;
    *z = vcen.z;
  }

  void set_direction( float x, float y, float z ) {
    vdir.x = x;
    vdir.y = y;
    vdir.z = z;
  }

  void get_direction( float *x, float *y, float *z ) {
    *x = vdir.x;
    *y = vdir.y;
    *z = vdir.z;
  }

  void set_vup( float x, float y, float z ) {
    vup.x = x;
    vup.y = y;
    vup.z = z;
  }

  void get_vup( float *x, float *y, float *z ) {
    *x = vup.x;
    *y = vup.y;
    *z = vup.z;
  }

  bool is_orthographic() { return ortho; }

  void set_lower_uv( float u, float v ) {
    uvmin.u = u;
    uvmin.v = v;
  }

  void set_upper_uv( float u, float v ) {
    uvmax.u = u;
    uvmax.v = v;
  }

  void get_uv_bounds( float *u0, float *v0, float *u1, float *v1 ) {
    *u0 = uvmin.u;
    *v0 = uvmin.v;
    *u1 = uvmax.u;
    *v1 = uvmax.v;
  }

  void set_nearfar( float n, float f) {
    hither = n;
    yon = f;
  }

  void get_nearfar( float *np, float *fp) {
    *np = hither;
    *fp = yon;
  }

};

class SG_primitive : public SG_node {

public:
  static char* type_id();
  char* node_type() { return type_id(); }

};

// base class for lights
class SG_light : public SG_node {

public:
  static char* type_id();
  char* node_type() { return type_id(); }
};

// base class for material attributes
class SG_material : public SG_group {

public:
  static char* type_id();
  char* node_type() { return type_id(); }

};



// indexed facet - facet whose vertices are indices
class SG_ifacet {
  int fld_n; // number of fields; usually up to 1,2, or 3 but could be more
  // 3d coord ; 3d normal; texture coords
  vector<int> ndx;

public:
  SG_ifacet(int nf) : fld_n(nf) {}
  void add_points(int n =1) {
    for (int i = 0; i<fld_n*n; i++)
      ndx.push_back(-1);
  }

  bool set_index( int, int, int );
  int index( int, int );
  int num_flds() { return fld_n; }
  int num_points() { return ndx.size() / fld_n; }
};

class SG_mesh : public SG_primitive {

  vector<SG_v3> vtx;
  vector<SG_v3> normal;
  vector<SG_v2> texture;
  vector<SG_v3> solidtex; //solid texture coords
  typedef vector<SG_ifacet> polyarray;
  polyarray poly;

public:
  static char* type_id();
  char* node_type() { return type_id(); }

  int add_vertex( float, float, float );
  bool set_vertex( int, float, float, float );

  int add_normal( float, float, float );

  void get_vertex( int i, float *xp, float *yp, float *zp ) {
    SG_v3 p = vtx[i];
    *xp = p.x; *yp = p.y; *zp = p.z;
  }

  void get_normal( int i, float *xp, float *yp, float *zp ) {
    SG_v3 p = normal[i];
    *xp = p.x; *yp = p.y; *zp = p.z;
  }

  SG_ifacet* new_facet(int);
  SG_ifacet* facet(int n) { return &poly[n]; }
  int num_facets() { return poly.size(); }

  // map a function over all facets
  void map_facets( void(*f)(SG_ifacet*, void*), void* );
};


#endif

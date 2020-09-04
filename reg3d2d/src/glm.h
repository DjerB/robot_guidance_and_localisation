/*    
      glm.h
      Nate Robins, 1997, 2000
      nate@pobox.com, http://www.pobox.com/~nate
 
      Wavefront OBJ model file format reader/writer/manipulator.

      Includes routines for generating smooth normals with
      preservation of edges, welding redundant vertices & texture
      coordinate generation (spheremap and planar projections) + more.

 */
#ifndef GLM_H
#define GLM_H

#include "GLUT/glut.h"

#ifndef M_PI
#define M_PI 3.14159265f
#endif

#define GLM_NONE     (0)            /* render with only vertices */
#define GLM_FLAT     (1 << 0)       /* render with facet normals */
#define GLM_SMOOTH   (1 << 1)       /* render with vertex normals */
#define GLM_TEXTURE  (1 << 2)       /* render with texture coords */
#define GLM_COLOR    (1 << 3)       /* render with colors */
#define GLM_MATERIAL (1 << 4)       /* render with materials */
#define GLM_STRIP    (1 << 5)		/* render using strips*/


/* GLMmaterial: Structure that defines a material in a model. 
 */
typedef struct _GLMmaterial
{
  char* name;                   /* name of material */
  GLfloat diffuse[4];           /* diffuse component */
  GLfloat ambient[4];           /* ambient component */
  GLfloat specular[4];          /* specular component */
  GLfloat emmissive[4];         /* emmissive component */
  GLfloat shininess;            /* specular exponent */
} GLMmaterial;

/* GLMtriangle: Structure that defines a triangle in a model.
 */
typedef struct _GLMtriangle {
  GLuint vindices[3];           /* array of triangle vertex indices */
  GLuint nindices[3];           /* array of triangle normal indices */
  GLuint tindices[3];           /* array of triangle texcoord indices*/
  GLuint findex;                /* index of triangle facet normal */
} GLMtriangle;

/* GLMgroup: Structure that defines a strip in a model.
 */
typedef struct _GLMstrip{
	GLuint numvertices;
	GLuint *vindices;
} GLMstrip;

/* GLMgroup: Structure that defines a group in a model.
 */
typedef struct _GLMgroup {
  char*             name;           /* name of this group */
  GLuint            numtriangles;   /* number of triangles in this group */
  GLuint*           triangles;      /* array of triangle indices */
  GLuint            material;       /* index to material for group */
  GLuint			numstrips;		/* number of strips in this group */
  GLuint*			strips;			/* array of triangle indices */
  struct _GLMgroup* next;           /* pointer to next group in model */
} GLMgroup;


/* GLMmodel: Structure that defines a model.
 */
typedef struct _GLMmodel {
  char*    pathname;            /* path to this model */
  char*    mtllibname;          /* name of the material library */

  GLuint   numvertices;         /* number of vertices in model */
  GLfloat* vertices;            /* array of vertices  */

  GLuint   numnormals;          /* number of normals in model */
  GLfloat* normals;             /* array of normals */

  GLuint   numtexcoords;        /* number of texcoords in model */
  GLfloat* texcoords;           /* array of texture coordinates */

  GLuint   numfacetnorms;       /* number of facetnorms in model */
  GLfloat* facetnorms;          /* array of facetnorms */

  GLuint       numtriangles;    /* number of triangles in model */
  GLMtriangle* triangles;       /* array of triangles */

  GLuint       nummaterials;    /* number of materials in model */
  GLMmaterial* materials;       /* array of materials */

  GLuint       numgroups;       /* number of groups in model */
  GLMgroup*    groups;          /* linked list of groups */

  GLuint	   numstrips;		/* number of strips in model */
  GLMstrip*	   strips;			/* array of strips */

  GLfloat position[3];          /* position of the model */

} GLMmodel;


/* GLMpoints: Structure that defines a vertex.
 */
typedef struct _GLMpoint {
	float x;
	float y;
	float z;
} GLMpoint;

typedef struct _GLMpoint2d {
	float x;
	float y;
} GLMpoint2d;


#define T(x) (model->triangles[(x)])

/* _GLMnode: general purpose node */
typedef struct _GLMnode {
    GLuint         index;
    GLboolean      averaged;
    struct _GLMnode* next;
} GLMnode;

/* glmBound: find bounding extents of model
 *
 * model - properly initialized GLMmodel structure 
 */
void
glmBound(GLMmodel* model, GLfloat[6] );

/* glmDimensions: Calculates the dimensions (width, height, depth) of
 * a model.
 *
 * model      - initialized GLMmodel structure
 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
 */
GLvoid
glmDimensions(GLMmodel* model, GLfloat* dimensions);

/* glmScale: Scales a model by a given amount.
 * 
 * model - properly initialized GLMmodel structure
 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
 */
GLvoid
glmScale(GLMmodel* model, GLfloat scale);

/* glmDelete: Deletes a GLMmodel structure.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmDelete(GLMmodel* model);

/* glmCopyModel: Copies a GLMmodel structure into an other
 *
 * origmodel - initialized GLMmodel structure
 * model - targer GLMmodel structure
 */
GLvoid
glmCopyModel( GLMmodel *origmodel, GLMmodel **target_model );


/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
GLMmodel* 
glmReadOBJ(char* filename);


/* glmReadPPM: read a PPM raw (type P6) file.  The PPM file has a header
 * that should look something like:
 *
 *    P6
 *    # comment
 *    width height max_value
 *    rgbrgbrgb...
 *
 * where "P6" is the magic cookie which identifies the file type and
 * should be the only characters on the first line followed by a
 * carriage return.  Any line starting with a # mark will be treated
 * as a comment and discarded.   After the magic cookie, three integer
 * values are expected: width, height of the image and the maximum
 * value for a pixel (max_value must be < 256 for PPM raw files).  The
 * data section consists of width*height rgb triplets (one byte each)
 * in binary format (i.e., such as that written with fwrite() or
 * equivalent).
 *
 * The rgb data is returned as an array of unsigned chars (packed
 * rgb).  The malloc()'d memory should be free()'d by the caller.  If
 * an error occurs, an error message is sent to stderr and NULL is
 * returned.
 *
 * filename   - name of the .ppm file.
 * width      - will contain the width of the image on return.
 * height     - will contain the height of the image on return.
 *
 */
 
 bool glmWriteNOFF(GLMmodel* model, char *path);
GLubyte* 
glmReadPPM(char* filename, int* width, int* height);

/* glmWritePPM: write an image in a PPM raw (type P6) file.  The PPM file has a header
 * that should look something like:
 *
 *    P6
 *    # comment
 *    width height max_value
 *    rgbrgbrgb...
 *
 * Read glmReadPPM for more information about the PPM format
 *
 * path		  - path of the .ppm file
 * width      - will contain the width of the image on writing.
 * height     - will contain the height of the image on writing.
 * image      - contains the image data in RGBA format (the A component is ignored)
 */
bool glmWritePPM( char *path, unsigned char *image, int width, int height);


/* glmCopyPPM: copy an rgba image from imgorig to imgcpy
 * 
 * Read glmReadPPM for more information about the PPM format
 *
 * iwidth      - will contain the width of both images
 * iheight     - will contain the height of both images
 */
void glmCopyPPM( unsigned char *imgorig, unsigned char *imgcpy, int iwidth, int iheight );

/* glmDraw: Renders the model to the current OpenGL context using the
 * mode specified.
 *
 * model - initialized GLMmodel structure
 * mode  - a bitwise OR of values describing what is to be rendered.
 *             GLM_NONE     -  render with only vertices
 *             GLM_FLAT     -  render with facet normals
 *             GLM_SMOOTH   -  render with vertex normals
 *             GLM_TEXTURE  -  render with texture coords
 *             GLM_COLOR    -  render with colors (color material)
 *             GLM_MATERIAL -  render with materials
 *             GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *             GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
 
 GLvoid
glmDraw(GLMmodel* model, GLuint mode);

GLvoid
glmDraw(GLMmodel* model, GLuint mode, GLfloat* centreline);

/* glmList: Generates and returns a display list for the model using
 * the mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE    -  render with only vertices
 *            GLM_FLAT    -  render with facet normals
 *            GLM_SMOOTH  -  render with vertex normals
 *            GLM_TEXTURE -  render with texture coords
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLuint
glmList(GLMmodel* model, GLuint mode, GLfloat* centreline);


/* glmFacetNormals: Generates facet normals for a model (by taking the
 * cross product of the two vectors derived from the sides of each
 * triangle).  Assumes a counter-clockwise winding.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmFacetNormals(GLMmodel* model);

/* glmVertexNormals: Generates smooth vertex normals for a model.
 * First builds a list of all the triangles each vertex is in.  Then
 * loops through each vertex in the the list averaging all the facet
 * normals of the triangles each vertex is in.  Finally, sets the
 * normal index in the triangle for the vertex to the generated smooth
 * normal.  If the dot product of a facet normal and the facet normal
 * associated with the first triangle in the list of triangles the
 * current vertex is in is greater than the cosine of the angle
 * parameter to the function, that facet normal is not added into the
 * average normal calculation and the corresponding vertex is given
 * the facet normal.  This tends to preserve hard edges.  The angle to
 * use depends on the model, but 90 degrees is usually a good start.
 *
 * model - initialized GLMmodel structure
 * angle - maximum angle (in degrees) to smooth across
 */
GLvoid
glmVertexNormals(GLMmodel* model, GLfloat angle);


/* glmCross: compute the cross product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 * n - array of 3 GLfloats (GLfloat n[3]) to return the cross product in
 */
static GLvoid
glmCross(GLfloat* u, GLfloat* v, GLfloat* n);

/* glmNormalize: normalize a vector
 *
 * v - array of 3 GLfloats (GLfloat v[3]) to be normalized
 */
static GLvoid
glmNormalize(GLfloat* v);

/* glmDot: compute the dot product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 */
static GLfloat
glmDot(GLfloat* u, GLfloat* v);


#endif

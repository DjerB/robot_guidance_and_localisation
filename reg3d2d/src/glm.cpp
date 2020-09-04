/*    
      glm.c
      Nate Robins, 1997, 2000
      nate@pobox.com, http://www.pobox.com/~nate
 
      Wavefront OBJ model file format reader/writer/manipulator.

      Includes routines for generating smooth normals with
      preservation of edges, welding redundant vertices & texture
      coordinate generation (spheremap and planar projections) + more.
  
*/


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "glm.h"
#include "global.h"
#include "tdgeometry.h"


/* glmFindGroup: Find a group in the model */
GLMgroup*
glmFindGroup(GLMmodel* model, char* name)
{
    GLMgroup* group;
    
    assert(model);
    
    group = model->groups;
    while(group) {
        if (!strcmp(name, group->name))
            break;
        group = group->next;
    }
    
    return group;
}

/* glmAddGroup: Add a group to the model */
GLMgroup*
glmAddGroup(GLMmodel* model, char* name)
{
    GLMgroup* group;
    
    group = glmFindGroup(model, name);
    if (!group) {
        group = (GLMgroup*)malloc(sizeof(GLMgroup));
        group->name = strdup(name);
        group->material = 0;
        group->numtriangles = 0;
        group->triangles = NULL;
		group->numstrips = 0;
		group->strips = NULL;
        group->next = model->groups;
        model->groups = group;
        model->numgroups++;
    }
    
    return group;
}

/* glmFindGroup: Find a material in the model */
GLuint
glmFindMaterial(GLMmodel* model, char* name)
{
    GLuint i;
    
    /* XXX doing a linear search on a string key'd list is pretty lame,
    but it works and is fast enough for now. */
    for (i = 0; i < model->nummaterials; i++) {
        if (!strcmp(model->materials[i].name, name))
            goto found;
    }
    
    /* didn't find the name, so print a warning and return the default
    material (0). */
    printf("glmFindMaterial():  can't find material \"%s\".\n", name);
    i = 0;
    
found:
    return i;
}


/* glmDirName: return the directory given a path
 *
 * path - filesystem path
 *
 * NOTE: the return value should be free'd.
 */
static char*
glmDirName(char* path)
{
    char* dir;
    char* s;
    
    dir = strdup(path);
    
    s = strrchr(dir, '/');
    if (s)
        s[1] = '\0';
    else
        dir[0] = '\0';
    
    return dir;
}


/* glmUnitize: find bounding extents
 *
 * model - properly initialized GLMmodel structure 
 */
void
glmBound(GLMmodel* model, GLfloat bounds[6])
{
    GLuint  i;
    GLfloat maxx, minx, maxy, miny, maxz, minz;
    
    assert(model);
    assert(model->vertices);
    
    /* get the max/mins */
    maxx = minx = model->vertices[ 0];
    maxy = miny = model->vertices[ 1];
    maxz = minz = model->vertices[ 2];
    for (i = 0; i < model->numvertices; i++) {
        if (maxx < model->vertices[3 * i + 0])
            maxx = model->vertices[3 * i + 0];
        if (minx > model->vertices[3 * i + 0])
            minx = model->vertices[3 * i + 0];
        
        if (maxy < model->vertices[3 * i + 1])
            maxy = model->vertices[3 * i + 1];
        if (miny > model->vertices[3 * i + 1])
            miny = model->vertices[3 * i + 1];
        
        if (maxz < model->vertices[3 * i + 2])
            maxz = model->vertices[3 * i + 2];
        if (minz > model->vertices[3 * i + 2])
            minz = model->vertices[3 * i + 2];
    }
    
    GLfloat extra = (maxz-minz)*0.4;
    bounds[0] = minx;
    bounds[1] = maxx;
    bounds[2] = miny;
    bounds[3] = maxy;
    bounds[4] = minz-extra;
    bounds[5] = maxz+extra;
}

/* glmDimensions: Calculates the dimensions (width, height, depth) of
 * a model.
 *
 * model   - initialized GLMmodel structure
 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
 */
GLvoid
glmDimensions(GLMmodel* model, GLfloat* dimensions)
{
    GLuint i;
    GLfloat maxx, minx, maxy, miny, maxz, minz;
    
    assert(model);
    assert(model->vertices);
    assert(dimensions);
    
    /* get the max/mins */
    maxx = minx = model->vertices[3 + 0];
    maxy = miny = model->vertices[3 + 1];
    maxz = minz = model->vertices[3 + 2];
    for (i = 1; i <= model->numvertices; i++) {
        if (maxx < model->vertices[3 * i + 0])
            maxx = model->vertices[3 * i + 0];
        if (minx > model->vertices[3 * i + 0])
            minx = model->vertices[3 * i + 0];
        
        if (maxy < model->vertices[3 * i + 1])
            maxy = model->vertices[3 * i + 1];
        if (miny > model->vertices[3 * i + 1])
            miny = model->vertices[3 * i + 1];
        
        if (maxz < model->vertices[3 * i + 2])
            maxz = model->vertices[3 * i + 2];
        if (minz > model->vertices[3 * i + 2])
            minz = model->vertices[3 * i + 2];
    }
    
    /* calculate model width, height, and depth */
    dimensions[0] = Myabs(maxx) + Myabs(minx);
    dimensions[1] = Myabs(maxy) + Myabs(miny);
    dimensions[2] = Myabs(maxz) + Myabs(minz);
}

/* glmScale: Scales a model by a given amount.
 * 
 * model - properly initialized GLMmodel structure
 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
 */
GLvoid
glmScale(GLMmodel* model, GLfloat scale)
{
    GLuint i;
    
    for (i = 1; i <= model->numvertices; i++) {
        model->vertices[3 * i + 0] *= scale;
        model->vertices[3 * i + 1] *= scale;
        model->vertices[3 * i + 2] *= scale;
    }
}

/* glmDelete: Deletes a GLMmodel structure.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmDelete(GLMmodel* model)
{
    GLMgroup* group;
    GLuint i;
    
    assert(model);
    
    if (model->pathname)     free(model->pathname);
    if (model->mtllibname) free(model->mtllibname);
    if (model->vertices)     free(model->vertices);
    if (model->normals)  free(model->normals);
    if (model->texcoords)  free(model->texcoords);
    if (model->facetnorms) free(model->facetnorms);
    if (model->triangles)  free(model->triangles);
    if (model->materials) {
        for (i = 0; i < model->nummaterials; i++)
            free(model->materials[i].name);
    }
    free(model->materials);
    while(model->groups) {
        group = model->groups;
        model->groups = model->groups->next;
        free(group->name);
        free(group->triangles);
        free(group);
    }
    
    free(model);
}

/* glmCopyModel: Copies a GLMmodel structure into an other
 *
 * origmodel - initialized GLMmodel structure
 * model - targer GLMmodel structure
 */
GLvoid
glmCopyModel( GLMmodel *origmodel, GLMmodel **target_model )
{
	int length;
	GLuint i, j;

	assert( origmodel );

	*target_model = new GLMmodel;
	GLMmodel *model = *target_model;

	//copy: path to this model
	if( origmodel->pathname )	
	{
		length = strlen( origmodel->pathname );
		model->pathname = new char[length+1];
		strcpy( model->pathname, origmodel->pathname );
	}
	else
		model->pathname = NULL;

	//copy: name of the material library
	if( origmodel->mtllibname )	
	{
		length = strlen( origmodel->mtllibname );
		model->mtllibname = new char[length+1];
		strcpy( model->mtllibname, origmodel->mtllibname );
	}
	else
		model->mtllibname = NULL;

	//copy: number of vertices in model
	//copy: array of vertices
	if( origmodel->numvertices )
	{
		assert( origmodel->vertices );
		model->numvertices = origmodel->numvertices;	
		model->vertices = new GLfloat[3*(origmodel->numvertices+1)];	
		for( i=0; i<3*(origmodel->numvertices+1); i++ )
			model->vertices[i] = origmodel->vertices[i];
	}
	else
	{
		model->numvertices = 0;
		model->vertices = NULL;
	}

	//copy: number of normals in model
	//copy: array of normals 
	if( origmodel->numnormals )
	{
		assert( origmodel->normals );
		model->numnormals = origmodel->numnormals;	
		model->normals = new GLfloat[3*(origmodel->numnormals+1)];	
		for( i=0; i<3*(origmodel->numnormals+1); i++ )
			model->normals[i] = origmodel->normals[i];
	}
	else
	{
		model->numnormals = 0;
		model->normals = NULL;
	}
	
	//copy: number of texcoords in model
	//copy: array of texture coordinates 
	if( origmodel->numtexcoords )
	{
		assert( origmodel->texcoords );
		model->numtexcoords = origmodel->numtexcoords;	
		model->texcoords = new GLfloat[2*(origmodel->numtexcoords+1)];	
		for( i=0; i<2*(origmodel->numtexcoords+1); i++ )
			model->texcoords[i] = origmodel->texcoords[i];
	}
	else
	{
		model->numtexcoords = 0;
		model->texcoords = NULL;
	}

	//copy: number of facetnorms in model
	//copy: array of facetnorms 
	if( origmodel->numfacetnorms )
	{
		assert( origmodel->facetnorms );
		model->numfacetnorms = origmodel->numfacetnorms;	
		model->facetnorms = new GLfloat[3*(origmodel->numfacetnorms+1)];	
		for( i=0; i<3*(origmodel->numfacetnorms+1); i++ )
			model->facetnorms[i] = origmodel->facetnorms[i];
	}
	else
	{
		model->numfacetnorms = 0;
		model->facetnorms = NULL;
	}

	//copy: number of triangles in model
	//copy: array of triangles
	if( origmodel->numtriangles )
	{
		assert( origmodel->triangles );

		model->numtriangles = origmodel->numtriangles;
		model->triangles = new GLMtriangle[origmodel->numtriangles];
		for( i=0; i<origmodel->numtriangles; i++ )
		{
			model->triangles[i].vindices[0] = origmodel->triangles[i].vindices[0];
			model->triangles[i].vindices[1] = origmodel->triangles[i].vindices[1];
			model->triangles[i].vindices[2] = origmodel->triangles[i].vindices[2];
			model->triangles[i].nindices[0] = origmodel->triangles[i].nindices[0];
			model->triangles[i].nindices[1] = origmodel->triangles[i].nindices[1];
			model->triangles[i].nindices[2] = origmodel->triangles[i].nindices[2];
			model->triangles[i].tindices[0] = origmodel->triangles[i].tindices[0];
			model->triangles[i].tindices[1] = origmodel->triangles[i].tindices[1];
			model->triangles[i].tindices[2] = origmodel->triangles[i].tindices[2];
			model->triangles[i].findex = origmodel->triangles[i].findex;
		}
	}
	else
	{
		model->numtriangles = 0;
		model->triangles = NULL;
	}

	//copy: number of materials in model
	//copy: array of materials
	if( origmodel->nummaterials )
	{
		assert( origmodel->materials );

		model->nummaterials = origmodel->nummaterials;
		model->materials = new GLMmaterial[origmodel->nummaterials];
		for( i=0; i<origmodel->nummaterials; i++ )
		{
			if( origmodel->materials[i].name )
			{
				length = strlen( origmodel->materials[i].name );
				model->materials[i].name = new char[length+1];
				strcpy( model->materials[i].name, origmodel->materials[i].name );
			}
			else
				model->materials[i].name = NULL;

			model->materials[i].diffuse[0] = origmodel->materials[i].diffuse[0];
			model->materials[i].diffuse[1] = origmodel->materials[i].diffuse[1];
			model->materials[i].diffuse[2] = origmodel->materials[i].diffuse[2];
			model->materials[i].diffuse[3] = origmodel->materials[i].diffuse[3];
			model->materials[i].ambient[0] = origmodel->materials[i].ambient[0];
			model->materials[i].ambient[1] = origmodel->materials[i].ambient[1];
			model->materials[i].ambient[2] = origmodel->materials[i].ambient[2];
			model->materials[i].ambient[3] = origmodel->materials[i].ambient[3];
			model->materials[i].specular[0] = origmodel->materials[i].specular[0];
			model->materials[i].specular[1] = origmodel->materials[i].specular[1];
			model->materials[i].specular[2] = origmodel->materials[i].specular[2];
			model->materials[i].specular[3] = origmodel->materials[i].specular[3];
			model->materials[i].emmissive[0] = origmodel->materials[i].emmissive[0];
			model->materials[i].emmissive[1] = origmodel->materials[i].emmissive[1];
			model->materials[i].emmissive[2] = origmodel->materials[i].emmissive[2];
			model->materials[i].emmissive[3] = origmodel->materials[i].emmissive[3];
			model->materials[i].shininess = origmodel->materials[i].shininess;
		}
	}
	else
	{
		model->nummaterials = 0;
		model->materials = NULL;
	}

	//copy: number of groups in model
	//copy: linked list of groups 
	if( origmodel->numgroups )
	{
		assert( origmodel->groups );

		model->numgroups = origmodel->numgroups;
		model->groups = new GLMgroup[origmodel->numgroups];
		for( i=0; i<origmodel->numgroups; i++ )
		{
			if( origmodel->groups[i].name )
			{
				length = strlen( origmodel->groups[i].name );
				model->groups[i].name = new char[length+1];
				strcpy( model->groups[i].name, origmodel->groups[i].name );
			}
			else
				model->groups[i].name = NULL;

			model->groups[i].material = origmodel->groups[i].material;
			model->groups[i].numtriangles = origmodel->groups[i].numtriangles;
			model->groups[i].next = origmodel->groups[i].next;
			model->groups[i].triangles = new GLuint[origmodel->groups[i].numtriangles];
			for( j=0; j<origmodel->groups[i].numtriangles; j++ )
				model->groups[i].triangles[j] = origmodel->groups[i].triangles[j];
		}
	}
	else
	{
		model->numgroups = 0;
		model->groups = NULL;
	}

	//copy: position of the model
	model->position[0] = origmodel->position[0];
	model->position[1] = origmodel->position[1];
	model->position[2] = origmodel->position[2];

}

/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
GLMmodel* 
glmReadOBJ(char* filename)
{
    GLMmodel* model;

	TDGeometry geometry;
    if (!geometry.TDGReadNOFF(filename) && !geometry.TDGReadOFF(filename)) exit(-1);

    /* allocate a new model */
    model = (GLMmodel*)malloc(sizeof(GLMmodel));
    model->pathname    = strdup(filename);
    model->mtllibname    = NULL;
    model->numvertices   = 0;
    model->vertices    = NULL;
    model->numnormals    = 0;
    model->normals     = NULL;
    model->numtexcoords  = 0;
    model->texcoords       = NULL;
    model->numfacetnorms = 0;
    model->facetnorms    = NULL;
    model->numtriangles  = 0;
    model->triangles       = NULL;
    model->nummaterials  = 0;
    model->materials       = NULL;
    model->numgroups       = 0;
    model->groups		   = NULL;
	model->numstrips	   = 0;
	model->strips		   = NULL;		
    model->position[0]   = 0.0;
    model->position[1]   = 0.0;
    model->position[2]   = 0.0;
    

    GLMgroup* group;            /* current group */
    group = glmAddGroup(model, "default");

	group->numtriangles = geometry.TDGGetNumPoly();
	group->numstrips = geometry.TDGGetNumStrip();

    /* allocate memory for the triangles in each group */
    group = model->groups;
    group->triangles = (GLuint*)malloc(sizeof(GLuint) * group->numtriangles);
	group->strips = (GLuint*)malloc(sizeof(GLuint) * group->numstrips );
 
	model->numvertices = geometry.TDGGetNumVert();
	model->numnormals = geometry.TDGGetNumVert();
	model->numtriangles = geometry.TDGGetNumPoly();
	model->numstrips	= geometry.TDGGetNumStrip();
	model->nummaterials = 1;	//default

    /* allocate memory */
    model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
        3 * (model->numvertices + 1));
    model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
        model->numtriangles);
    if (model->numnormals) {
        model->normals = (GLfloat*)malloc(sizeof(GLfloat) *
            3 * (model->numnormals + 1));
    }
    if (model->numtexcoords) {
        model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
            2 * (model->numtexcoords + 1));
    }

   if (model->nummaterials) {
        model->materials = (GLMmaterial*)malloc(sizeof(GLMmaterial) * model->nummaterials );
    }

   if (model->numstrips ) {
	   model->strips = (GLMstrip*)malloc(sizeof(GLMstrip) * model->numstrips);
   }
    
	//copy data    
	unsigned int i;
	for( i=0; i < model->numvertices; i++ )
	{
		stvertex vertex = geometry.TDGGetVertex( i );
		model->vertices[i*3] = vertex.p[0];
		model->vertices[i*3+1] = vertex.p[1];
		model->vertices[i*3+2] = vertex.p[2];
		model->normals[i*3] = vertex.n[0];
		model->normals[i*3+1] = vertex.n[1];
		model->normals[i*3+2] = vertex.n[2];
		//printf("vertices=%fvertex=%f",model->vertices[i*3],vertex.p[0]);
	}

	for( i=0; i < model->numtriangles; i++)
	{
		stfacetl triangle1 = geometry.TDGGetIndex( i );
		model->triangles[i].vindices[0] = triangle1.findex[0];
		model->triangles[i].vindices[1] = triangle1.findex[1];
		model->triangles[i].vindices[2] = triangle1.findex[2];
		model->triangles[i].nindices[0] = triangle1.findex[0];
		model->triangles[i].nindices[1] = triangle1.findex[1];
		model->triangles[i].nindices[2] = triangle1.findex[2];
	    group->triangles[i] = i;
	}

	for( i=0; i < model->numstrips; i++)
	{
		ststrip strip = geometry.TDGGetStrip( i );
		model->strips[i].numvertices = strip.length;
		model->strips[i].vindices = new GLuint[strip.length];
		for( int j=0; j<strip.length; j++ )
			model->strips[i].vindices[j] = strip.list[j];
		group->strips[i] = i;
	}


	//default values
	for( i=0; i < model->nummaterials; i++)
	{
		model->materials[i].diffuse[0] = 0.43; 
		model->materials[i].diffuse[1] = 0.47;
		model->materials[i].diffuse[2] = 0.54;
		model->materials[i].diffuse[3] = 1.0;
		model->materials[i].ambient[0] = 0.0;
		model->materials[i].ambient[1] = 0.0;
		model->materials[i].ambient[2] = 0.0;
		model->materials[i].ambient[3] = 0.0;
		model->materials[i].emmissive[0] = 0.0;
		model->materials[i].emmissive[1] = 0.0;
		model->materials[i].emmissive[2] = 0.0;
		model->materials[i].emmissive[3] = 0.0;
		model->materials[i].specular[0] = 0.0; 
		model->materials[i].specular[1] = 0.0;
		model->materials[i].specular[2] = 0.0;
		model->materials[i].specular[3] = 0.0;
		model->materials[i].shininess = 0.0;
		model->materials[i].name = "default";
	}

    return model;
}




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
GLubyte* 
glmReadPPM(char* filename, int* width, int* height)
{	
	//printf("in glmReadPPM\n");
    FILE* fp;
    int i, w, h, d;
    unsigned char* image;
    char head[70];          /* max line <= 70 in PPM (per spec). */
    
    fp = fopen(filename, "rb");
    if (!fp) {
        perror(filename);
        return NULL;
    }
    
    /* grab first two chars of the file and make sure that it has the
       correct magic cookie for a raw PPM file. */
    fgets(head, 70, fp);
    if (strncmp(head, "P6", 2)) {
        fprintf(stderr, "%s: Not a raw PPM file\n", filename);
        return NULL;
    }
    //printf("It is a PPM file\n");
    /* grab the three elements in the header (width, height, maxval). */
    i = 0;
    if (head[2]==' ') {
    	char subhead[67];
    	for (int i = 0; i<67; i++)
    		subhead[i] = head[i+3];
    	printf("in subhead=%s",subhead);
    	i += sscanf(subhead, "%d %d %d", &w, &h, &d);
    }
    while(i < 3) {
        fgets(head, 70, fp);
        if (head[0] == '#')     /* skip comments. */
            continue;
        if (i == 0)
            i += sscanf(head, "%d %d %d", &w, &h, &d);
        else if (i == 1)
            i += sscanf(head, "%d %d", &h, &d);
        else if (i == 2)
            i += sscanf(head, "%d", &d);
    }
    //printf("read in w h d= %d, %d, %d\n",w,h,d);
    
    /* grab all the image data in one fell swoop. */
//    image = (unsigned char*)malloc(sizeof(unsigned char)*w*h*3);
//    fread(image, sizeof(unsigned char), w*h*3, fp);

    image = (unsigned char*)malloc(sizeof(unsigned char)*w*h*4);
    int j = h;
    while(j--)
      for( i=0; i<w; i++ )
	{
	  fread(&image[(j*w+i)*4], sizeof(unsigned char), 3, fp);
	  image[(j*w+i)*4 + 3] = 0;
	}

    fclose(fp);
    
    *width = w;
    *height = h;
    return image;
}

void glmCopyPPM( unsigned char *imgorig, unsigned char *imgcpy, int iwidth, int iheight )
{
	for( int i=0; i<iwidth*iheight; i++ )
	{
		imgcpy[i*4] = imgorig[i*4];
		imgcpy[i*4+1] = imgorig[i*4+1];
		imgcpy[i*4+2] = imgorig[i*4+2];
		imgcpy[i*4+3] = imgorig[i*4+3];

	}
}


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
bool glmWriteNOFF(GLMmodel* model, char *path)
 {
	TDGeometry geometry;
	geometry.TDGSetNumVert(model->numvertices);
	
	geometry.TDGSetNumPoly(model->numtriangles);
	
	 /* allocate memory */
    geometry.TDGInit();
    
	//copy data    
	unsigned int i;
	for( i=0; i < model->numvertices; i++ )
	{
		float temp[6];
		temp[0]=model->vertices[i*3];
		temp[1]=model->vertices[i*3+1];
		temp[2]=model->vertices[i*3+2];
		temp[3]=model->normals[i*3];
		temp[4]=model->normals[i*3+1];
		temp[5]=model->normals[i*3+2];
		
		geometry.TDGSetVertex(i, temp);
	}

	for( i=0; i < model->numtriangles; i++)
	{
		long dump[3];
		dump[0]= model->triangles[i].vindices[0];
		dump[1]= model->triangles[i].vindices[1];
		dump[2]= model->triangles[i].vindices[2];
		
	    geometry.TDGSetIndex(i, dump);
	}

	
 	geometry.TDGWriteNOFF(path );
 }

bool glmWritePPM( char *path, unsigned char *image, int width, int height)
{
	FILE	*file;
	unsigned char max = 255;

	file = fopen( path, "wb" );
	if( file == NULL )	//display error - cannot open file for reading
		return false;
	
	//write header
	fprintf( file, "P6\n" );
	//write width - height - maximum color value
	fprintf( file, "%d %d %d\n", width, height, max );
	for( int i=0; i<width*height; i++ )
	{
		fwrite( &image[i*3], sizeof(unsigned char), 3, file );
	}

	fclose( file );

	return true;
}


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
glmDraw(GLMmodel* model, GLuint mode)
{
    static GLuint i;
    static GLMgroup* group;
    static GLMtriangle* triangle;
    static GLMmaterial* material;
	static GLMstrip* strip;
    
    assert(model);
    assert(model->vertices);
    
    /* do a bit of warning */
    if (mode & GLM_FLAT && !model->facetnorms) {
        printf("glmDraw() warning: flat render mode requested "
            "with no facet normals defined.\n");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_SMOOTH && !model->normals) {
        printf("glmDraw() warning: smooth render mode requested "
            "with no normals defined.\n");
        mode &= ~GLM_SMOOTH;
    }
    if (mode & GLM_TEXTURE && !model->texcoords) {
        printf("glmDraw() warning: texture render mode requested "
            "with no texture coordinates defined.\n");
        mode &= ~GLM_TEXTURE;
    }
    if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
        printf("glmDraw() warning: flat render mode requested "
            "and smooth render mode requested (using smooth).\n");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_COLOR && !model->materials) {
        printf("glmDraw() warning: color render mode requested "
            "with no materials defined.\n");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_MATERIAL && !model->materials) {
        printf("glmDraw() warning: material render mode requested "
            "with no materials defined.\n");
        mode &= ~GLM_MATERIAL;
    }
    if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
        printf("glmDraw() warning: color and material render mode requested "
            "using only material mode.\n");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_COLOR){
    	//printf("mode & GLM_COLOR enabled\n");
        glEnable(GL_COLOR_MATERIAL);
        }
    else if (mode & GLM_MATERIAL){
    	//printf("mode & GLM_MATERIAL disabled\n");
        glDisable(GL_COLOR_MATERIAL);
        //glFrontFace(/*GL_CW or */GL_CCW); // phantom
//         glFrontFace(GL_CW /*or GL_CCW*/); // in vivo Fani
        glFrontFace(GL_CW /*or GL_CCW*/); // in vivo Pallav
        // glEnable(GL_CULL_FACE);
// 		//glDisable(GL_CULL_FACE);
//         glCullFace(GL_BACK /* or GL_FRONT or even GL_FRONT_AND_BACK */);
        }
    
    /* perhaps this loop should be unrolled into material, color, flat,
       smooth, etc. loops?  since most cpu's have good branch prediction
       schemes (and these branches will always go one way), probably
       wouldn't gain too much?  */
    
    group = model->groups;
    while (group) {
        if (mode & GLM_MATERIAL) {
        	//printf("GLM_Material....\n");
            material = &model->materials[group->material];
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material->ambient);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material->diffuse);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material->specular);
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material->shininess);
        }
        
        if (mode & GLM_COLOR) {
            glColor3fv(material->diffuse);
        }

		if ( mode & GLM_STRIP ){
			for( i=0; i<group->numstrips; i++ ) {
				strip = &( model->strips[(group->strips[i])] );
				glBegin(GL_TRIANGLE_STRIP);

				for( int j=0; j<strip->numvertices; j++ ) {
					if (mode & GLM_SMOOTH)
						glNormal3fv(&model->normals[3*strip->vindices[j]]);
//					if (mode & GLM_FLAT)
//						glNormal3fv(&model->facetnorms[3 * triangle->findex]);
//					if (mode & GLM_TEXTURE)
//						glTexCoord2fv(&model->texcoords[2 * triangle->tindices[0]]);
					glVertex3fv(&model->vertices[3*strip->vindices[j]]);

				}
				
				glEnd();
			}

		}
		else
		{
			glBegin(GL_TRIANGLES);
			//printf("glm.cpp,glBegin(GL_TRIANGLES);\n");
			//glShadeModel(GL_FLAT);
			for (i = 0; i < group->numtriangles; i++) {
				triangle = &T(group->triangles[i]);
				//glColor4f(1.0, 1.0, 1.0, 0.5);
				if (mode & GLM_FLAT){
					printf("mode& GLM_FLAT\n");
					glNormal3fv(&model->facetnorms[3 * triangle->findex]);
					}
				if (mode & GLM_SMOOTH){
					//printf("mode& GLM_SMOOTH\n");
					glNormal3fv(&model->normals[3 * triangle->nindices[0]]);
					}
				if (mode & GLM_TEXTURE){
					printf("mode& GLM_TEXTURE\n");
					glTexCoord2fv(&model->texcoords[2 * triangle->tindices[0]]);
					}
				glVertex3fv(&model->vertices[3 * triangle->vindices[0]]);
				
				if (mode & GLM_SMOOTH)
					glNormal3fv(&model->normals[3 * triangle->nindices[1]]);
				if (mode & GLM_TEXTURE)
					glTexCoord2fv(&model->texcoords[2 * triangle->tindices[1]]);
				glVertex3fv(&model->vertices[3 * triangle->vindices[1]]);
				
				if (mode & GLM_SMOOTH)
					glNormal3fv(&model->normals[3 * triangle->nindices[2]]);
				if (mode & GLM_TEXTURE)
					glTexCoord2fv(&model->texcoords[2 * triangle->tindices[2]]);
				glVertex3fv(&model->vertices[3 * triangle->vindices[2]]);
			}
			glEnd();
			
			
		}
		
		group = group->next;
    }
}


GLvoid
glmDraw(GLMmodel* model, GLuint mode, GLfloat* centreline)
{
    static GLuint i;
    static GLMgroup* group;
    static GLMtriangle* triangle;
    static GLMmaterial* material;
	static GLMstrip* strip;
    
    assert(model);
    assert(model->vertices);
    
    /* do a bit of warning */
    if (mode & GLM_FLAT && !model->facetnorms) {
        printf("glmDraw() warning: flat render mode requested "
            "with no facet normals defined.\n");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_SMOOTH && !model->normals) {
        printf("glmDraw() warning: smooth render mode requested "
            "with no normals defined.\n");
        mode &= ~GLM_SMOOTH;
    }
    if (mode & GLM_TEXTURE && !model->texcoords) {
        printf("glmDraw() warning: texture render mode requested "
            "with no texture coordinates defined.\n");
        mode &= ~GLM_TEXTURE;
    }
    if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
        printf("glmDraw() warning: flat render mode requested "
            "and smooth render mode requested (using smooth).\n");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_COLOR && !model->materials) {
        printf("glmDraw() warning: color render mode requested "
            "with no materials defined.\n");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_MATERIAL && !model->materials) {
        printf("glmDraw() warning: material render mode requested "
            "with no materials defined.\n");
        mode &= ~GLM_MATERIAL;
    }
    if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
        printf("glmDraw() warning: color and material render mode requested "
            "using only material mode.\n");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_COLOR){
    	//printf("mode & GLM_COLOR enabled\n");
        glEnable(GL_COLOR_MATERIAL);
        }
    else if (mode & GLM_MATERIAL){
    	//printf("mode & GLM_MATERIAL disabled\n");
        glDisable(GL_COLOR_MATERIAL);
        glFrontFace(GL_CW /*or GL_CCW */);
        // glEnable(GL_CULL_FACE);
// 		//glDisable(GL_CULL_FACE);
//         glCullFace(GL_BACK /* or GL_FRONT or even GL_FRONT_AND_BACK */);
        }
    
    /* perhaps this loop should be unrolled into material, color, flat,
       smooth, etc. loops?  since most cpu's have good branch prediction
       schemes (and these branches will always go one way), probably
       wouldn't gain too much?  */
    
    group = model->groups;
    while (group) {
        if (mode & GLM_MATERIAL) {
        	//printf("GLM_Material....\n");
            material = &model->materials[group->material];
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material->ambient);
            glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material->diffuse);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material->specular);
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material->shininess);
        }
        
        if (mode & GLM_COLOR) {
            glColor3fv(material->diffuse);
        }

		if ( mode & GLM_STRIP ){
			for( i=0; i<group->numstrips; i++ ) {
				strip = &( model->strips[(group->strips[i])] );
				glBegin(GL_TRIANGLE_STRIP);

				for( int j=0; j<strip->numvertices; j++ ) {
					if (mode & GLM_SMOOTH)
						glNormal3fv(&model->normals[3*strip->vindices[j]]);
//					if (mode & GLM_FLAT)
//						glNormal3fv(&model->facetnorms[3 * triangle->findex]);
//					if (mode & GLM_TEXTURE)
//						glTexCoord2fv(&model->texcoords[2 * triangle->tindices[0]]);
					glVertex3fv(&model->vertices[3*strip->vindices[j]]);

				}
				
				glEnd();
			}

		}
		else
		{
			glBegin(GL_TRIANGLES);
			printf("glm.cpp,glBegin(GL_TRIANGLES);\n");
			//glShadeModel(GL_FLAT);
			for (i = 0; i < group->numtriangles; i++) {
				triangle = &T(group->triangles[i]);
				//glColor4f(1.0, 1.0, 1.0, 0.5);
				if (mode & GLM_FLAT){
					printf("mode& GLM_FLAT\n");
					glNormal3fv(&model->facetnorms[3 * triangle->findex]);
					}
				if (mode & GLM_SMOOTH){
					//printf("mode& GLM_SMOOTH\n");
					glNormal3fv(&model->normals[3 * triangle->nindices[0]]);
					}
				if (mode & GLM_TEXTURE){
					printf("mode& GLM_TEXTURE\n");
					glTexCoord2fv(&model->texcoords[2 * triangle->tindices[0]]);
					}
				glVertex3fv(&model->vertices[3 * triangle->vindices[0]]);
				
				if (mode & GLM_SMOOTH)
					glNormal3fv(&model->normals[3 * triangle->nindices[1]]);
				if (mode & GLM_TEXTURE)
					glTexCoord2fv(&model->texcoords[2 * triangle->tindices[1]]);
				glVertex3fv(&model->vertices[3 * triangle->vindices[1]]);
				
				if (mode & GLM_SMOOTH)
					glNormal3fv(&model->normals[3 * triangle->nindices[2]]);
				if (mode & GLM_TEXTURE)
					glTexCoord2fv(&model->texcoords[2 * triangle->tindices[2]]);
				glVertex3fv(&model->vertices[3 * triangle->vindices[2]]);
			}
			glEnd();
			// plot centreline here
			//glLineWidth(50.0f);
			glBegin(GL_LINES);
			// origin: -142.75 * -248.25 * 805.5
			// voxel spacing: 0.5* 0.5* 0.700012
			for (int i = 0; i<660; i++){
				//printf("%f,%f,%f",centreline[3*i],centreline[3*i+1],centreline[3*i+2]);
  				glVertex3f(centreline[3*i], centreline[3*i+1], centreline[3*i+2]);
  				//glVertex3f(centreline[3*i+3], centreline[3*i+4], centreline[3*i+5]);
  				}
			glEnd();
			
		}
		
		group = group->next;
    }
}

/* glmList: Generates and returns a display list for the model using
 * the mode specified.
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
 * GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLuint
glmList(GLMmodel* model, GLuint mode, GLfloat* centreline)
{
    GLuint list;
    
    list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glmDraw(model, mode, centreline);
    glEndList();
    
    return list;
}

/* glmFacetNormals: Generates facet normals for a model (by taking the
 * cross product of the two vectors derived from the sides of each
 * triangle).  Assumes a counter-clockwise winding.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmFacetNormals(GLMmodel* model)
{
    GLuint  i;
    GLfloat u[3];
    GLfloat v[3];
    
    assert(model);
    assert(model->vertices);
    
    /* clobber any old facetnormals */
    if (model->facetnorms)
        free(model->facetnorms);
    
    /* allocate memory for the new facet normals */
    model->numfacetnorms = model->numtriangles;
    model->facetnorms = (GLfloat*)malloc(sizeof(GLfloat) *
                       3 * (model->numfacetnorms + 1));
    
    for (i = 0; i < model->numtriangles; i++) {
        model->triangles[i].findex = i+1;
        
        u[0] = model->vertices[3 * T(i).vindices[1] + 0] -
            model->vertices[3 * T(i).vindices[0] + 0];
        u[1] = model->vertices[3 * T(i).vindices[1] + 1] -
            model->vertices[3 * T(i).vindices[0] + 1];
        u[2] = model->vertices[3 * T(i).vindices[1] + 2] -
            model->vertices[3 * T(i).vindices[0] + 2];
        
        v[0] = model->vertices[3 * T(i).vindices[2] + 0] -
            model->vertices[3 * T(i).vindices[0] + 0];
        v[1] = model->vertices[3 * T(i).vindices[2] + 1] -
            model->vertices[3 * T(i).vindices[0] + 1];
        v[2] = model->vertices[3 * T(i).vindices[2] + 2] -
            model->vertices[3 * T(i).vindices[0] + 2];
        
        //glmCross(u, v, &model->facetnorms[3 * (i+1)]);
        glmCross(v,u, &model->facetnorms[3 * (i+1)]);
        glmNormalize(&model->facetnorms[3 * (i+1)]);
    }
}

/* glmVertexNormals: Generates smooth vertex normals for a model.
 * First builds a list of all the triangles each vertex is in.   Then
 * loops through each vertex in the the list averaging all the facet
 * normals of the triangles each vertex is in.   Finally, sets the
 * normal index in the triangle for the vertex to the generated smooth
 * normal.   If the dot product of a facet normal and the facet normal
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
glmVertexNormals(GLMmodel* model, GLfloat angle)
{
    GLMnode*    node;
    GLMnode*    tail;
    GLMnode** members;
    GLfloat*    normals;
    GLuint  numnormals;
    GLfloat average[3];
    GLfloat dot, cos_angle;
    GLuint  i, avg;
    
    assert(model);
    assert(model->facetnorms);
    
    /* calculate the cosine of the angle (in degrees) */
    cos_angle = cos(angle * M_PI / 180.0);
    
    /* nuke any previous normals */
    if (model->normals)
        free(model->normals);
    
    /* allocate space for new normals */
    model->numnormals = model->numtriangles * 3; /* 3 normals per triangle */
    model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));
    
    /* allocate a structure that will hold a linked list of triangle
    indices for each vertex */
    members = (GLMnode**)malloc(sizeof(GLMnode*) * (model->numvertices + 1));
    for (i = 1; i <= model->numvertices; i++)
        members[i] = NULL;
    
    /* for every triangle, create a node for each vertex in it */
    for (i = 0; i < model->numtriangles; i++) {
        node = (GLMnode*)malloc(sizeof(GLMnode));
        node->index = i;
        node->next  = members[T(i).vindices[0]];
        members[T(i).vindices[0]] = node;
        
        node = (GLMnode*)malloc(sizeof(GLMnode));
        node->index = i;
        node->next  = members[T(i).vindices[1]];
        members[T(i).vindices[1]] = node;
        
        node = (GLMnode*)malloc(sizeof(GLMnode));
        node->index = i;
        node->next  = members[T(i).vindices[2]];
        members[T(i).vindices[2]] = node;
    }
    
    /* calculate the average normal for each vertex */
    numnormals = 1;
    for (i = 1; i <= model->numvertices; i++) {
    /* calculate an average normal for this vertex by averaging the
        facet normal of every triangle this vertex is in */
        node = members[i];
        if (!node)
            fprintf(stderr, "glmVertexNormals(): vertex w/o a triangle\n");
        average[0] = 0.0; average[1] = 0.0; average[2] = 0.0;
        avg = 0;
        while (node) {
        /* only average if the dot product of the angle between the two
        facet normals is greater than the cosine of the threshold
        angle -- or, said another way, the angle between the two
            facet normals is less than (or equal to) the threshold angle */
            dot = glmDot(&model->facetnorms[3 * T(node->index).findex],
                &model->facetnorms[3 * T(members[i]->index).findex]);
            if (dot > cos_angle) {
                node->averaged = GL_TRUE;
                average[0] += model->facetnorms[3 * T(node->index).findex + 0];
                average[1] += model->facetnorms[3 * T(node->index).findex + 1];
                average[2] += model->facetnorms[3 * T(node->index).findex + 2];
                avg = 1;            /* we averaged at least one normal! */
            } else {
                node->averaged = GL_FALSE;
            }
            node = node->next;
        }
        
        if (avg) {
            /* normalize the averaged normal */
            glmNormalize(average);
            
            /* add the normal to the vertex normals list */
            model->normals[3 * numnormals + 0] = average[0];
            model->normals[3 * numnormals + 1] = average[1];
            model->normals[3 * numnormals + 2] = average[2];
            avg = numnormals;
            numnormals++;
        }
        
        /* set the normal of this vertex in each triangle it is in */
        node = members[i];
        while (node) {
            if (node->averaged) {
                /* if this node was averaged, use the average normal */
                if (T(node->index).vindices[0] == i)
                    T(node->index).nindices[0] = avg;
                else if (T(node->index).vindices[1] == i)
                    T(node->index).nindices[1] = avg;
                else if (T(node->index).vindices[2] == i)
                    T(node->index).nindices[2] = avg;
            } else {
                /* if this node wasn't averaged, use the facet normal */
                model->normals[3 * numnormals + 0] = 
                    model->facetnorms[3 * T(node->index).findex + 0];
                model->normals[3 * numnormals + 1] = 
                    model->facetnorms[3 * T(node->index).findex + 1];
                model->normals[3 * numnormals + 2] = 
                    model->facetnorms[3 * T(node->index).findex + 2];
                if (T(node->index).vindices[0] == i)
                    T(node->index).nindices[0] = numnormals;
                else if (T(node->index).vindices[1] == i)
                    T(node->index).nindices[1] = numnormals;
                else if (T(node->index).vindices[2] == i)
                    T(node->index).nindices[2] = numnormals;
                numnormals++;
            }
            node = node->next;
        }
    }
    
    model->numnormals = numnormals - 1;
    
    /* free the member information */
    for (i = 1; i <= model->numvertices; i++) {
        node = members[i];
        while (node) {
            tail = node;
            node = node->next;
            free(tail);
        }
    }
    free(members);
    
    /* pack the normals array (we previously allocated the maximum
    number of normals that could possibly be created (numtriangles *
    3), so get rid of some of them (usually alot unless none of the
    facet normals were averaged)) */
    normals = model->normals;
    model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));
    for (i = 1; i <= model->numnormals; i++) {
        model->normals[3 * i + 0] = normals[3 * i + 0];
        model->normals[3 * i + 1] = normals[3 * i + 1];
        model->normals[3 * i + 2] = normals[3 * i + 2];
    }
    free(normals);
}

/* glmCross: compute the cross product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 * n - array of 3 GLfloats (GLfloat n[3]) to return the cross product in
 */
static GLvoid
glmCross(GLfloat* u, GLfloat* v, GLfloat* n)
{
    assert(u); assert(v); assert(n);
    
    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];
}


/* glmNormalize: normalize a vector
 *
 * v - array of 3 GLfloats (GLfloat v[3]) to be normalized
 */
static GLvoid
glmNormalize(GLfloat* v)
{
    GLfloat l;
    
    assert(v);
    
    l = (GLfloat)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] /= l;
    v[1] /= l;
    v[2] /= l;
}

/* glmDot: compute the dot product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 */
static GLfloat
glmDot(GLfloat* u, GLfloat* v)
{
    assert(u); assert(v);
    
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}




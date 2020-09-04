#include <math.h>
//#include <itkTransformation.h>
//#include <itkAFreeFormTransformation.h>

#include <vector>
//using std::vector;
using namespace std;

#include "glm.h"


typedef vector<long> vector_long;
typedef vector<float> vector_float;

static GLMmodel *copy_def[2] = { NULL, NULL };


void RestoreDef( GLMmodel *model, int which )
{
	if( copy_def[which] )
	{
		if( model )
			glmDelete( model );
		glmCopyModel( copy_def[which], &model );
	}
	else
		glmCopyModel( model, &copy_def[which] );
}


void backupModel( GLMmodel *model, int which )
{
	if( copy_def[which] )
		glmDelete( copy_def[which] );
	glmCopyModel( model, &copy_def[which] );
}

void ApplyDeformationSphere( GLMmodel *model, float radius, float *sphere_center, float deform )
{
	GLMpoint center; 
	center.x = sphere_center[0];
	center.y = sphere_center[1];
	center.z = sphere_center[2];
	
	float *vertices;
	long numvertices = model->numvertices;
	for( long i=0; i<numvertices; i++ )
	{
				vertices = &model->vertices[3*i];
				float sphere = (vertices[0]-center.x)*(vertices[0]-center.x)+
        						(vertices[1]-center.y)*(vertices[1]-center.y)+
								(vertices[2]-center.z)*(vertices[2]-center.z) - radius*radius;
				if( sphere <= 0 )	//then apply deformation
				{
					float dist = sqrt( (vertices[0]-center.x)*(vertices[0]-center.x)+
        							   (vertices[1]-center.y)*(vertices[1]-center.y)+
								       (vertices[2]-center.z)*(vertices[2]-center.z) ) / radius;
					float def = 2.0*dist*dist*dist - 3.0*dist*dist + 1.0;
					def *= (radius*deform);
					float len = sqrt( vertices[0]*vertices[0]+vertices[1]*vertices[1]+vertices[2]*vertices[2] );

					vertices[0] = vertices[0] - vertices[0]*def/len; 
					vertices[1] = vertices[1] - vertices[1]*def/len; 
					vertices[2] = vertices[2] - vertices[2]*def/len; 
				}
	}

	glmFacetNormals( model );
	glmVertexNormals( model, 90.0 );
}

/*
static itkAFreeFormTransformation *Transform = NULL;

void FFDinitial( GLMmodel *model, float *sphere_center, float radius )
{
	//define two diametric opposite points of the square
	float x1 = sphere_center[0] - radius;
	float y1 = sphere_center[1] - radius;
	float z1 = sphere_center[2] + radius;
	float x2 = sphere_center[0] + radius;
	float y2 = sphere_center[1] + radius;
	float z2 = sphere_center[2] - radius;
	int numcontrx = 5;	//number of control points in x
	int numcontry = 5;	//number of control points in y
	int numcontrz = 5;	//number of control points in z

	Transform = new itkAFreeFormTransformation( x1, y1, z1, x2, y2, z2, numcontrx, numcontry, numcontrz );
}

void FFDdisplay()
{
	if( !Transform )
		return;

	int numx = Transform->GetX();	//get the number of control points on X
	int numy = Transform->GetY();	//get the number of control points on Y
	int numz = Transform->GetZ();	//get the number of control points on Z

}



*/

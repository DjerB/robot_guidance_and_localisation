

#ifndef DEFORMATION_H
#define DEFORMATION_H

void ApplyDeformationSphere( GLMmodel *model, float radius, float *sphere_center, float deform );
void RestoreDef( GLMmodel *model, int which );
void backupModel( GLMmodel *model, int which );

#endif

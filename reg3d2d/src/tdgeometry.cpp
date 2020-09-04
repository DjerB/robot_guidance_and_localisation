//   TDGeometry.cpp

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "tdgeometry.h"
#include "mylist.h"
#include "global.h" 

using namespace std;

///////////////////////////////////////////////////////////////////////////
//	Implementation class TDGeometry	//


TDGeometry::TDGeometry( void )
{
	Polygons = NULL;
	Vertexes = NULL;
	NumVertexes = 0;
	NumPolygons = 0;
	NumEdges = 0;
	NumStrips = 0;
	Info.calc = false;
}

TDGeometry::~TDGeometry( void )
{
	if( Polygons != NULL )
		delete [] Polygons;
	if( Vertexes  != NULL )
		delete [] Vertexes;
	if( Strips != NULL )
	{
		for( int i=0; i<NumStrips; i++ )
			delete Strips[i].list;
		delete [] Strips;
	}

	NumVertexes = 0;
	NumPolygons = 0;
	NumEdges = 0;
	NumStrips = 0;
	Info.calc = false;
}


void TDGeometry::TDGInit( void )
{
	Vertexes = new stvertex[NumVertexes];
	Polygons = new stfacetl[NumPolygons];
	Strips	 = new ststrip[NumStrips];
}


int TDGeometry::TDGReadNOFF( char *path )
{
	cout << "trying to open model file as NOFF file" << endl;
	Type = 0;	//0 =>error, 1 =>NOFF, 2 =>NOFF - STRIPS

	std::ifstream fin(path);
	if( !fin )
		return Type;

	//read header
	char header[15];
	fin.getline( header, 15 );
	// cout << header[0]<<'/'<<header[1]<<'/'<<header[2]<<'/'<<header[3]<<'/'<<header[4]<<'/';
// 	cout << header[5]<<'/'<<header[6]<<"/"<<header[7]<<"/"<<header[8]<<"/"<<header[9]<<"/"<<header[10]<<"/";
// 	cout <<header[11]<<"/"<<header[12]<<"/"<<header[13]<<"/"<<header[14]<<endl;
	if (header[0]=='N' && header[1]=='O' && header[2]=='F' && header[3]=='F') {
		//printf("First 4 chars are NOFF\n");
	//if( !strcmp( header, "NOFF" ) )	{
		if (header[4]=='\0') {
		Type++;
		//printf("input file is NOFF\n");
		}
		if (header[4]==' ' && header[5]=='-' && header[6]==' ' && header[7]=='S' && header[8]=='T' && header[9]=='R'&&header[10]=='I'&& header[11]=='P'&& header[12]=='S') {
	//else if( !strcmp( header, "NOFF - STRIPS" ) ){
		printf("input file is NOFF-STRIPS\n");
		Type = 2;
		}
		}
	else{
		//display error - file format does not agree with the specifications
		fprintf(stderr,"bad NOFF header: %s\n",header);
		fin.close();
		return Type;
	}

	if( Type == 1 )		//NOFF: read triangles
	{
		//read number of vertexes, number of triangles, edges
		fin >> NumVertexes >> NumPolygons >> NumEdges;

		//allocate memory - initialize
		TDGInit();

		//read vertexes
		//std::cout<<"reading "<<NumVertexes<<" vertices..."<<std::endl;
		for( int i=0; i<NumVertexes; i++ )
		{
			float temp[6];
			fin >> temp[0] >> temp[1] >> temp[2] >> temp[3] >> temp[4] >> temp[5];
			Vertexes[i].p[0] = temp[0]; 
			Vertexes[i].p[1] = temp[1]; 
			Vertexes[i].p[2] = temp[2]; 
			Vertexes[i].n[0] = temp[3];
			Vertexes[i].n[1] = temp[4];
			Vertexes[i].n[2] = temp[5];
		}

		//read polygons and store indexes
		//std::cout<<"reading "<<NumPolygons<<" polygons..."<<std::endl;
		for( int i=0; i<NumPolygons; i++ )
		{
			long dump[4];
			fin >> dump[0] >> dump[1] >> dump[2] >> dump[3];
			Polygons[i].findex[0] = dump[1];
			Polygons[i].findex[1] = dump[2];
			Polygons[i].findex[2] = dump[3];
		}

	}
	else if( Type ==2 )	//NOFF - STRIPS: read strips
	{
		//read number of vertexes, number of strips
		fin >> NumVertexes >> NumStrips;

		//allocate memory - initialize
		TDGInit();

		//read vertexes
		std::cout<<"reading "<<NumVertexes<<" vertices..."<<std::endl;
		for( int i=0; i<NumVertexes; i++ )
		{
			float temp[6];
			fin >> temp[0] >> temp[1] >> temp[2] >> temp[3] >> temp[4] >> temp[5];
			Vertexes[i].p[0] = temp[0]; 
			Vertexes[i].p[1] = temp[1]; 
			Vertexes[i].p[2] = temp[2]; 
			Vertexes[i].n[0] = temp[3];
			Vertexes[i].n[1] = temp[4];
			Vertexes[i].n[2] = temp[5];
		}

		//read stips and store indexes
		std::cout<<"reading "<<NumStrips<<" strips..."<<std::endl;
		for( int i=0; i<NumStrips; i++ )
		{
			int len;
			fin >> len;
			Strips[i].length = len;
			Strips[i].list = new int[len];
			for( int j=0; j<len; j++ )
				fin >> Strips[i].list[j];
		}

	}

	fin.close();

	return Type;
}

int TDGeometry::TDGReadOFF( char *path )
{
	cout << "trying to open model file as OFF file" << endl;
	Type = 0;	//0 =>error, 1 =>NOFF, 2 =>NOFF - STRIPS

	std::ifstream fin(path);
	if( !fin )
		return Type;

	//read header
	char header[5];
	fin.getline( header, 5 );
	cout << header<<endl;
	// cout <<"1."<< header[0]<<" 2."<<header[1]<<" 3."<<header[2]<<" 4."<<header[3]<<" 5."<<header[4]<<endl;
// 	if (header[0]==' ' && header[1]=='F' && header[2]=='F' && header[3]=='\0') 
		Type++;

	if( Type == 1 )		//NOFF: read triangles
	{
		//read number of vertexes, number of triangles, edges
		fin >> NumVertexes >> NumPolygons >> NumEdges;

		//allocate memory - initialize
		TDGInit();

		//read vertexes
		std::cout<<"reading "<<NumVertexes<<" vertices..."<<std::endl;
		for( int i=0; i<NumVertexes; i++ )
		{
			float temp[3];
			fin >> temp[0] >> temp[1] >> temp[2];
			Vertexes[i].p[0] = temp[0]; 
			Vertexes[i].p[1] = temp[1]; 
			Vertexes[i].p[2] = temp[2]; 
			Vertexes[i].n[0] = 0;
			Vertexes[i].n[1] = 0;
			Vertexes[i].n[2] = 0;
		}

		//read polygons and store indexes
		std::cout<<"reading "<<NumPolygons<<" polygons..."<<std::endl;
		for( int i=0; i<NumPolygons; i++ )
		{
			long dump[4];
			fin >> dump[0] >> dump[1] >> dump[2] >> dump[3];
			Polygons[i].findex[0] = dump[1];
			Polygons[i].findex[1] = dump[2];
			Polygons[i].findex[2] = dump[3];
		}

	}
	
	fin.close();

	return Type;
}
bool TDGeometry::TDGWriteNOFF( char * path )
{
	FILE	*file;
	file = fopen( path, "wt" );
	if( file == NULL )	//display error - cannot open file for reading
		return false;
	
	//write header
	fprintf( file, "NOFF\n" );
	//write number of vertexes - number of polygons - number of edges
	fprintf( file, "%d %d %d\n", NumVertexes, NumPolygons, NumEdges );

	//write number of vertixes
	int i;
	for( i=0; i<NumVertexes; i++ )
	{
		fprintf( file, "%f %f %f %f %f %f\n", 
			Vertexes[i].p[0], Vertexes[i].p[1], Vertexes[i].p[2], Vertexes[i].n[0], Vertexes[i].n[1], Vertexes[i].n[2] );
	}

	//write number of polygons
	for( i=0; i<NumPolygons; i++ )
	{
		fprintf( file, "3 %d %d %d\n", Polygons[i].findex[0], Polygons[i].findex[1], Polygons[i].findex[2]);
	}

	fclose( file );

	return true;
}

void TDGeometry::TDGScale( float scale )
{
	TDGGetInfo( true );		

	float infomatrix[6];

	infomatrix[0] = Info.maxXYZ[0];
	infomatrix[1] = Info.maxXYZ[1];
	infomatrix[2] = Info.maxXYZ[2];
	infomatrix[3] = Info.minXYZ[0];
	infomatrix[4] = Info.minXYZ[1];
	infomatrix[5] = Info.minXYZ[2];

	Myabs( infomatrix, 6 );
	float max = Mymax( infomatrix, 6 );

	if( max && scale )
	{
		for( int i=0; i<NumVertexes; i++ )
		{
			Vertexes[i].p[0] = Vertexes[i].p[0] * scale / max; 
			Vertexes[i].p[1] = Vertexes[i].p[1] * scale / max; 
			Vertexes[i].p[2] = Vertexes[i].p[2] * scale / max; 
		}
	}
}

long TDGeometry::TDGGetNumPoly( void )
{
	return NumPolygons;
}

void TDGeometry::TDGSetNumPoly( long NumPoly )
{
	NumPolygons=NumPoly;
}

long TDGeometry::TDGGetNumVert( void )
{
	return NumVertexes;
}

void TDGeometry::TDGSetNumVert( long NumVert )
{
	NumVertexes=NumVert;
}

long TDGeometry::TDGGetNumEdge( void )
{
	return NumEdges;
}

void TDGeometry::TDGSetNumEdge( long NumEd )
{
	NumEdges=NumEd;
}

long TDGeometry::TDGGetNumStrip( void )
{
	return NumStrips;
}

stvertex TDGeometry::TDGGetVertex( long i )
{
	return Vertexes[i];
}

void TDGeometry::TDGSetVertex( long i, float* temp)
{
			Vertexes[i].p[0] = temp[0]; 
			Vertexes[i].p[1] = temp[1]; 
			Vertexes[i].p[2] = temp[2]; 
			Vertexes[i].n[0] = temp[3];
			Vertexes[i].n[1] = temp[4];
			Vertexes[i].n[2] = temp[5];
}

stfacetl TDGeometry::TDGGetIndex( long i )
{
	return Polygons[i];
}

void TDGeometry::TDGSetIndex( long i, long* dump)
{
			Polygons[i].findex[0] = dump[1];
			Polygons[i].findex[1] = dump[2];
			Polygons[i].findex[2] = dump[3];
}

ststrip TDGeometry::TDGGetStrip( long i )
{
	return Strips[i];
}

stinfo TDGeometry::TDGGetInfo( bool recalc )
{
	//recalc = true: forces recalculation of info
	//recalc = false: calculate info only if it is not available

	if( (recalc && NumVertexes) || (!recalc && NumVertexes && !Info.calc) )
	{
		float max[3], min[3], aver[3];

		max[0] = Vertexes[0].p[0];
		max[1] = Vertexes[0].p[1];
		max[2] = Vertexes[0].p[2];

		min[0] = Vertexes[0].p[0];
		min[1] = Vertexes[0].p[1];
		min[2] = Vertexes[0].p[2];

		aver[0] = 0;
		aver[1] = 0;
		aver[2] = 0;

		for( int i=0; i<NumVertexes; i++ )
		{
			if( Vertexes[i].p[0] > max[0] )
				max[0] = Vertexes[i].p[0];
			if( Vertexes[i].p[0] < min[0] )
				min[0] = Vertexes[i].p[0];
			aver[0] = aver[0] + Vertexes[i].p[0];
			
			if( Vertexes[i].p[1] > max[1] )
				max[1] = Vertexes[i].p[1];
			if( Vertexes[i].p[1] < min[1] )
				min[1] = Vertexes[i].p[1];
			aver[1] = aver[1] + Vertexes[i].p[1];
			
			if( Vertexes[i].p[2] > max[2] )
				max[2] = Vertexes[i].p[2];
			if( Vertexes[i].p[2] < min[2] )
				min[2] = Vertexes[i].p[2];
			aver[2] = aver[2] + Vertexes[i].p[2];
		}

		aver[0] = aver[0] / NumVertexes;
		aver[1] = aver[1] / NumVertexes;
		aver[2] = aver[2] / NumVertexes;

		Info.averXYZ[0] = aver[0];
		Info.averXYZ[1] = aver[1];
		Info.averXYZ[2] = aver[2];

		Info.maxXYZ[0] = max[0];
		Info.maxXYZ[1] = max[1];
		Info.maxXYZ[2] = max[2];

		Info.minXYZ[0] = min[0];
		Info.minXYZ[1] = min[1];
		Info.minXYZ[2] = min[2];

		Info.dimXYZ[0] = Myabs( max[0] ) + Myabs( min[0] );
		Info.dimXYZ[1] = Myabs( max[1] ) + Myabs( min[1] );
		Info.dimXYZ[2] = Myabs( max[2] ) + Myabs( min[2] );

		Info.calc = true;
	}

	return Info;

}






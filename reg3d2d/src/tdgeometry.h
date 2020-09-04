//   TDGeometry.h 

//store vertex point: p[3] and normal: n[3]
struct stvertex{		
	float p[3];
	float n[3];
};

//store polygon index 
struct stfacetl{
	long findex[3];		//import only triangles
};

struct stinfo{
	float maxXYZ[3];
	float minXYZ[3];
	float dimXYZ[3];	//Dimensions of the model:Width->x, Height->y, Depth->z
	float averXYZ[3];
	bool calc;	//true when stinfo has been calculated, false otherwise
};

struct ststrip{
	int length;
	int *list;
};

class TDGeometry{
private:
	int Type;
	stvertex *Vertexes;
	stfacetl *Polygons;
	ststrip	 *Strips;
	long NumVertexes;		//number of vertexes
	long NumPolygons;		//number of polygons
	long NumEdges;			//number of edges
	long NumStrips;			//number of strips
	stinfo Info;

public:
	TDGeometry( void );
	~TDGeometry( void );
	void TDGInit( void );
	int TDGReadNOFF( char *path );		//Read NOFF files 
	int TDGReadOFF( char *path );
	bool TDGWriteNOFF( char * path );	//Write NOFF files
	void TDGScale( float scale );		//Scale vertex data in order 'scale' to be the maximum
	long TDGGetNumPoly( void );
	void TDGSetNumPoly( long NumPoly );
	long TDGGetNumVert( void );
	void TDGSetNumVert( long NumVert );
	long TDGGetNumEdge( void );
	void TDGSetNumEdge( long NumEd );
	long TDGGetNumStrip( void );
	stvertex TDGGetVertex( long i );
	void TDGSetVertex( long i, float* temp);
	stfacetl TDGGetIndex( long i );
	void TDGSetIndex( long i, long* dump);
	ststrip TDGGetStrip( long i );
	stinfo TDGGetInfo( bool recalc );	//recalculate info information when recalc = true
};


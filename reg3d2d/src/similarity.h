//	similarity.h --> calculate similarity measure between two images.....

#define MAX 9000000

class Similarity{
	int Height;
	int Width;
	int Mode;
	float FactorR;
	float FactorG;
	float FactorB;
public:
	Similarity(void);		//in order to initialize properly the object, a mode and images dimensions should be defined
	~Similarity(void);
	void init( int height, int width );
	void init( int mode );
	void init( float factorR, float factorG, float factorB ); 
    //template<class T> float findsimilarity( T *image1, T *image2 );	//calculate similarity between two images
	float findsimilarity( unsigned char *image1, unsigned char *image2 );	//calculate similarity between two images
	float findsimilarityDepth(float *image1, float _minDepth, float _maxDepth,float *image2 );
	float findsimilarityDepthMI(float *image1, float _minDepth, float _maxDepth,float *image2 );
	template<class T> float usecorr( T *image1, T *image2 );	//when mode = 1 use correlation measure
	template<class T> float usemutualInfo( T *image1, T *image2 );	//when mode = 1 use correlation measure
};

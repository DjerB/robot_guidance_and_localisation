// global.h 
//define global function as templates for general use

#include <assert.h>

#ifdef WIN32

#include <windows.h>

#endif
#include <FL/gl.h>

//////////////////////////////////////////////////////////////////

template<class T>
T Mymin( T *matrix, int size )
{
	if( size > 0 )
	{
		T elem = matrix[0];
		for( int i=0; i<size; i++ )
		{
			if( elem > matrix[i] )
				elem = matrix[i];
		}
		return elem;
	}
		
	return 0;
}


template<class T>

T Mymax( T a, T b )

{

	if( a >= b )

		return a;

	else

		return b;

}


template<class T>
T Mymax( T *matrix, int size )
{
	if( size > 0 )
	{
		T elem = matrix[0];
		for( int i=0; i<size; i++ )
		{
			if( elem < matrix[i] )
				elem = matrix[i];
		}
		return elem;
	}
		
	return 0;
}

template<class T>
T Myabs( T elem)
{
	if( elem<0 )
		return -elem;
	else 
		return elem;
}

template<class T>
bool Myabs( T *matrix, int size)
{
	if( size > 0 )
	{
		for( int i=0; i<size; i++ )
		{
			if( matrix[i] < 0 )
				 matrix[i] = -matrix[i];
		}
		return true;
	}

	return false;
}

//////////////////////////////////////////////////////////////////

/* MyDotProd: compute the dot product of two vectors
 * u - array of 3 T
 * v - array of 3 T
*/
template<class T>
T MyDotProd( T* u, T* v )
{
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

/* MyCrossProd: compute the cross product of two vectors
 *
 * u - array of 3 T
 * v - array of 3 T
 * n - array of 3 T to return the cross product in
 */
template<class T>
void MyCrossProd( T* u, T* v, T* n)
{
    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];
}

/* MyNormalize: normalize a vector
 *
 * v - array of 3 T to be normalized
 */
template<class T>
T MyNormalize(T* v)
{
    T len;
    
    len = (GLfloat)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

	assert(len);	//abort if len=0

    v[0] /= len;
    v[1] /= len;
    v[2] /= len;

	return len;
}

/* MyIdentity: produces an 4 by 4 identity matrix
 * m - the 4 by 4 identity matrix produced
 */
template<class T>
void identity(T m[16])
{
    m[0+4*0] = 1; m[0+4*1] = 0; m[0+4*2] = 0; m[0+4*3] = 0;
    m[1+4*0] = 0; m[1+4*1] = 1; m[1+4*2] = 0; m[1+4*3] = 0;
    m[2+4*0] = 0; m[2+4*1] = 0; m[2+4*2] = 1; m[2+4*3] = 0;
    m[3+4*0] = 0; m[3+4*1] = 0; m[3+4*2] = 0; m[3+4*3] = 1;
}


/* MyInvertArray: Invert 4 by 4 square matrix
 *
 * src - array of 16 T to be inverted
 * inverse - array of 16 T to store result
 */
template<class T>
bool MyInvertArray( T src[16], T inverse[16])
{
    T t;
    int i, j, k, swap;
    T tmp[4][4];
    
    identity(inverse);
    
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tmp[i][j] = src[i*4+j];
        }
    }
    
    for (i = 0; i < 4; i++) {
        // look for largest element in column. //
        swap = i;
        for (j = i + 1; j < 4; j++) {
            if (fabs(tmp[j][i]) > fabs(tmp[i][i])) {
                swap = j;
            }
        }
        
        if (swap != i) {
            // swap rows. //
            for (k = 0; k < 4; k++) {
                t = tmp[i][k];
                tmp[i][k] = tmp[swap][k];
                tmp[swap][k] = t;
                
                t = inverse[i*4+k];
                inverse[i*4+k] = inverse[swap*4+k];
                inverse[swap*4+k] = t;
            }
        }
        
        if (tmp[i][i] == 0) {
			// no non-zero pivot.  the matrix is singular, which shouldn't happen. 
			// This means the user gave us a bad matrix. 
            return false;
        }
        
        t = tmp[i][i];
        for (k = 0; k < 4; k++) {
            tmp[i][k] /= t;
            inverse[i*4+k] /= t;
        }
        for (j = 0; j < 4; j++) {
            if (j != i) {
                t = tmp[j][i];
                for (k = 0; k < 4; k++) {
                    tmp[j][k] -= tmp[i][k]*t;
                    inverse[j*4+k] -= inverse[i*4+k]*t;
                }
            }
        }
    }
    return true;
}





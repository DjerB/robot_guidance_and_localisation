//   MyList.h 
// define the class using template  



template<class T>
struct myelement{
	T elem;
	myelement *next;
};

template<class T>
class MyList{
private:
	long length;
	bool init;
	myelement<T> *start;
	myelement<T> *last;
public:
	MyList();
	~MyList();
	bool DestroyList();
	void AddElement(T elem );
	bool DeleteElement( long i ); 
	long GetLength();
	bool CopyToVector( T *array );
};


///////////////////////////////////////////////////////////////////////////
//	Implementation class MyList	//
//!! I cannot compile it when it is in a seperate file.....


template<class T>
MyList<T>::MyList()
{
	length = 0;
	init = false;
	start = NULL;
	last = NULL;
}

template<class T>
MyList<T>::~MyList()
{
	bool check = DestroyList();
}

template<class T>
bool MyList<T>::DestroyList()
{
	myelement<T> *temp1, *temp;
	temp = start;

	long i = 0;

	while(1)
	{
		if( init )
		{
			i++;

			if( temp == NULL && i==length+1 )
			{
				init = false;
				length = 0;
				return true;
			}
			else if( temp !=NULL && i<=length)
			{
				temp1 = temp->next;
				delete temp;
				temp = temp1;
			}
			else
				return false;

		}
		else
			return true;
	}

}

template<class T>
void MyList<T>::AddElement( T elem )
{
	myelement<T> *temp = new myelement<T>;
	temp->elem = elem;
	temp->next = NULL;


	if( !init )
	{
		start = temp;
		init = true;
	}
	else
		last->next = temp;

	last = temp;

	length++;
}

template<class T>
bool MyList<T>::DeleteElement( long i )	//this function has not been tested yet!!!!
{
	if( !init || i>length )
		return false;

	if( length == 1 )
		DestroyList();

	myelement<T> *temp1, *temp2, *temp3;
	temp1 = start->next;
	for( int j=1; j<=i; j++ )
	{
		temp2 = temp1->next;
		temp1 = temp2;
		if ( j == i-1 )
			temp3 = temp1;
	}
	temp2 = temp1->next;
	if( temp2 == NULL )
		temp3->next = NULL;
	else
		temp3->next = temp2;

	delete temp1;
	length--;
	
	return true;	
}

template<class T>
long MyList<T>::GetLength()
{
	return length;
}

template<class T>
bool MyList<T>::CopyToVector( T *array )
{
	myelement<T> *temp1, *temp;
	temp = start;

	long i = 0;

	while(1)
	{
		if( init )
		{
			i++;

			if( temp == NULL && i==length+1 )
			{
				return true;
			}
			else if( temp !=NULL && i<=length)
			{
				temp1 = temp->next;
				array[i-1] = temp->elem;
				temp = temp1;
			}
			else
				return false;

		}
		else
			return true;
	}

}


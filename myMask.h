#include <iostream>
using namespace std;
	static const int COL_NUM = 640;//number of y in frame
	static const int ROW_NUM = 480;//number of x in frame
	static  int VERTEX_NUM = COL_NUM * ROW_NUM;
	static const int COORDS_PER_VERTEX = 3;
	float VERTEX_GAP=0.0143;//_B must be small enough in real case

	float* rowValues = new float[ROW_NUM];
	float* colValues = new float[COL_NUM];
	float* vertexCoords = new float[VERTEX_NUM * COORDS_PER_VERTEX];
	void maskinitial(float xCenter, float yCenter)
	{
		
		colValues[0] = xCenter - (float)((COL_NUM-1)/2.0 * VERTEX_GAP);  // 0- 479/2*0.5
    	for(int i=1 ; i<COL_NUM ; i++)
    		colValues[i] = colValues[i-1] + VERTEX_GAP; 	
    	rowValues[0] = yCenter + (float)((ROW_NUM-1)/2.0 * VERTEX_GAP);
    	for(int i=1 ; i<ROW_NUM ; i++)
    		rowValues[i] = rowValues[i-1] - VERTEX_GAP;

		//********************************************
		
    	int rowIdx=0, colIdx=0, vertexIdx=0; 	
		for(vertexIdx=0 ; vertexIdx<VERTEX_NUM ; vertexIdx++)
    	{
    		rowIdx = vertexIdx / COL_NUM;
    		colIdx = vertexIdx % COL_NUM;
    		
    		//init vertexCoords
    		vertexCoords[vertexIdx * COORDS_PER_VERTEX] = colValues[colIdx];//x
    		vertexCoords[vertexIdx * COORDS_PER_VERTEX + 1] = rowValues[rowIdx];//y
    		vertexCoords[vertexIdx * COORDS_PER_VERTEX + 2] = -100;//z, set by setZValues later
    	}//end: trace each vertex
		
		
	}
	void setDepthValues(float *depthValues)
    {
    	//to move the current position of FloatBuffer, store the x and y float values to the tmp float array 
    	float* tmp = new float[2];// 	
    	for(int vertexIdx=0 ; vertexIdx<	VERTEX_NUM ; vertexIdx++)
    	{
			if (depthValues[vertexIdx]>2200) //1400
				vertexCoords[vertexIdx * COORDS_PER_VERTEX + 2] = -100.0;
			else
				vertexCoords[vertexIdx * COORDS_PER_VERTEX + 2] =  -1.0;
			
		}
		
    	for(int vertexIdx=VERTEX_NUM-1 ; vertexIdx>=0 ; vertexIdx--)
    	{ 		
			if ( vertexIdx * COORDS_PER_VERTEX - 3*300 + 2 >0 )
				;
	}
		
    }

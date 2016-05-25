#define EPSILON 0.00001

__kernel void mollerAndata(int dim, 
							__global float *v1, 
							__global float *v2, 
							__global float *v3, 
							__global float *directions, 
							__global float *results, 
							__global float *new_dir, 
							__global float *cameraPosition, 
							int numDirections, 
							int baseline,
							__global float * sources)	
{
	int j = get_global_id(0);	// direzione
	if (j%3==0){
		
		//float source[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
		
		float sourceA = FLT_MAX;
		float sourceB = FLT_MAX;
		float sourceC = FLT_MAX;
		float source[3] ;
		if(j < numDirections * 3){
			source[0] = cameraPosition[0];
			source[1] = cameraPosition[1] + baseline;
			source[2] = cameraPosition[2];
		}	
		else{
			source[0] = cameraPosition[0];
			source[1] = cameraPosition[1] - baseline ;
			source[2] = cameraPosition[2];
		}
			
		float t_max = FLT_MAX;
		float point[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
		
		for (int i = 0; i < dim * 3 ; i = i + 3) {
			float vertex1[3] = { v1[i], v1[i + 1], v1[i + 2] };
			float vertex2[3] = { v2[i], v2[i + 1], v2[i + 2] };
			float vertex3[3] = { v3[i], v3[i + 1], v3[i + 2] };

			float det, inv_det, u, v;
			float t;

			//Find vectors for two edges sharing V1
			float e1[3] = {vertex2[0] - vertex1[0], vertex2[1] - vertex1[1], vertex2[2] - vertex1[2] };
			float e2[3] = {vertex3[0] - vertex1[0], vertex3[1] - vertex1[1], vertex3[2] - vertex1[2] };

			//Begin calculating determinant - also used to calculate u parameter
			float P[3] = { 	directions[j + 1] * e2[2] - directions[j + 2] * e2[1],
							directions[j + 2] * e2[0] - directions[j + 0] * e2[2],
							directions[j + 0] * e2[1] - directions[j + 1] * e2[0] };

			det = e1[0] * P[0] + e1[1] * P[1] + e1[2] * P[2];

			if (!(det > -EPSILON && det < EPSILON)) {
				inv_det = 1.f / det;

				float T[3] ={source[0] - vertex1[0], source[1] - vertex1[1], source[2] - vertex1[2] };

				u = T[0] * P[0] + T[1] * P[1] + T[2] * P[2];
				u = u*inv_det;
				if (!(u < 0.f || u > 1.f)) {
					
					float Q[3] = {	T[1] * e1[2] - T[2] * e1[1],
									T[2] * e1[0] - T[0] * e1[2],
									T[0] * e1[1] - T[1] * e1[0] };

					v = directions[j] * Q[0] + directions[j + 1] * Q[1] + directions[j + 2] * Q[2];
					v = v*inv_det;

					if (!(v < 0.f || u + v  > 1.f)){

						t = e2[0] * Q[0] + e2[1] * Q[1] + e2[2] * Q[2];
						t = t * inv_det;

						if (t > EPSILON && t<t_max) { //ray intersection
							point[0] = source[0] + t*directions[j] ;
							point[1] = source[1] + t*directions[j + 1] ;
							point[2] = source[2] + t*directions[j + 2] ;
							t_max = t;
						}
					
					}	
				}
			}
		}
		results[j] = point[0];
		results[j+1] = point[1];
		results[j+2] = point[2];
		
		if (point[0] == FLT_MAX) {
			new_dir[j] = FLT_MAX;
			new_dir[j + 1] = FLT_MAX;
			new_dir[j + 2] = FLT_MAX;
		}
		else {
			float direct[3];
			direct[0] = results[j] - cameraPosition[0];
			direct[1] = results[j + 1] - cameraPosition[1];
			direct[2] = results[j + 2] - cameraPosition[2];
			float denom = sqrt(pow(direct[0], 2) + pow(direct[1], 2) + pow(direct[1], 2));
			// direct = direct / denom // norm(direct);
			direct[0] = direct[0] /denom;
			direct[1] = direct[1] /denom;
			direct[2] = direct[2] /denom;
			new_dir[j] = direct[0];
			new_dir[j + 1] = direct[1];
			new_dir[j + 2] = direct[2];
		}
		
		sources[j] = baseline;
		sources[j+1] = baseline; 
		sources[j+2] = baseline	;
	}	
}
	

__kernel void mollerRitorno(int dim, __global float *v1, __global float *v2, __global float *v3, __global float *source, __global float *directions, __global float *results) {
	int j = get_global_id(0);	// direzione
	
	if (j%3==0){
		float t_max = FLT_MAX;
		float pointFLOAT[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
		float pointFinal[3] = {0,0,0};
		bool found = false;
	
		for (int i = 0; i < dim*3 ; i = i + 3) {
			if( results[i] == FLT_MAX) break;
			float vertex1[3] = { v1[i], v1[i + 1], v1[i + 2] };
			float vertex2[3] = { v2[i], v2[i + 1], v2[i + 2] };
			float vertex3[3] = { v3[i], v3[i + 1], v3[i + 2] };

			float det, inv_det, u, v;
			float t;

			//Find vectors for two edges sharing V1
			float e1[3] = {vertex2[0] - vertex1[0], vertex2[1] - vertex1[1], vertex2[2] - vertex1[2] };
			float e2[3] = {vertex3[0] - vertex1[0], vertex3[1] - vertex1[1], vertex3[2] - vertex1[2] };

			//Begin calculating determinant - also used to calculate u parameter
			float P[3] = { 	directions[j + 1] * e2[2] - directions[j + 2] * e2[1],
							directions[j + 2] * e2[0] - directions[j + 0] * e2[2],
							directions[j + 0] * e2[1] - directions[j + 1] * e2[0] };

			det = e1[0] * P[0] + e1[1] * P[1] + e1[2] * P[2];

			if (!(det > -EPSILON && det < EPSILON)) {
				inv_det = 1.f / det;

				float T[3] ={source[0] - vertex1[0], source[1] - vertex1[1], source[2] - vertex1[2] };

				u = T[0] * P[0] + T[1] * P[1] + T[2] * P[2];
				u = u*inv_det;
				
				if (!(u < 0.f || u > 1.f)) {
					
					float Q[3] = {	T[1] * e1[2] - T[2] * e1[1],
									T[2] * e1[0] - T[0] * e1[2],
									T[0] * e1[1] - T[1] * e1[0] };

					v = directions[j] * Q[0] + directions[j + 1] * Q[1] + directions[j + 2] * Q[2];
					v = v*inv_det;

					if (!(v < 0.f || u + v  > 1.f)){

						t = e2[0] * Q[0] + e2[1] * Q[1] + e2[2] * Q[2];
						t = t * inv_det;

						if (t > EPSILON ) { //ray intersection
							pointFinal[2] = source[2] + t*directions[j + 2] ;
							if (pointFinal[2]>results[j+2]){
									found = true;
									break;	// a lui interessa trovarne uno, non tutti
							}
						}
					}
				}
			}
		}
		if(found){
			results[j]	= pointFLOAT[0];
			results[j+1] = pointFLOAT[1];
			results[j+2] = pointFLOAT[2];
		}
		else{
			results[j]	= results[j] - source[0] ;
			results[j+1] = results[j+1] - source[1];
			results[j+2] = results[j+2] - source[2];
		}
	}
}	
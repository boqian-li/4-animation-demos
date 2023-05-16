using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.0025f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

    Vector3 Gravity = new Vector3(0.0f, -9.8f, 0.0f);
    float uN = 0.5f;
    float uT = 0.5f;

    int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}

        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int i = 0; i < tet_number; i ++)
        {
            inv_Dm[i] = Build_Edge_Matrix(i).inverse;
        }

    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        ret[0, 0] = X[Tet[tet * 4]].x - X[Tet[tet * 4 + 1]].x;
        ret[1, 0] = X[Tet[tet * 4]].y - X[Tet[tet * 4 + 1]].y;
        ret[2, 0] = X[Tet[tet * 4]].z - X[Tet[tet * 4 + 1]].z;

        ret[0, 1] = X[Tet[tet * 4]].x - X[Tet[tet * 4 + 2]].x;
        ret[1, 1] = X[Tet[tet * 4]].y - X[Tet[tet * 4 + 2]].y;
        ret[2, 1] = X[Tet[tet * 4]].z - X[Tet[tet * 4 + 2]].z;

        ret[0, 2] = X[Tet[tet * 4]].x - X[Tet[tet * 4 + 3]].x;
        ret[1, 2] = X[Tet[tet * 4]].y - X[Tet[tet * 4 + 3]].y;
        ret[2, 2] = X[Tet[tet * 4]].z - X[Tet[tet * 4 + 3]].z;

        ret[3, 3] = 1.0f;

        return ret;
    }


    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
            //TODO: Add gravity to Force.
            Force[i] = Gravity * mass;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
            //TODO: Deformation Gradient
            Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];

            //TODO: Green Strain
            Matrix4x4 G = matrixMutiplyfloat(matrixSubmatrix(F.transpose * F, Matrix4x4.identity), 0.5f);

            //TODO: Second PK Stress
            Matrix4x4 S = matrixAddmatrix(matrixMutiplyfloat(G, 2.0f * stiffness_1),
                matrixMutiplyfloat(Matrix4x4.identity, stiffness_0 * trace(G)));
            Matrix4x4 P = F * S;

            /** alternative way by SVD
             
            **/

            //TODO: Elastic Force
            Matrix4x4 forces = matrixMutiplyfloat(P * inv_Dm[tet].transpose, -1.0f / (6.0f * inv_Dm[tet].determinant));

            Vector3 tmp = new Vector3(0.0f, 0.0f, 0.0f);
            for (int i = 1; i < 4; i ++)
            {
                Force[Tet[tet * 4 + i]] += new Vector3(forces[0, i-1], forces[1, i-1], forces[2, i-1]);
                tmp -= new Vector3(forces[0, i - 1], forces[1, i - 1], forces[2, i - 1]);
            }
            Force[Tet[4 * tet]] += tmp;
    	}

        Vector3 floorpos = new Vector3(0, -3.0f, 0);
        Vector3 floorNormal = new Vector3(0, 1.0f, 0);

    	for(int i=0; i<number; i++)
    	{
            //TODO: Update X and V here.
            V[i] += dt * Force[i] / mass;
            V[i] *= damp;
            X[i] += V[i] * dt;

            //TODO: (Particle) collision with floor.
            float sign = Vector3.Dot(X[i] - floorpos, floorNormal);
            if (sign <0 && Vector3.Dot(V[i], floorNormal) < 0)
            {
                X[i] -= sign * floorNormal;
                Vector3 vN = Vector3.Dot(V[i], floorNormal) * floorNormal;
                Vector3 vT = V[i] - vN;

                float a = Math.Max(1 - uT * (1 + uN) * vN.magnitude / vT.magnitude, 0);
                V[i] = -uN * vN + a * vT;
            }    

    	}

        //Laplacian Smoothing.
        for (int i = 0; i< number; i ++)
        {
            V_num[i] = 0;
            V_sum[i] = new Vector3(0.0f, 0.0f, 0.0f);
        }

        for (int tet = 0; tet < tet_number; tet ++)
        {
            V_sum[Tet[4 * tet]] += V[Tet[4 * tet + 1]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet + 3]];
            V_num[Tet[4 * tet]] += 3;
            V_sum[Tet[4 * tet + 1]] += V[Tet[4 * tet]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet + 3]];
            V_num[Tet[4 * tet + 1]] += 3;
            V_sum[Tet[4 * tet + 2]] += V[Tet[4 * tet + 1]] + V[Tet[4 * tet]] + V[Tet[4 * tet + 3]];
            V_num[Tet[4 * tet + 2]] += 3;
            V_sum[Tet[4 * tet + 3]] += V[Tet[4 * tet + 1]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet]];
            V_num[Tet[4 * tet + 3]] += 3;
        }

        for (int i = 0; i < number; i++)
        {
            V[i] = V_sum[i] / V_num[i] * 0.4f + V[i] * 0.6f;
        }
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }

    Matrix4x4 matrixSubmatrix(Matrix4x4 a, Matrix4x4 b)
    {
        //Matrix4x4 sub = new Matrix4x4();

        for (int i = 0; i < 4; i ++)
            for (int j = 0; j < 4; j ++)
            {
                a[i, j] = a[i, j] - b[i, j];
            }

        return a;
    }

    Matrix4x4 matrixAddmatrix(Matrix4x4 a, Matrix4x4 b)
    {
        //Matrix4x4 sub = new Matrix4x4();

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                a[i, j] = a[i, j] + b[i, j];
            }

        return a;
    }
    
    float trace(Matrix4x4 a)
    {
        float k = a[0, 0] + a[1, 1] + a[2, 2] + a[3, 3];
        return k;
    }

    Matrix4x4 matrixMutiplyfloat(Matrix4x4 a, float b)
    {
        //Matrix4x4 c = new Matrix4x4();

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
            {
                a[i, j] = a[i, j] * b;
            }
        return a;
    }
}

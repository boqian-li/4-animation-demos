using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity

	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia
	
	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;			
	
	float restitution 	= 0.5f;                 // for collision 弹性系数
	float friction = 0.2f;                  // 摩擦系数

	Vector3 G = new Vector3(0.0f, -9.8f, 0.0f);		//重力加速度


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	private Quaternion Add(Quaternion a, Quaternion b)
    {
		Quaternion c = new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);

		return c;
    }

	private Matrix4x4 Matrix_subtraction(Matrix4x4 a, Matrix4x4 b)
    {
		for (int i = 0; i < 4; i ++)
			for (int j = 0; j < 4; j++)
            {
				a[i, j] -= b[i, j];
            }

		return a;
    }

	private Matrix4x4 Matrix_multiply(Matrix4x4 a, float b)
	{
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				a[i, j] *= b;
			}
		}
		return a;
	}



	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		int collisionNum = 0;
		Vector3 r_collided = new Vector3(0, 0, 0);

		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		Quaternion q = transform.rotation;
		Vector3 x = transform.position;

		//1. 迭代检测每个点是否有碰撞，筛选出速度向内的点, 并累加其半径向量和数量
		for (int i = 0; i < vertices.Length; i++)
        {
			Vector3 x_i = x + q * vertices[i];
			bool condition_0 = Vector3.Dot((x_i - P), N) < 0;

			Vector3 v_i = v + Vector3.Cross(w, q * vertices[i]);
			bool condition_1 = Vector3.Dot(v_i, N) < 0;

			if (condition_0 && condition_1)
            {
				collisionNum += 1;
				r_collided += vertices[i];
            }
			
        }

		//2. 计算“平均碰撞点”，并通过弹性系数和摩擦系数计算这个点的前后速度
		if (collisionNum == 0) return;

		r_collided /= collisionNum;
		Vector3 v_cld = v + Vector3.Cross(w, q * r_collided);
		Vector3 v_N = Vector3.Dot(v_cld, N) * N;
		Vector3 v_T = v_cld - v_N;
		float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
		Vector3 v_N_new = -1.0f * restitution * v_N;
		Vector3 v_T_new = a * v_T;
		Vector3 v_cld_new = v_N_new + v_T_new;

		//3. 根据“平均碰撞点”的前后速度与冲量j的关系式，计算出此次碰撞受到的冲量j
		Matrix4x4 R = Matrix4x4.Rotate(q);
		Matrix4x4 I_now = R * I_ref * Matrix4x4.Transpose(R);

		Matrix4x4 K = Matrix_subtraction(
			Matrix_multiply(Matrix4x4.identity, 1.0f / mass), Get_Cross_Matrix(q * r_collided) * I_now.inverse * Get_Cross_Matrix(q * r_collided));

		Vector3 j = K.inverse.MultiplyVector(v_cld_new - v_cld);

		//4. 根据求得的冲量j，更新v&w
		v += 1.0f / mass * j;
		w += I_now.inverse.MultiplyVector(Vector3.Cross(q * r_collided, j));
	}



	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3 (0, 1, 1);
			launched=true;
		}

		if (launched)
        {
			// Part I: Update velocities 根据重力和空气阻力两个因素改变v & w
			v += dt * G;
			v *= linear_decay;
			w *= angular_decay;

			// Part II: Collision Impulse 根据碰撞改变v & w
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x_0 = transform.position;
			Vector3 x_1 = x_0 + dt * v;

			//Update angular status
			Quaternion q_0 = transform.rotation;
			Quaternion q_t = new Quaternion(
				w.x * 0.5f * dt,
				w.y * 0.5f * dt, 
				w.z * 0.5f * dt,
				0.0f);
			Quaternion q_1 = Add(q_0, q_t * q_0);

			// Part IV: Assign to the object
			transform.position = x_1;
			transform.rotation = q_1;

		}
	}
}

using UnityEngine;
using System.Collections;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0.0f;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float[,] new_h)
	{
		for (int i = 0; i < size; i ++)
			for (int j = 0; j <size; j ++)
            {
				low_h[i, j] = 99999;
			}

		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
            {
				new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);

				if (i != 0) new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
				if (j != 0) new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
				if (i != size - 1) new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
				if (j != size - 1) new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);

			}


		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		GameObject block1 = GameObject.Find("Block");
		Bounds block1_bounds = block1.GetComponent<MeshFilter>().mesh.bounds;

		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X = mesh.vertices;

		Vector3 block1_pos = block1.transform.position;
		int block_li = (int)(block1_pos.x / 0.1f + 50.0f) - 4;
		int block_ui = (int)(block1_pos.x / 0.1f + 50.0f) + 4;
		int block_lj = (int)(block1_pos.z / 0.1f + 50.0f) - 4;
		int block_uj = (int)(block1_pos.z / 0.1f + 50.0f) + 4;

		for (int i = block_li; i <= block_ui; i++)
			for (int j = block_lj; j <= block_uj; j++)
				if (i >= 0 && j >=0 && i <size && j <size)
                {
					Vector3 ray_pos_0 = block1.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
					Vector3 ray_pos_1 = block1.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -9, X[i * size + j].z));

					Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
					float distance = 99999;
					block1_bounds.IntersectRay(ray, out distance);
					low_h[i, j] = -10 + distance;
				}

		//TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				if (low_h[i, j] > h[i, j])
				{
					b[i, j] = 0;
					cg_mask[i, j] = false;
					vh[i, j] = 0;
				}
				else
				{
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
			}


		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, Mathf.Max(0, block_li),Mathf.Min(block_ui, size-1), Mathf.Max(block_lj, 0), Mathf.Min(size-1, block_uj));

		//TODO: for block 2, calculate low_h.
		GameObject block2 = GameObject.Find("Cube");
		Bounds block2_bounds = block2.GetComponent<MeshFilter>().mesh.bounds;

		Vector3 block2_pos = block2.transform.position;
		int block_li_ = (int)(block2_pos.x / 0.1f + 50.0f) - 4;
		int block_ui_ = (int)(block2_pos.x / 0.1f + 50.0f) + 4;
		int block_lj_ = (int)(block2_pos.z / 0.1f + 50.0f) - 4;
		int block_uj_ = (int)(block2_pos.z / 0.1f + 50.0f) + 4;

		for (int i = block_li_; i <= block_ui_; i++)
			for (int j = block_lj_; j <= block_uj_; j++)
				if (i >= 0 && j >= 0 && i < size && j < size)
				{
					Vector3 ray_pos_0 = block2.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -10, X[i * size + j].z));
					Vector3 ray_pos_1 = block2.transform.InverseTransformPoint(new Vector3(X[i * size + j].x, -9, X[i * size + j].z));

					Ray ray = new Ray(ray_pos_0, ray_pos_1 - ray_pos_0);
					float distance = 99999;
					block2_bounds.IntersectRay(ray, out distance);
					low_h[i, j] = -10 + distance;
				}

		//TODO: then set up b and cg_mask for conjugate gradient.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				if (low_h[i, j] > h[i, j])
				{
					b[i, j] = 0;
					cg_mask[i, j] = false;
					vh[i, j] = 0;
				}
				else
				{
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
			}

		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, Mathf.Max(0, block_li_), Mathf.Min(block_ui_, size - 1), Mathf.Max(block_lj_, 0), Mathf.Min(size - 1, block_uj_));
		//TODO: Diminish vh.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				vh[i, j] *= gamma;
			}

		//TODO: Update new_h by vh.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				if (i > 0)
					new_h[i, j] += rate * (vh[i - 1, j] - vh[i, j]);
				if (i < size - 1)
					new_h[i, j] += rate * (vh[i + 1, j] - vh[i, j]);
				if (j > 0)
					new_h[i, j] += rate * (vh[i, j - 1] - vh[i, j]);
				if (j < size - 1)
					new_h[i, j] += rate * (vh[i, j + 1] - vh[i, j]);
			}

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
            {
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
            }


		//Step 4: Water->Block coupling.
		//More TODO here.
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size]; 
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i = 0; i < size; i ++)
			for (int j = 0; j < size; j ++)
			{
				h[i, j] = X[i * size + j].y;
			}

		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int k = 0;
			while (k <4)
            {
				int i = (int)(Random.Range(0.0f, 1.0f) * size);
				int j = (int)(Random.Range(0.0f, 1.0f) * size);

				if (i == 0 || j == 0) { i = 1; j = 1; }
				else if (i >= size - 1 || j >= size - 1) { i = size - 2; j = size - 2; }

				float r = 0.5f * Random.Range(1.0f, 2.0f);

				h[i, j] += r;
				for (int m = -1; m < 2; m++)
					for (int n = -1; n < 2; n++)
					{
						if (m == 0 && n == 0) continue;
						h[i + m, j + n] -= r / 8;
					}
				k++;
			}
		}

		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				X[i * size + j].y = h[i, j];
			}

		mesh.vertices = X;
		mesh.RecalculateNormals();
	}
}

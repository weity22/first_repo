using UnityEngine;
using System.Collections;


public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    float dt = 0.015f;
    Vector3 v = new Vector3(0, 0, 0);	// velocity
    Vector3 w = new Vector3(0, 0, 0);	// angular velocity

    float mass;									// mass
    Matrix4x4 I_ref;							// reference inertia

    float linear_decay = 0.999f;				// for velocity decay
    float angular_decay = 0.998f;
    float restitution = 0.8f;					// for collision
    float u_T = 0.8f;

    Matrix4x4 MultipyScalar(Matrix4x4 A, float b)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = A[i, j] * b;
            }
        }
        return result;
    }


    Matrix4x4 AddTwoMatrix(Matrix4x4 A, Matrix4x4 B)
    {
        Matrix4x4 result = Matrix4x4.zero;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = A[i, j] + B[i, j];
            }
        }
        return result;
    }

    Matrix4x4 MinusTwoMatrix(Matrix4x4 A, Matrix4x4 B)
    {
        return AddTwoMatrix(A, MultipyScalar(B, -1));
    }

    // Use this for initialization
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude;
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] collision_points = new int[vertices.Length];
        int collision_num = 0;


        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        Vector3 Rri = new Vector3(0, 0, 0);
        //判断碰撞：
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 xi = transform.position + R.MultiplyVector(vertices[i]);
            Rri = R.MultiplyVector(vertices[i]);
            if (Vector3.Dot(xi - P, N) < 0 && Vector3.Dot(v + Vector3.Cross(w, Rri),N)<0)
            {
                collision_points[collision_num] = i;
                collision_num++;
            }

        }

        if (collision_num == 0)
        {
            return;
        }
        //找到平均碰撞中心：
        Vector3 ri = new Vector3(0, 0, 0);
        for (int i = 0; i < collision_num; i++)
        {
            ri += vertices[collision_points[i]];
        }
        ri /= (float)collision_num;

        //找法向速度最大的点：
        /*Vector3 ri = new Vector3(0, 0, 0);
        Vector3 vnmax = new Vector3(0, 0, 0);
        Vector3 vni = new Vector3(0, 0, 0);
        for (int i=0;i< collision_num;i++)
        {
            Rri = R.MultiplyVector(vertices[collision_points[i]]);
            vni = Vector3.Dot(v + Vector3.Cross(w, Rri), N) * N;
            if (Vector3.Magnitude(vni) > Vector3.Magnitude(vnmax))
            {
                vnmax = vni;
                ri = vertices[collision_points[i]];
            }
        }*/

        //求碰撞时的速度：
        Vector3 vi = new Vector3(0, 0, 0);
        Vector3 vinew = new Vector3(0, 0, 0);
        Rri = R.MultiplyVector(ri);
        vi = v + Vector3.Cross(w, Rri);
        Vector3 vn = Vector3.Dot(vi, N) * N;
        Vector3 vt = vi - vn;
        //求出碰撞后的速度：
        float a = Mathf.Max(0, 1.0f - u_T * (1 + restitution) * Vector3.Magnitude(vn) / Vector3.Magnitude(vt));
        Debug.Log("a="+a);
        //float a = 1.0f;
        vinew = -1 * restitution * vn + a * vt;
        Debug.Log("vinew =" + vinew);
        //由碰撞点的速度变化情况反推冲量：
        Matrix4x4 I_inverse = Matrix4x4.Inverse(R * I_ref * Matrix4x4.Transpose(R));
        Matrix4x4 Rri_star = Get_Cross_Matrix(Rri);
        Matrix4x4 K = MinusTwoMatrix(MultipyScalar(Matrix4x4.identity, 1 / mass), Rri_star * I_inverse * Rri_star);
        Vector3 J = K.inverse.MultiplyVector(vinew - vi);
        Debug.Log("J=" + J/1000);
        //由冲量更新质心运动情况：
        Vector3 vnew = v + 1 / mass * J;
        Vector3 wnew = w + I_inverse.MultiplyVector(Vector3.Cross(J, Rri));
        v = vnew;
        w = wnew;
        Debug.Log("vinew2 =" + (v + Vector3.Cross(w, Rri)));
    }

    // Update is called once per frame
    void Update()
    {
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }
        if (Input.GetKey("l"))
        {
            v = new Vector3(3, 3, 0);
            launched = true;
        }

        // Part I: Update velocities
        if (launched == false)
        {
            v = new Vector3(0, 0, 0);
            w = new Vector3(0, 0, 0);
            return;
        }
        
        //float E1 = 0.5f * mass * Vector3.Dot(v, v) + mass * 9.8f * transform.position.y + 0.5f * Vector3.Dot(w, I.MultiplyVector(w));
        //E1 /= 1000;
        //Debug.Log("E1=" + E1);
        v = v - new Vector3(0, 9.8f * dt, 0);

        v *= linear_decay;
        w *= angular_decay;

        // Part II: Collision Impulse
        Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

        // Part III: Update position & orientation
        //Update linear status
        Vector3 x = transform.position;
        //Update angular status
        Quaternion q = transform.rotation;

        x += v * dt;
        Quaternion wq = new Quaternion(w.x, w.y, w.z, 0);
        Quaternion delta_q = wq * q;
        q.x += delta_q.x * 0.5f * dt;
        q.y += delta_q.y * 0.5f * dt;
        q.z += delta_q.z * 0.5f * dt;
        q.w += delta_q.w * 0.5f * dt;

        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;

        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        Matrix4x4 I = R * I_ref * Matrix4x4.Transpose(R);
        float E2 = 0.5f * mass * Vector3.Dot(v, v) + mass * 9.8f * transform.position.y + 0.5f * Vector3.Dot(w, I.MultiplyVector(w));
        E2 /= 1000;
        Debug.Log("E2=" + E2);



    }
}





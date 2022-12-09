using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MyMathTools;

public class MeshGenerator2 : MonoBehaviour
{
    delegate Vector3 ComputePositionDelegate(float kX, float kZ);

    // Start is called before the first frame update
    void Start()
    {
        MeshFilter mf = GetComponent<MeshFilter>();
        //mf.mesh = CreateQuadXZ(new Vector3(4,1,6));
        //mf.mesh = CreateStripXZ(new Vector3(4, 1, 6),40);
        /*mf.mesh = CreateNormalizedPlaneXZ(10,7,
            (float kX,float kZ)=> {
                return  new Vector3(Mathf.Lerp(-5, 5, kX), 0, Mathf.Lerp(-2, 3, kZ));
            }
            );
        */
        /*
        mf.mesh = CreateNormalizedPlaneXZ(200, 200,
            (float kX, float kZ) => {
                return new Vector3(
                    Mathf.Lerp(-5, 5, kX),
                    (1+Mathf.Sin(kX*2*Mathf.PI*4))*.5f* (1 + Mathf.Cos(kZ * 2 * Mathf.PI * 3)) * .5f, Mathf.Lerp(-5, 5, kZ)) ;
            }
            );
        */

        //TORUS
        int nTurns = 6;

        mf.mesh = CreateNormalizedPlaneXZ(20 * nTurns, 10,
            (float kX, float kZ) => {
                float R = 3, r = 1;
                float theta = nTurns * 2 * Mathf.PI * kX;
                float alpha = 2 * Mathf.PI * (1 - kZ);
                Vector3 OOmega = new Vector3(
                    R * Mathf.Cos(theta),
                    0,
                    R * Mathf.Sin(theta));

                return OOmega
                + OOmega.normalized * r * Mathf.Cos(alpha)
                + r * Mathf.Sin(alpha) * Vector3.up
                + nTurns * 2 * r * kX * Vector3.up;
            }
        );

        gameObject.AddComponent<MeshCollider>();
    }


    Mesh CreateQuadXZ(Vector3 halfSize)
    {
        Mesh mesh = new Mesh();
        mesh.name = "quadXZ";

        Vector3[] vertices = new Vector3[4];
        Vector3[] normals = new Vector3[vertices.Length];
        int[] triangles = new int[6];

        vertices[0] = new Vector3(-halfSize.x, 0, halfSize.z);
        vertices[1] = new Vector3(halfSize.x, 0, halfSize.z);
        vertices[2] = new Vector3(halfSize.x, 0, -halfSize.z);
        vertices[3] = new Vector3(-halfSize.x, 0, -halfSize.z);

        normals[0] = Random.onUnitSphere; //  Vector3.up; /// same as new Vector3(0,1,0)
        normals[1] = Random.onUnitSphere;/// same as new Vector3(0,1,0)
        normals[2] = Random.onUnitSphere; /// same as new Vector3(0,1,0)
        normals[3] = Random.onUnitSphere; /// same as new Vector3(0,1,0)

        triangles[0] = 0;
        triangles[1] = 1;
        triangles[2] = 2;

        triangles[3] = 0;
        triangles[4] = 2;
        triangles[5] = 3;

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;

        mesh.RecalculateBounds();

        return mesh;
    }


    Mesh CreateStripXZ(Vector3 halfSize, int nSegmentsX)
    {
        Mesh mesh = new Mesh();
        mesh.name = "stripXZ";

        Vector3[] vertices = new Vector3[2 * (nSegmentsX + 1)];
        Vector3[] normals = new Vector3[vertices.Length];
        Vector2[] uv = new Vector2[vertices.Length];
        int[] triangles = new int[nSegmentsX * 2 * 3];

        for (int i = 0; i < nSegmentsX + 1; i++)
        {
            float k = (float)i / nSegmentsX;

            Cylindrical cyl = new Cylindrical(2 + .25f * Mathf.Sin(8 * 2 * Mathf.PI * k), k * 2 * Mathf.PI, halfSize.z * 2);
            vertices[i] = CoordConvert.CylindricalToCartesian(cyl);

            cyl = new Cylindrical(2, k * 2 * Mathf.PI, 0);
            vertices[i + nSegmentsX + 1] = CoordConvert.CylindricalToCartesian(cyl);
            /*
            vertices[i] = Vector3.Lerp(new Vector3(-halfSize.x, 0, halfSize.z),
                                        new Vector3(halfSize.x, 0, halfSize.z),
                                        k);
            vertices[i + nSegmentsX + 1] = vertices[i] - Vector3.forward * halfSize.z * 2;
            */
            uv[i] = new Vector2(k, 1);
            uv[i + nSegmentsX + 1] = new Vector2(k, 0);
        }

        for (int i = 0; i < normals.Length; i++)
        {
            normals[i] = new Vector3(vertices[i].x, 0, vertices[i].z).normalized;
        }

        int index = 0;
        for (int i = 0; i < nSegmentsX; i++)
        {
            triangles[index++] = i;
            triangles[index++] = i + nSegmentsX + 1 + 1;
            triangles[index++] = i + nSegmentsX + 1;

            triangles[index++] = i;
            triangles[index++] = i + 1;
            triangles[index++] = i + nSegmentsX + 1 + 1;
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;
        mesh.uv = uv;

        mesh.RecalculateBounds();

        return mesh;
    }

    Mesh CreateNormalizedPlaneXZ(int nSegmentsX, int nSegmentsZ,
        ComputePositionDelegate posCompute = null)
    {
        Mesh mesh = new Mesh();
        mesh.name = "planeXZ";

        Vector3[] vertices = new Vector3[(nSegmentsX + 1) * (nSegmentsZ + 1)];
        Vector3[] normals = new Vector3[vertices.Length];
        Vector2[] uv = new Vector2[vertices.Length];
        int[] triangles = new int[nSegmentsX * nSegmentsZ * 2 * 3];

        int index = 0;
        for (int i = 0; i < nSegmentsZ + 1; i++)
        {
            float kZ = (float)i / nSegmentsZ;
            for (int j = 0; j < nSegmentsX + 1; j++)
            {
                float kX = (float)j / nSegmentsX;
                vertices[index] = posCompute != null ? posCompute(kX, kZ) : new Vector3(kX, kZ);
                uv[index++] = new Vector2(kX, kZ);
            }
        }

        index = 0;
        int indexOffset = 0;
        for (int i = 0; i < nSegmentsZ; i++)
        {
            for (int j = 0; j < nSegmentsX; j++)
            {
                triangles[index++] = indexOffset + j;
                triangles[index++] = indexOffset + j + nSegmentsX + 1;
                triangles[index++] = indexOffset + j + 1;

                triangles[index++] = indexOffset + j + 1;
                triangles[index++] = indexOffset + j + nSegmentsX + 1;
                triangles[index++] = indexOffset + j + nSegmentsX + 1 + 1;
            }
            indexOffset += nSegmentsX + 1;
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;
        mesh.uv = uv;

        mesh.RecalculateBounds();
        mesh.RecalculateNormals();

        return mesh;
    }
}
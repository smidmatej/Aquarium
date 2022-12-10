using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using MyMathTools;
using UnityEngine.Rendering;


public class MeshGenerator : MonoBehaviour
{

    public delegate Vector3 ComputePositionDelegate(float kx,float kz);

    [SerializeField] Texture2D m_HeightMap;

    [SerializeField] int nx = 10;

    [SerializeField] int nz = 10;

    [SerializeField] Vector2 hsize = new Vector2(10f, 10f);

    [SerializeField] float noisePower = 1;
    MeshFilter mf;
    // Start is called before the first frame update
    void Start()
    {
        mf = GetComponent<MeshFilter>();
        mf.mesh = CreateNormalizedPlaneXZ(nx, nz,
         (float kx, float kz) => { 
            return new Vector3(
                        Mathf.Lerp(-hsize.x, hsize.x,kx), 
                        (Mathf.PerlinNoise(kx, kz) - 0.5f) * noisePower,
                        Mathf.Lerp(-hsize.y, hsize.y, kz));
            } 
            );
        gameObject.AddComponent<MeshCollider>();
        

    }

    Vector3 TorusCoordinateMap(float kx, float kz, float R, float r) 
    {
        Vector3 i = new Vector3(1,0,0);
        Vector3 o = new Vector3(0,0,0);
        Spherical omega_spherical = CoordConvert.CartesianToSpherical(o + i*R);
        omega_spherical.theta += 2*Mathf.PI*kx;
        Vector3 omega = CoordConvert.SphericalToCartesian(omega_spherical);
        Vector3 P = omega + CoordConvert.SphericalToCartesian(new Spherical(r, 0, 2*Mathf.PI*kz));
        Vector3 oomega = omega-o;
        Vector3 omegap = P-omega;
        Vector3 op = oomega + omegap;
        Debug.Log(op);
        Debug.DrawRay(o, op, Color.blue);
        return op;
    }

    void Update() {

        //Debug.Log(op);
        /*
        Debug.DrawRay(o, oomega, Color.red);
        Debug.DrawRay(omega, omegap, Color.green);
        Debug.DrawRay(o, op, Color.blue);
        */
    }

    Mesh CreateTriangle ()
    {
        Mesh mesh = new Mesh();
        mesh.name = "triangle";
        Vector3[] vertices = new Vector3[3];
        int[] triangles = new int[3];

        vertices[0] = new Vector3(1, 0, 0);
        vertices[1] = new Vector3(0, 1, 0);
        vertices[2] = new Vector3(0, 0, 1);

        triangles[0] = 0;
        triangles[1] = 1;
        triangles[2] = 2;

        mesh.vertices = vertices;
        mesh.triangles = triangles;

        mesh.RecalculateBounds();

        return mesh;
    }
    Mesh CreateQuad(Vector3 halfSize, Vector3 Position)
    {
        Mesh mesh = new Mesh();
        mesh.name = "quad_xz";
        Vector3[] vertices = new Vector3[4];
        Vector3[] normals = new Vector3[vertices.Length];
        int[] triangles = new int[6];

        vertices[0] = new Vector3(0, 0, 0) + Position;
        vertices[1] = new Vector3(halfSize.x, 0, 0) + Position;
        vertices[2] = new Vector3(halfSize.x, 0, halfSize.z) + Position;
        vertices[3] = new Vector3(0, 0, halfSize.z) + Position;

        normals[0] = Random.onUnitSphere;
        normals[1] = Random.onUnitSphere;
        normals[2] = Random.onUnitSphere;
        normals[3] = Random.onUnitSphere;

        triangles[0] = 0;
        triangles[1] = 2;
        triangles[2] = 1;
        triangles[3] = 0;
        triangles[4] = 3;
        triangles[5] = 2;

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.normals = normals;

        mesh.RecalculateBounds();

        return mesh;
    }
    Mesh CreateMesh18(Vector3 halfSize, int number_of_quads)
    {
        Mesh final_mesh = new Mesh();
        final_mesh.name = "9quads_xz";
       
        List<Vector3> List_vertices = new List<Vector3>();
        List<Vector3> List_normals = new List<Vector3>();
        List<int> List_triangles = new List<int>();

        
        for (int i = 0; i < number_of_quads; i += 1)
        {   
            Vector3 position = new Vector3(halfSize.x/number_of_quads * i, 0, 0);
            Mesh mesh = CreateQuad(halfSize/number_of_quads, position);

            for (int j = 0; j < mesh.vertices.Length; j += 1)
                List_vertices.Add(mesh.vertices[j]);
           
            
            for (int j = 0; j < mesh.normals.Length; j += 1)
                List_normals.Add(mesh.normals[j]);
            
          
            for (int j = 0; j < mesh.triangles.Length; j += 1)
                List_triangles.Add(mesh.triangles[j] + i*4); // +i to compensate number of current quad, each quad has 4 vertices
            
        }
        final_mesh.vertices = List_vertices.ToArray();
        for (int i = 0; i < final_mesh.vertices.Length; i++)
        {
            Debug.Log(final_mesh.vertices[i]);
        }
        
        final_mesh.normals = List_normals.ToArray();
        final_mesh.triangles = List_triangles.ToArray();
        for (int i = 0; i < final_mesh.triangles.Length; i++)
        {
            Debug.Log(final_mesh.triangles[i]);
        }

        final_mesh.RecalculateBounds();
        final_mesh.RecalculateNormals();

        return final_mesh;
    }

    Mesh CreateStripXZ(Vector3 size, int nSegmentsX)
    {
        Mesh mesh = new Mesh();
        mesh.name = "xzMesh";

        Vector3[] vertices = new Vector3[2*(nSegmentsX+1)];
        Vector3[] normals = new Vector3[2*(nSegmentsX+1)];
        Vector2[] uv = new Vector2[2*(nSegmentsX+1)];
        int[] triangles = new int[nSegmentsX*2*3];

        Vector3 startPositionTop = new Vector3(-size.x/2, 0, size.z/2);
        Vector3 endPositionTop = new Vector3(size.x/2, 0, size.z/2);
        Vector3 startPositionBottom = new Vector3(-size.x/2, 0, -size.z/2);
        Vector3 endPositionBottom = new Vector3(size.x/2, 0, -size.z/2);
        for (int i = 0; i < nSegmentsX+1; i++)
        {
            float k= (float)i/(nSegmentsX);
            Cylindrical cyl = new Cylindrical(0, k*3*2*Mathf.PI, 0);
            vertices[i] = CoordConvert.CylindricalToCartesian(cyl);
            cyl = new Cylindrical(2+k*20, k*3*2*Mathf.PI, 4);
            vertices[i+nSegmentsX+1] = CoordConvert.CylindricalToCartesian(cyl);
            /*
            vertices[i] = Vector3.Lerp(startPositionTop, endPositionTop, k);
            vertices[i+nSegmentsX+1] = Vector3.Lerp(startPositionBottom, endPositionBottom, k);
            */
            uv[i] = new Vector2(k, 1);
            uv[i+nSegmentsX+1] = new Vector2(k, 0);
        }

        int index = 0;
        for (int i = 0; i < nSegmentsX; i++)
        {
            triangles[index++] = i;
            triangles[index++] = i+(nSegmentsX+1)+1;
            triangles[index++] = i+(nSegmentsX+1);

            triangles[index++] = i;
            triangles[index++] = i+1;
            triangles[index++] = i+(nSegmentsX+1)+1;
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv=uv;
        mesh.RecalculateNormals();
        return mesh;
    }
    public static Mesh CreateNormalizedPlaneXZ(int nSegmentsX, int nSegmentsZ, ComputePositionDelegate posCompute = null)
    {
        Mesh mesh = new Mesh();
        mesh.indexFormat = IndexFormat.UInt32;
        mesh.name = "xzPlane";

        Vector3[] vertices = new Vector3[(nSegmentsZ+1)*(nSegmentsX+1)];
        Vector3[] normals = new Vector3[(nSegmentsZ+1)*(nSegmentsX+1)];
        Vector2[] uv = new Vector2[(nSegmentsZ+1)*(nSegmentsX+1)];
        int[] triangles = new int[nSegmentsZ*nSegmentsX*2*3];

        Vector3 startPositionTop = new Vector3(0, 0, 1);
        Vector3 endPositionTop = new Vector3(1, 0, 1);
        Vector3 startPositionBottom = new Vector3(0, 0, 0);
        Vector3 endPositionBottom = new Vector3(1, 0, 0);
        for (int j = 0; j < nSegmentsZ+1; j++)
        {
            float kz= (float)j/(nSegmentsZ);

            for (int i = 0; i < nSegmentsX+1; i++)
            {
                float kx= (float)i/(nSegmentsX);

                //slide = Mathf.Lerp(slide,10.0f, Time.deltaTime*0.00001f);

                //float z = 100*Mathf.Sin(0.01f*Mathf.Pow((kx*size.x-size.x/2)*slide, 2)) + 100*Mathf.Cos(0.01f*Mathf.Pow((kz*size.z-size.z/2)*slide, 2)+Mathf.PI) + 0*Random.Range(0.1f, 30.0f);

                vertices[i + j* (nSegmentsX+1)] = posCompute!=null ? posCompute(kx,kz): new Vector3(kx,0,kz);
                uv[i + j* (nSegmentsX+1)] = new Vector2((float)i/nSegmentsX, (float)j/nSegmentsZ);
                //normals[i + j* (nSegmentsX+1)] = Vector3.up;
                //Debug.Log(vertices[i + j* (nSegmentsX+1)]);
            }
        }


        int index = 0;
        for (int j = 0; j < nSegmentsZ; j++)
        {
            for (int i = 0; i < nSegmentsX; i++)
            {
              
                triangles[index++] = j*(nSegmentsX+1) + i;
                triangles[index++] = (j+1)*(nSegmentsX+1) + i;
                triangles[index++] = (j+1)*(nSegmentsX+1) + i +1;

                triangles[index++] = j*(nSegmentsX+1) + i ;
                triangles[index++] = (j+1)*(nSegmentsX+1) + i +1;
                triangles[index++] = (j)*(nSegmentsX+1) + i + 1;

            }
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv=uv;
        mesh.RecalculateNormals();
        return mesh;
    }
}

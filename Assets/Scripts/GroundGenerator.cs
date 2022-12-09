using System.Collections;
using System.Collections.Generic;
using UnityEngine;


 
public class GroundGenerator : MonoBehaviour {
    public float power = 3.0f;
    public float scale = 1.0f;
    private Vector2 startPoint = new Vector2(0f, 0f);

    void Start () {
        MakeNoise ();
    }

    void MakeNoise() {
        MeshFilter mf = GetComponent<MeshFilter>();
        Vector3[] vertices = mf.mesh.vertices; 
        for (int i = 0; i < vertices.Length; i++) {    
        float x = startPoint.x + vertices[i].x  * scale;
        float z = startPoint.y + vertices[i].z  * scale; 
            vertices[i].y = (Mathf.PerlinNoise (x, z) - 0.5f) * power;  
        }
        mf.mesh.vertices = vertices; 
        mf.mesh.RecalculateBounds();
        mf.mesh.RecalculateNormals(); 
    }
} 


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utils : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public static List<Vector3> fibonacciSphere(int samples, Quaternion centerRotation)
    {
        List<Vector3> points = new List<Vector3>();
        float phi = Mathf.PI * (3.0f - Mathf.Sqrt(5.0f));  // golden angle in radians

  
        for(int i=0; i<samples; i++)
        {
            float y = 1 - (i / (float)(samples - 1));  // y goes from 1 to -1
            float radius = Mathf.Sqrt(1 - y * y); // radius at y

            float theta = phi * i; // golden angle increment

            float x = Mathf.Cos(theta) * radius;
            float z = Mathf.Sin(theta) * radius;

            points.Add(centerRotation*(new Vector3(x, y, z)));
        }

        return points;
    }

    
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utils : MonoBehaviour
{

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

    public static void SpawnObjects(int numberOfObjects, GameObject fish, Vector3 spawnCenter, float spawnRadius)
    {
        for(int i=0; i<numberOfObjects; i++)
        {
            Vector3 spawnPos = spawnCenter + Random.insideUnitSphere * spawnRadius;
            Vector3 spawnRot = new Vector3(0, Random.Range(0.0f, 360.0f), 0);
            Instantiate(fish, spawnPos, Quaternion.Euler(spawnRot));
        }
    }

    
}

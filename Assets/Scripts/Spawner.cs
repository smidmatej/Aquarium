using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour
{
    public GameObject fish;

    public int numberOfObjects = 10;
    public Vector3 spawnCenter = new Vector3(0, 0, 0);
    public float spawnRadius = 1f;

    // Start is called before the first frame update
    void Start()
    {
        Utils.SpawnObjects(numberOfObjects, fish, spawnCenter, spawnRadius);
       
    }


}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TreeSpawner : MonoBehaviour
{
    public GameObject tree;

    public int numberOfObjects = 10;
    public Vector3 spawnCenter = new Vector3(0, 0, 0);
    public float spawnRadius = 10f;
    [SerializeField] Texture2D Map;

    // Start is called before the first frame update
    void Start()
    {
        Utils.SpawnObjectsOnPlane(numberOfObjects, tree, spawnCenter, spawnRadius);

    }


}
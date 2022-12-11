using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using static UnityEngine.Rendering.HighDefinition.LocalVolumetricFogArtistParameters;

public class AquariumAssembler : MonoBehaviour
{
    [Header("Aquarium Size")]
    public Vector2 hsize = new Vector2(1, 1);
    public float wallHeight = 1;
    public float wallThickness = 0.5f;

    [Header("Fish Spawn")]
    public int NumberOfFish = 10;
    public int NumberOfTrees = 12;


    [Header("Ground and Water Detail")]
    public int nVertexesX = 10;
    public int nVertexesY = 10;
    public float groundNoisePower = 1;
    
    [Header("Prefabs")]
    public GameObject glassWallPrefab;
    public GameObject baseWallPrefab;
    public GameObject groundPrefab;
    public GameObject waterPrefab;
    public GameObject fishPrefab;
    public GameObject treePrefab;

    private GameObject parent;
    private float fogOverlap = 0.3f;

    
    // Start is called before the first frame update
    void Start()
    {
        int a = 1+1;
        int b = 1;
        parent = new GameObject("Aquarium");
        InstantiateWalls();
        InstantiateBase();
        InstantiateSand();
        InstantiateWaterSurfaceSurface();

        float spawnRadius = Mathf.Min(new float[] { hsize.x, hsize.y, wallHeight }) / 2;
        Utils.SpawnObjects(NumberOfFish, fishPrefab, new Vector3(0, wallHeight/2, 0), spawnRadius);

        Vector3 spawnCenter = new Vector3(0f, 0f, 0f);
        Utils.SpawnObjects(NumberOfTrees, treePrefab, spawnCenter, spawnRadius * 1.7f);

    }

    void InstantiateWater()
    {
        GameObject water = Instantiate(waterPrefab, new Vector3(0f, wallHeight/2f, 0f), Quaternion.Euler(0, 0, 0));
        water.GetComponent<LocalVolumetricFog>().parameters.size = new Vector3(2*hsize.x - fogOverlap, wallHeight*0.7f, 2*hsize.y - fogOverlap);
        water.transform.SetParent(parent.transform, false);
    }

    void InstantiateBase()
    {
        GameObject aquariumBase = new GameObject("Base");
        // Create the aquarium base
        Vector3 southPos = new Vector3(0, -wallHeight/2, -hsize.y - wallThickness/2);
        Vector3 northPos = new Vector3(0, -wallHeight/2, hsize.y + wallThickness/2);
        Vector3 eastPos = new Vector3(hsize.x + wallThickness/2, -wallHeight/2, 0);
        Vector3 westPos = new Vector3(-hsize.x - wallThickness/2, -wallHeight/2, 0);
        

        GameObject newObject;
        newObject = Instantiate(baseWallPrefab, southPos, Quaternion.Euler(0, 180, 0));
        newObject.transform.localScale = new Vector3(2*hsize.x + wallThickness, wallHeight, wallThickness/2);
        newObject.transform.SetParent(aquariumBase.transform, false);
        newObject = Instantiate(baseWallPrefab, northPos, Quaternion.Euler(0, 0, 0));
        newObject.transform.localScale = new Vector3(2*hsize.x + wallThickness, wallHeight, wallThickness/2);
        newObject.transform.SetParent(aquariumBase.transform, false);
        newObject = Instantiate(baseWallPrefab, eastPos, Quaternion.Euler(0, 90, 0));
        newObject.transform.localScale = new Vector3(2*hsize.y + wallThickness, wallHeight, wallThickness/2);
        newObject.transform.SetParent(aquariumBase.transform, false);
        newObject = Instantiate(baseWallPrefab, westPos, Quaternion.Euler(0, -90, 0));
        newObject.transform.localScale = new Vector3(2*hsize.y + wallThickness, wallHeight, wallThickness/2);
        newObject.transform.SetParent(aquariumBase.transform, false);

        aquariumBase.transform.SetParent(parent.transform, false);
    }



    void InstantiateSand()
    {
        GameObject sandDunes = new GameObject("SubstrateDunes");
        
        //sandDunes.transform.parent = transform;
        MeshFilter mf = sandDunes.AddComponent<MeshFilter>();
        
        mf.mesh = MeshGenerator.CreateNormalizedPlaneXZ(nVertexesX, nVertexesY,
         (float kx, float kz) => { 
            return new Vector3(
                        Mathf.Lerp(-hsize.x, hsize.x,kx), 
                        (Mathf.PerlinNoise(kx, kz) - 0.5f) * groundNoisePower,
                        Mathf.Lerp(-hsize.y, hsize.y, kz));
            } 
            );
        sandDunes.transform.position = new Vector3(0f, wallHeight/5f, 0f);
        sandDunes.AddComponent<MeshCollider>();
        sandDunes.GetComponent<MeshCollider>().convex = true;
        sandDunes.AddComponent<MeshRenderer>();
        Material myMat = Resources.Load("Materials/SubstrateDunes", typeof(Material)) as Material;
        
        sandDunes.GetComponent<MeshRenderer>().material = myMat;

        sandDunes.transform.SetParent(parent.transform, false);
    }

    void InstantiateWaterSurfaceSurface()
    {
        GameObject waterSurface = new GameObject("WaterSurface");
        
        //sandDunes.transform.parent = transform;
        MeshFilter mf = waterSurface.AddComponent<MeshFilter>();
        
        mf.mesh = MeshGenerator.CreateNormalizedPlaneXZ(nVertexesX, nVertexesY,
         (float kx, float kz) => { 
            return new Vector3(
                        Mathf.Lerp(-hsize.x, hsize.x,kx), 
                        0,
                        Mathf.Lerp(-hsize.y, hsize.y, kz));
            } 
            );
        waterSurface.transform.position = new Vector3(0f, wallHeight*0.9f, 0f);
        waterSurface.AddComponent<MeshCollider>();
        waterSurface.GetComponent<MeshCollider>().convex = true;
        waterSurface.AddComponent<MeshRenderer>();
        Material myMat = Resources.Load("Materials/WaterSurface", typeof(Material)) as Material;
        
        waterSurface.GetComponent<MeshRenderer>().material = myMat;
        // Watersurface does not cast shadows, because I dont know how to make its shadows look good
        waterSurface.GetComponent<MeshRenderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        waterSurface.transform.SetParent(parent.transform, false);
    }

    void InstantiateWalls()
    {
        // Create the walls
        Vector3 southPos = new Vector3(0, wallHeight/2, -hsize.y);
        Vector3 northPos = new Vector3(0, wallHeight/2, hsize.y);
        Vector3 eastPos = new Vector3(hsize.x, wallHeight/2, 0);
        Vector3 westPos = new Vector3(-hsize.x, wallHeight/2, 0);
        Vector3 groundPos = new Vector3(0, 0, 0);

        GameObject newObject;
        newObject = Instantiate(glassWallPrefab, southPos, Quaternion.Euler(0, 0, 0));
        newObject.transform.localScale = new Vector3(2*hsize.x, wallHeight, 1);
        newObject.transform.SetParent(parent.transform, false);
        newObject = Instantiate(glassWallPrefab, northPos, Quaternion.Euler(0, 0, 0));
        newObject.transform.localScale = new Vector3(2*hsize.x, wallHeight, 1);
        newObject.transform.SetParent(parent.transform, false);
        newObject = Instantiate(glassWallPrefab, eastPos, Quaternion.Euler(0, 90, 0));
        newObject.transform.localScale = new Vector3(2*hsize.y, wallHeight, 1);
        newObject.transform.SetParent(parent.transform, false);
        newObject = Instantiate(glassWallPrefab, westPos, Quaternion.Euler(0, 90, 0));
        newObject.transform.localScale = new Vector3(2*hsize.y, wallHeight, 1);
        newObject.transform.SetParent(parent.transform, false);

        


        // Create the ground
        newObject = Instantiate(groundPrefab, groundPos, Quaternion.Euler(90, 0, 0));
        newObject.transform.localScale = new Vector3(2*hsize.x, 2*hsize.y, 1);
        newObject.transform.SetParent(parent.transform, false);



    }

}

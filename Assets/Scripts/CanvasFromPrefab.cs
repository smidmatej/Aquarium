using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class CanvasFromPrefab : MonoBehaviour
{

    [SerializeField] public GameObject CanvasPrefab;
    // Start is called before the first frame update
    void Start()
    {
        Instantiate(CanvasPrefab, new Vector3(0, 0, 0), Quaternion.Euler(0, 0, 0));
        Destroy(transform.gameObject);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}

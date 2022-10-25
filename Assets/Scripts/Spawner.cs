using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour
{
    [SerializeField] public GameObject fish;

    [SerializeField] public int number_of_objects;
    // Start is called before the first frame update
    void Start()
    {
        for(int i=0; i<number_of_objects; i++)
        {

            Instantiate(fish, new Vector3(Random.Range(-10.0f, 10.0f), 
                            Random.Range(-10.0f, 10.0f), 
                            Random.Range(-10.0f, 10.0f)), 
                            Quaternion.Euler(Random.Range(0.0f, 360.0f), Random.Range(0.0f, 360.0f), Random.Range(0.0f, 360.0f)));
        }
       
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}

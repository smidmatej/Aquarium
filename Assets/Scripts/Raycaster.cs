using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Raycaster : MonoBehaviour
{

    Vector3 headPosition = new Vector3(0,0,0);
    Quaternion lookOrientationLocal; 
    List<Vector3> pointsOnASphere;
    float rayLength = 5f;
    int numberOfPoints = 100;
    bool headingForCollision = false;
    Vector3 avoidanceDirection;
    Vector3 avoidanceForce;
    float maxSpeed = 1f;
    Vector3 velocity;
    Vector3 acceleration;
    float accelerationFactor = 0.001f;
    float maxSteerForce = 1000f;
    float mySize = 2f;



    // bitshift to fish layer
    int fishLayerMask;


    Collider m_Collider;

    
    // Start is called before the first frame update
    void Start()
    {
        velocity = this.transform.forward.normalized * maxSpeed;
        fishLayerMask = (1 << LayerMask.NameToLayer("Fish"));
        // This casts rays only against colliders in layer 3.
        // But to collide against everything except layer 3, use the ~ operator because it inverts a bitmask.
        fishLayerMask = ~fishLayerMask;
        //Fetch the Collider from the GameObject
        m_Collider = GetComponent<Collider>();

        // Extents are the half-widths of the box.
        // I only care about the thickness
        mySize = m_Collider.bounds.extents.x;

        lookOrientationLocal = Quaternion.FromToRotation(Vector3.up, Vector3.forward);
       
    }


    // Update is called once per frame
    void FixedUpdate()
    {
        acceleration = Vector3.zero;
        avoidanceForce = Vector3.zero;
        avoidanceDirection = new Vector3(-1,0,0);
        
        pointsOnASphere = fibonacciSphere(numberOfPoints, lookOrientationLocal);
        displayCollisionRays();
        if(Time.frameCount%10==0)
        {
        headingForCollision = isHeadingForColision();
        if(headingForCollision){
            Debug.Log("Heading for collision");
            avoidanceDirection = noObstacleDirection();
            Debug.Log("Avoidance direction: " + avoidanceDirection);
            avoidanceForce = SteeringForce(avoidanceDirection);
            acceleration += avoidanceForce;
            //Debug.Log("avoidanceForce: " + avoidanceForce);
            Debug.DrawRay(this.transform.position, avoidanceForce, Color.cyan);
            //velocity = avoidanceForce.normalized*maxSpeed;
        }
        else
        {
            //velocity = velocity;
            //Debug.Log("Not heading for collision");
        }
        /*
        if(acceleration == Vector3.zero)
        {
            acceleration = this.transform.forward.normalized * accelerationFactor;
        }
        */
        }
        Debug.DrawRay(this.transform.position, avoidanceForce, Color.cyan);
        Debug.DrawRay(this.transform.position, velocity, Color.green);
        Debug.Log("Acceleration: " + acceleration);
        velocity += acceleration * Time.fixedDeltaTime;
        
        velocity = Vector3.ClampMagnitude(velocity, maxSpeed);

        this.transform.position += velocity * Time.fixedDeltaTime;
        Vector3 dir = velocity.normalized;
        this.transform.rotation = Quaternion.LookRotation(dir);
    }

    // This is basically a P controller
    Vector3 SteeringForce (Vector3 desiredDirection) {
        Vector3 v = desiredDirection.normalized - velocity.normalized;
        v = v * maxSteerForce;
        //Vector3 v = desiredDirection.normalized * maxSpeed;
        return Vector3.ClampMagnitude (v, maxSteerForce);
    }


    void displayCollisionRays()
    {
        foreach (Vector3 direction in pointsOnASphere)
        {

            Ray ray = new Ray(this.transform.position + this.transform.rotation*headPosition, this.transform.rotation*direction);
            RaycastHit hitInfo;

            // Cast a ray in ray direction and check for hit
            if(Physics.Raycast(ray, out hitInfo, rayLength, fishLayerMask))
            {
                //Debug.Log("Hit something");
                Debug.DrawRay(this.transform.position + this.transform.rotation*headPosition, rayLength*(this.transform.rotation*direction), Color.red);
            }
            else
            {
                Debug.DrawRay(this.transform.position + this.transform.rotation*headPosition, rayLength*(this.transform.rotation*direction), Color.green);
            }
        }
    }

    bool isHeadingForColision(){


        Ray ray = new Ray(this.transform.position + this.transform.rotation*headPosition, this.transform.forward);
        RaycastHit hitInfo;

        // Cast a ray in ray direction and check for hit
        if(Physics.SphereCast(ray, mySize, out hitInfo, rayLength, fishLayerMask))
        {
            //Debug.Log(hitInfo.distance);
            Debug.DrawRay(this.transform.position + this.transform.rotation*headPosition, hitInfo.distance*this.transform.forward, Color.red);
            return true;
        }
        
        return false;
    }

    Vector3 noObstacleDirection(){

        foreach (Vector3 direction in pointsOnASphere)
        {
            Ray ray = new Ray(this.transform.position + this.transform.rotation*headPosition, this.transform.rotation*direction);

            // Cast a ray in ray direction and check for hit
            // if there is no obstacle, return that direction
            // directions in pointsOnASphere are sorted by distance from the center of the hemisphere, 
            // so the first one is the closest to straight direction
            if(!Physics.SphereCast(ray, mySize, rayLength, fishLayerMask))
            {
                Debug.DrawRay(this.transform.position + this.transform.rotation*headPosition, rayLength*(this.transform.rotation*direction), Color.blue);
                return direction;
            }
        }
        return Vector3.zero;
    }





    List<Vector3> fibonacciSphere(int samples, Quaternion centerRotation)
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


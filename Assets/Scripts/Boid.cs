using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MyMathTools;

public class Boid : MonoBehaviour
{


    float swim_speed;
    float randomRotationVariance = 0.1f;


    float frequency = 1.0f;
    float damping = 1.0f;
    float searchRadius = 10.0f;
    
    float repelForceMultiplier = 1.0f;
    float attractForceMultiplier = 0.1f;

    float collisionDetectionRange = 10.0f;
    float evasionAvoidanceCoefficient = 1.0f;
    float spinAvoidanceCoefficient = 1.0f;

    float minSpeed = 1.0f;
    float maxSpeed = 10.0f;

    GameObject[] otherFish;
    GameObject[] nearFish = new GameObject[1]; 

    Vector3 noiseVelocityRandomWalk = new Vector3(0,0,0);



    //[SerializeField] int numberOfRays = 10;
    int numberOfRays = 5;

    
    Vector3 velocity = new Vector3(0,0,0);
    public FishSettings fishSettings;



    // Start is called before the first frame update
    void Start()
    {
        GameObject fishSettingsObject = GameObject.FindGameObjectWithTag("FishSettings");
        fishSettings = fishSettingsObject.GetComponent<FishSettings>();
        
        velocity = new Vector3(0,0,0);
        string my_species = "fish";

        otherFish = GameObject.FindGameObjectsWithTag(my_species);
        // I am a fish, but I am not other fish
        RemoveMyselfFromOtherFishArray();

        if(otherFish.Length == 0)
            nearFish = findNearFish();
        
    }

    
    void FixedUpdate() {

        // I only look for swarm mates in a radius around me
        if(otherFish.Length != 0)
            nearFish = findNearFish(); 

        

        // Applies torque to avoid collisions
        //ObstacleAvoidance();

        Vector3 separationForce = new Vector3(0,0,0);
        Vector3 cohesionForce = new Vector3(0,0,0);
        Vector3 swarmDirection = new Vector3(0,0,0);
        Vector3 swarmForce = new Vector3(0,0,0);

        if(nearFish.Length > 1)
        {   /*
            // I see other fish, I swarm
            // calculates a rotation that will be given as a reference to the PID controler
            //desiredRotation = CalculateSwarmAligment();


            swarmDirection = CalculateSwarmAligment();
            
            

            Vector3 offsetToFlockmatesCentre = (CenterOfFlockmates() - this.transform.position);
            // Applies a attract/repel force based on the swarm
            cohesionForce = CohesionForce();
            separationForce = SeparationForce(); 
            swarmForce = SteerTowards(swarmDirection) * fishSettings.swarmCoefficient;
            cohesionForce = SteerTowards(offsetToFlockmatesCentre) * fishSettings.cohesionCoefficient;
            separationForce = SteerTowards(separationForce) * fishSettings.separationCoefficient;
            separationForce = new Vector3(0,0,0);
            /*
            Debug.Log($"swarmDirection= {swarmDirection}");
            Debug.Log($"cohesionForce= {cohesionForce}");
    
            Debug.Log($"separationForce= {separationForce}");
            */
        }
        else
        {
            // I am all alone, keep my rotation
            //desiredRotation = m_rb.transform.rotation;
        }

        


        Vector3 acc = swarmDirection + cohesionForce + separationForce;
        Debug.Log($"acc ={acc}" );
        acc = new Vector3(0,0,0);
        velocity = velocity + acc*Time.fixedDeltaTime;
        Debug.Log($"Velocity ={velocity}" );
        float speed = velocity.magnitude;
        Vector3 dir = velocity / speed;
        speed = Mathf.Clamp (speed, fishSettings.minSpeed, fishSettings.maxSpeed);
        velocity = dir * speed;

        Debug.Log($"Velocity ={velocity}" );
        this.transform.position += velocity*Time.fixedDeltaTime;
        this.transform.forward = dir;
    }


    private void Update() {
        SetSettings();
    }

    
    void SetSettings()
    {   
        swim_speed = fishSettings.swim_speed;
        randomRotationVariance = fishSettings.randomRotationVariance;
        frequency = fishSettings.frequency;
        damping = fishSettings.damping;
        searchRadius = fishSettings.searchRadius;
        repelForceMultiplier = fishSettings.repelForceMultiplier;
        attractForceMultiplier = fishSettings.attractForceMultiplier;
        collisionDetectionRange = fishSettings.collisionDetectionRange;
        evasionAvoidanceCoefficient = fishSettings.evasionAvoidanceCoefficient;
        spinAvoidanceCoefficient = fishSettings.spinAvoidanceCoefficient;


    }

    void RemoveMyselfFromOtherFishArray()
    {
        // Utility function 
        List<GameObject> newOtherFish = new List<GameObject>();
        for (int i = 0; i < otherFish.Length; i++)
        {
            if(otherFish[i] != gameObject)
                newOtherFish.Add(otherFish[i]);
        }
        otherFish = newOtherFish.ToArray();
    }







    
    Vector3 SteerTowards (Vector3 vector) {
        Vector3 v = vector.normalized * fishSettings.maxSpeed - velocity;
        return Vector3.ClampMagnitude (v, fishSettings.maxSteerForce);
    }


    Vector3 AddNoiseToVector(Vector3 v)
    {
        v.x += NextGaussian(0, randomRotationVariance);
        v.y += NextGaussian(0, randomRotationVariance);
        v.z += NextGaussian(0, randomRotationVariance);
        return v;
    }
    Quaternion AddNoiseToRotation(Quaternion q)
    {
        q.w += NextGaussian(0, randomRotationVariance);
        q.x += NextGaussian(0, randomRotationVariance);
        q.y += NextGaussian(0, randomRotationVariance);
        q.z += NextGaussian(0, randomRotationVariance);
        return q;
    }

        Vector3 CalculateSwarmAligment()
    {
            float w=.0f,x=0.0f,y=0.0f,z=0.0f;
            // What are the other fish doing?
            Vector3 averageDirection = new Vector3(0,0,0);
            for (int i = 0; i < nearFish.Length; i++)
            {
                // Calculate mean rotation of all the other fish
                w+=nearFish[i].transform.rotation.w;
                x+=nearFish[i].transform.rotation.x;
                y+=nearFish[i].transform.rotation.y;
                z+=nearFish[i].transform.rotation.z;

                Vector3 direction = nearFish[i].transform.rotation*Vector3.forward;
                averageDirection += direction;
            }
            averageDirection /= nearFish.Length;
            Quaternion meanRotation = new Quaternion(w/otherFish.Length, x/otherFish.Length, y/otherFish.Length, z/otherFish.Length);
            return averageDirection;
    }


    void SwarmCohesionAndSeparation()
    {   
        Vector3 repelForce = new Vector3(0,0,0);
        Vector3 attactForce = new Vector3(0,0,0);
        // What are the other fish doing?
        for (int i = 0; i < nearFish.Length; i++)
        {
                // Each fish looks at all the other fish each time.fixedDeltaTime -> O(n^2) 
            // Attract/Repel force
            float d = Vector3.Distance(nearFish[i].transform.position, this.transform.position);
            Vector3 dir = nearFish[i].transform.position - this.transform.position;
            repelForce += dir/Mathf.Pow(d,2); // separation force
            attactForce += dir/Mathf.Pow(d,1); // cohesion force
        }
        

        //m_rb.AddForce(-repelForceMultiplier*repelForce); // Not too close to others
        //m_rb.AddForce(attractForceMultiplier*attactForce); // Not too far from others
    }

    Vector3 CohesionForce()
    {
        Vector3 attactForce = new Vector3(0,0,0);
        // What are the other fish doing?
        for (int i = 0; i < nearFish.Length; i++)
        {
                // Each fish looks at all the other fish each time.fixedDeltaTime -> O(n^2) 
            // Attract/Repel force
            float d = Vector3.Distance(nearFish[i].transform.position, this.transform.position);
            Vector3 dir = nearFish[i].transform.position - this.transform.position;
            attactForce += dir/Mathf.Pow(d,1); // cohesion force
        }
        
        return attactForce;
    }

    Vector3 SeparationForce()
    {
        Vector3 repelForce = new Vector3(0,0,0);
        // What are the other fish doing?
        for (int i = 0; i < nearFish.Length; i++)
        {
                // Each fish looks at all the other fish each time.fixedDeltaTime -> O(n^2) 
            // Attract/Repel force
            float d = Vector3.Distance(nearFish[i].transform.position, this.transform.position);
            Vector3 dir = nearFish[i].transform.position - this.transform.position;
            repelForce += dir/Mathf.Pow(d,2); // cohesion force
            
        }
        return repelForce;
    }

    Vector3 CenterOfFlockmates()
    {
        Vector3 avgPos = new Vector3(0,0,0);
        // What are the other fish doing?
        for (int i = 0; i < nearFish.Length; i++)
        {
            avgPos += nearFish[i].transform.position;
        }
        return avgPos/nearFish.Length;
    }
/*
    Vector3 CalculatePDTorque(Quaternion desiredRotation)
    {
        float kp = (6f*frequency)*(6f*frequency)* 0.25f;
        float kd = 4.5f*frequency*damping;
        float dt = Time.fixedDeltaTime;
        float g = 1 / (1 + kd * dt + kp * dt * dt);
        float ksg = kp * g;
        float kdg = (kd + kp * dt) * g;
        Vector3 xVec;
        float xMag;
        Quaternion q = desiredRotation * Quaternion.Inverse(transform.rotation);
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (q.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        
        q.ToAngleAxis (out xMag, out xVec);
        xVec.Normalize ();
        xVec *= Mathf.Deg2Rad;
        Vector3 pidv = kp * xVec * xMag - kd * m_rb.angularVelocity;
        Quaternion rotInertia2World = m_rb.inertiaTensorRotation * transform.rotation;
        pidv = Quaternion.Inverse(rotInertia2World) * pidv;
        pidv.Scale(m_rb.inertiaTensor);
        pidv = rotInertia2World * pidv;
        return pidv;
    }
*/


    GameObject[] findNearFish()
    {
        float[] distances = new float[otherFish.Length];
        List<int> index_of_close_fish = new List<int>();

        for (int i = 0; i < otherFish.Length; i++)
        {
            distances[i] = Vector3.Distance(otherFish[i].transform.position, this.transform.position);
            
            if(distances[i] < searchRadius)
                index_of_close_fish.Add(i);
        }

        
        GameObject[] closeFish = new GameObject[index_of_close_fish.Count];
        for (int i = 0; i < index_of_close_fish.Count; i++)
        {
            closeFish[i] = otherFish[index_of_close_fish[i]];
        }

        return closeFish;

    }

    void ObstacleAvoidance()
    {
        // Obstacle avoidance
        for (int i = 0; i < numberOfRays; i++)
        {
            for (int j = 0; j < numberOfRays; j++)
            {

                float phi = i/(float)numberOfRays*Mathf.PI;
                float theta = j/(float)numberOfRays*Mathf.PI/2;
                Spherical localDir = new Spherical(1, phi, theta); // Hemisphere in swim direction (what do my eyes see?)
                Vector3 localDir_cart = CoordConvert.SphericalToCartesian(localDir);

                Quaternion q_wb = this.transform.rotation; // body to world
                
                Vector3 direction = q_wb * localDir_cart; // Direction of ray in world coordinates

                Ray ray = new Ray(this.transform.position, direction);
                Debug.DrawRay(this.transform.position, direction *collisionDetectionRange, Color.yellow);
                //Debug.Log(ray);
                RaycastHit hitInfo;

                // Cast a ray in ray direction and check for hit
                if(Physics.Raycast(ray, out hitInfo, collisionDetectionRange))
                {
                    Debug.DrawRay(this.transform.position, direction *hitInfo.distance, Color.red);
                    //Debug.Log(hitInfo.transform.gameObject.tag.Equals("fish"));
                    // The ray hit something, something is in front of me, I should evade
                    if(!hitInfo.transform.gameObject.tag.Equals("fish")){
                        // Oh its just another fish
                        // Apply a torque around the dorsoventral axis (yaw) to dodge when I see an obstacle in from of me
                        Debug.Log(hitInfo);
                        Vector3 evasiveManouverTorqueAxis = q_wb * Vector3.up; // change swim direction 
                        Vector3 spinTorqueAroundMainAxis = q_wb * Vector3.right; // little bit of spin around longitudinal axis
                        
                        Debug.Log((1/Mathf.Pow(hitInfo.distance, 2)*spinAvoidanceCoefficient / (float)numberOfRays) * spinTorqueAroundMainAxis);
                        //this.AddTorque(spinAvoidanceCoefficient / (float)numberOfRays * spinTorqueAroundMainAxis); // spin a bit to avoid getting stuck
                        //this.AddTorque((1/Mathf.Pow(hitInfo.distance, 2)*evasionAvoidanceCoefficient / (float)numberOfRays) * evasiveManouverTorqueAxis); // 
                        
                    }
                }
            }

        }
    }



    public static float NextGaussian() {
        float v1, v2, s;
        do {
            v1 = 2.0f * Random.Range(0f,1f) - 1.0f;
            v2 = 2.0f * Random.Range(0f,1f) - 1.0f;
            s = v1 * v1 + v2 * v2;
        } while (s >= 1.0f || s == 0f);
        s = Mathf.Sqrt((-2.0f * Mathf.Log(s)) / s);
    
        return v1 * s;
    }

    public static float NextGaussian(float mean, float standard_deviation)
    {
        return mean + NextGaussian() * standard_deviation;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controller : MonoBehaviour
{
    [SerializeField] Rigidbody m_rb; 
    GameObject targetObject; 
    [SerializeField] GameObject viz; 
    [SerializeField] float swim_speed;
    [SerializeField] float m_RotationSpeed; 
    [SerializeField]  float randomRotationVariance = 0.1f;


    [SerializeField]  float frequency = 1.0f;
    [SerializeField]  float damping = 1.0f;
    [SerializeField]  float searchRadius = 10.0f;
    
    [SerializeField]  float repelForceMultiplier = 1.0f;
    [SerializeField]  float attractForceMultiplier = 0.1f;

    GameObject[] otherFish;
    GameObject[] nearFish = new GameObject[1]; 

    Vector3 noiseVelocityRandomWalk = new Vector3(0,0,0);
     

    [SerializeField] int numberOfRays = 10;
    [SerializeField] float collisionDetectionRange = 10.0f;
    [SerializeField] float collisionAvoidanceRepelCoefficient = 1.0f;
    
    float angleMax = 90;
    int fixedUpdateCount=0;

    Vector3 repelTorque;

    // Start is called before the first frame update
    void Start()
    {
        m_rb = GetComponent<Rigidbody>();
        string my_species = "fish";
        targetObject = GameObject.FindGameObjectWithTag("Target");

        // Remove this GameObject from the list
        otherFish = GameObject.FindGameObjectsWithTag(my_species);
        
        RemoveMyselfFromOtherFishArray();

        nearFish = findNearFish();
        
    }

    void RemoveMyselfFromOtherFishArray()
    {
        List<GameObject> newOtherFish = new List<GameObject>();
        for (int i = 0; i < otherFish.Length; i++)
        {
            if(otherFish[i] != gameObject)
                newOtherFish.Add(otherFish[i]);
        }
        otherFish = newOtherFish.ToArray();
    }

    private void OnDrawGizmos() {
        // Obstacle avoidance
        /*for (int i = 0; i < numberOfRays; i++)
        {
            for (int j = 0; j < numberOfRays; j++)
            {
                Quaternion rotation = m_rb.transform.rotation;
                Quaternion rotationMody = Quaternion.AngleAxis( (i/ ((float)numberOfRays-1 ) * angleMax * 2 - angleMax), m_rb.transform.up);
                Quaternion rotationModz = Quaternion.AngleAxis( (j/ ((float)numberOfRays-1 ) * angleMax * 2 - angleMax), m_rb.transform.forward);
                Vector3 direction = rotation * rotationModz * rotationMody * new Vector3(1,0,0);

                Gizmos.DrawRay(m_rb.transform.position, direction);
            }



        }
        */

        Gizmos.DrawRay(m_rb.transform.position, repelTorque);
    }

    void FixedUpdate() {

        Vector3 deltaPosition = Vector3.zero;



        
        nearFish = findNearFish(); // only look for swarm mates in a radius around you

        Quaternion desiredRotation;
        Vector3 repelForce = new Vector3(0,0,0);
        Vector3 attactForce = new Vector3(0,0,0);


        repelTorque = new Vector3(0,0,0);
        if(fixedUpdateCount%10==0)
        {
            // Obstacle avoidance
            for (int i = 0; i < numberOfRays; i++)
            {
                for (int j = 0; j < numberOfRays; j++)
                {
                    Quaternion rotation = m_rb.transform.rotation;
                    Quaternion rotationModx = Quaternion.AngleAxis( (i/ ((float)numberOfRays-1 ) * angleMax * 2 - angleMax), m_rb.transform.up);
                    Quaternion rotationMody = Quaternion.AngleAxis( (j/ ((float)numberOfRays-1 ) * angleMax * 2 - angleMax), m_rb.transform.forward);
                    Vector3 direction = rotation * rotationModx * rotationMody * new Vector3(1,0,0);

                    Ray ray = new Ray(m_rb.transform.position, direction);
                    RaycastHit hitInfo;
                    if(Physics.Raycast(ray, out hitInfo, collisionDetectionRange))
                    {
                        if(!hitInfo.transform.gameObject.tag.Equals("fish")){
                            // I dont care about hitting other fish
                            Vector3 torqueDirection = rotation * rotationModx * rotationMody * new Vector3(0,1,0);
                            repelTorque += (collisionAvoidanceRepelCoefficient / numberOfRays) * torqueDirection;
                        }
                        
                    }
                }

            }
        }


        if(nearFish.Length > 1)
        {
            // I see other fish, I swarm


            float w=.0f,x=0.0f,y=0.0f,z=0.0f;
            // What are the other fish doing?
            for (int i = 0; i < nearFish.Length; i++)
            {
                // Each fish looks at all the other fish each time.fixedDeltaTime -> O(n^2) 
                // Attract/Repel force
                float d = Vector3.Distance(nearFish[i].transform.position, m_rb.transform.position);
                Vector3 dir = nearFish[i].transform.position - m_rb.transform.position;
                repelForce += dir/Mathf.Exp(d);
                attactForce += dir/Mathf.Pow(d,1);


                // Calculate mean rotation of all the other fish
                w+=nearFish[i].transform.rotation.w;
                x+=nearFish[i].transform.rotation.x;
                y+=nearFish[i].transform.rotation.y;
                z+=nearFish[i].transform.rotation.z;
            }
            
            Quaternion meanRotation = new Quaternion(w/otherFish.Length, x/otherFish.Length, y/otherFish.Length, z/otherFish.Length);
            desiredRotation = meanRotation;
        }
        else
        {
            // I am all alone, keep my rotation
            desiredRotation = m_rb.transform.rotation;
        }

        //desiredRotation = 

        // Random noise is accumulated into a torque vector. Noise is normal -> random walk, mean 0.
        // When I applied noise directly to torque the movement was jerky. This functions as an integrator.
        noiseVelocityRandomWalk = AddNoiseToVector(noiseVelocityRandomWalk);

        // Calculate the desired torque vector using a PD controller
        Vector3 pid_rotation = CalculatePDTorque(desiredRotation);

        m_rb.AddTorque (pid_rotation);
        m_rb.AddTorque (noiseVelocityRandomWalk, ForceMode.VelocityChange);
        m_rb.AddTorque (-repelTorque);

        
        m_rb.AddForce(-repelForceMultiplier*repelForce); // Not too close to others
        m_rb.AddForce(attractForceMultiplier*attactForce); // Not too far from others

        m_rb.AddForce(swim_speed*m_rb.transform.right, ForceMode.VelocityChange);

        fixedUpdateCount++;
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



    GameObject[] findNearFish()
    {
        float[] distances = new float[otherFish.Length];
        List<int> index_of_close_fish = new List<int>();

        for (int i = 0; i < otherFish.Length; i++)
        {
            distances[i] = Vector3.Distance(otherFish[i].transform.position, m_rb.transform.position);
            
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
    // Update is called once per frame
    void Update()
    {
        //VELOCITY MODE
        //Vector3 targetVelocity = swim_speed * transform.forward;
        //Vector3 velocityChange = targetVelocity - m_rb.velocity;
        //m_rb.AddForce(velocityChange, ForceMode.VelocityChange);

        //Vector3 dirDifference = dir - m_rb.transform.forward;
        /*
        Vector3 angularVelocity = new Vector3(1,1,1)*0.0000000001f;
        Quaternion qwb = m_rb.transform.rotation;
        Quaternion dq = qwb * new Quaternion(0, angularVelocity.x/ 2.0f, angularVelocity.y/ 2.0f, angularVelocity.z/ 2.0f);
        m_rb.AddTorque(dq.eulerAngles, ForceMode.VelocityChange);
        */
       
        //qwb.SetFromToRotation(transform.up, m_rb.transform.rotation)
        /*
        Quaternion orientation = new Quaternion()
        
        Vector3 angularVelocity = new Vector3(1,1,1);

        
        Vector3 Quaternion = new Vector3()
        Vector3 targetAngularVelocity = m_RotationSpeed * transform.up;
        Vector3 angularVelocityChange = targetAngularVelocity - m_rb.angularVelocity;
        */
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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;






public class FishSettings : MonoBehaviour
{
    [SerializeField] public float swim_speed;
    [SerializeField] public float randomRotationVariance = 0.1f;

    [Header("Cohesion PID")]
    [SerializeField] public float frequency = 1.0f;
    [SerializeField] public float damping = 1.0f;
    [SerializeField] public float searchRadius = 10.0f;
    
    [Header("Swarming")]
    [SerializeField] public float repelForceMultiplier = 1.0f;
    [SerializeField] public float attractForceMultiplier = 0.1f;

    [Header("Collision Decection")]
    [SerializeField] public float collisionDetectionRange = 10.0f;
    [SerializeField] public float evasionAvoidanceCoefficient = 1.0f;
    [SerializeField] public float spinAvoidanceCoefficient = 1.0f;

}



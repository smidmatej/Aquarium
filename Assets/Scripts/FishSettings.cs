using System.Collections;
using System.Collections.Generic;
using UnityEngine;






public class FishSettings : MonoBehaviour
{
    [Header("Fish Settings")]
    [SerializeField] public float swimSpeed;


    [Header("Obstacle Avoidance PID")]
    [SerializeField] public float frequencyPDObstacleAvoidance = 1.0f;
    [SerializeField] public float dampingPDObstacleAvoidance = 1.0f;

    [Header("Alignment PID")]
    [SerializeField] public float frequencyPDalignment = 1.0f;
    [SerializeField] public float dampingPDalignment = 1.0f;
    [SerializeField] public float swarmSearchRadius = 10.0f;


    [Header("Relative Priority of Behaviours")]
    [SerializeField] public float alignmentCoefficient = 1.0f;
    [SerializeField] public float avoidanceCoefficient = 1.0f;
    [SerializeField] public float randomTorqueCoefficient = 1.0f;


    [Header("Random Torque")]
    [SerializeField] public float randomRotationVariance = 0.1f;

    [Header("Collision Decection")]
    [SerializeField] public int numberOfRays = 10;

    [SerializeField] public float collisionDetectionRange = 10.0f;
    

}



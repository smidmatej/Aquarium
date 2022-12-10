using UnityEngine;



public class SubmarineController : MonoBehaviour
{

    public float propellerAmplitude = 10f;
    public float rotationAmplitude = 10f;
    public float upDownAmplitude = 10f;

    /*
    [Range(-180, 180)]
    public float desiredRotationX = 0f;
    [Range(-180, 180)]
    public float desiredRotationZ = 0f;

    public float proportionalParameter = 0.1f;
    */

    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {

        float forwardAmplitude = propellerAmplitude * Mathf.Clamp(Input.GetAxis("Vertical"), 0f, 1f);
        float turnAmplitude = rotationAmplitude * Input.GetAxis("Horizontal");
        float riseAmplitude = -upDownAmplitude * Input.GetAxis("Jump"); // Jump axis is used for rise and fall of the submarine (shift + space)

        Vector3 turnForce = transform.rotation * new Vector3(0, turnAmplitude, 0);
        Vector3 actuatorForce = transform.rotation * new Vector3(0, 0, forwardAmplitude);
        Vector3 upDownForce = transform.rotation * new Vector3(0, riseAmplitude, 0);

        /*
        Vector3 localAxisX = transform.rotation * new Vector3(1, 0, 0);
        Vector3 localAxisZ = transform.rotation * new Vector3(0, 0, 1);
        // P controller stabilisation calculation
        float rotationErrorX = desiredRotationX * Mathf.Deg2Rad - rb.rotation.eulerAngles.x  * Mathf.Deg2Rad;
        float rotationErrorZ = desiredRotationZ * Mathf.Deg2Rad - rb.rotation.eulerAngles.z  * Mathf.Deg2Rad;

        rotationErrorX = Mathf.Clamp(rotationErrorX, -Mathf.PI, Mathf.PI);	// Clamp rotation error to [-pi, pi]
        rotationErrorZ = Mathf.Clamp(rotationErrorZ, -Mathf.PI, Mathf.PI);	// Clamp rotation error to [-pi, pi]

        //rotationError *= -transform.rotation.eulerAngles.y; // Invert rotation error if the submarine is upside down
        Debug.Log(rb.rotation.eulerAngles);
        Vector3 stabilisationForceX = proportionalParameter * rotationErrorX * localAxisX;
        Vector3 stabilisationForceZ = proportionalParameter * rotationErrorZ * localAxisZ;

        // P Controller stabilisation
        rb.AddTorque(stabilisationForceX, ForceMode.Force);
        rb.AddTorque(stabilisationForceZ, ForceMode.Force);
        */

        // User input
        rb.AddTorque(turnForce, ForceMode.Force);
        rb.AddForce(actuatorForce, ForceMode.Force);
        rb.AddForce(upDownForce, ForceMode.Force);

    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MyMathTools;

public class OrbitalCamera : MonoBehaviour
{
    [SerializeField] Spherical m_StartSphPos = new Spherical(0f,0f,0f);   // the initial spherical position of the camera, angles are setup in degrees, and converted in radians in the Start method

    Spherical m_SphPos;         // the current spherical position
    Spherical m_TargetSphPos;         // the current target spherical position

    [SerializeField] Spherical m_SphMin;    // the minimum values for the spherical coordinates, merely rho and phi are concerned, theta is not restricted to a specific range
    [SerializeField] Spherical m_SphMax;    // the maximum values for the spherical coordinates, merely rho and phi are concerned, theta is not restricted to a specific range
    [SerializeField] Spherical m_SphSpeed;  // the spherical speed, rho in m/s, theta in degree/pixel, phi in degree/pixel
    [SerializeField] Spherical _SphLerpCoeffs;
    [SerializeField] Transform m_Target;    // the object at the centre of the orbit

    Vector3 m_PreviousMousePos;     // previous mouse position, useful to compute the mouse move vector

    void SetSphericalPosition(Spherical sphPos)
    {
        transform.position = m_Target.position + CoordConvert.SphericalToCartesian(sphPos);
        transform.LookAt(m_Target); // the camera looks at the target object
    }

    // Start is called before the first frame update
    void Start()
    {
        //Conversion from degrees to radians
        m_StartSphPos.theta *= Mathf.Deg2Rad;
        m_StartSphPos.phi *= Mathf.Deg2Rad;
        m_SphMin.phi *= Mathf.Deg2Rad;
        m_SphMax.phi *= Mathf.Deg2Rad;
        m_SphSpeed.theta *= Mathf.Deg2Rad;
        m_SphSpeed.phi *= Mathf.Deg2Rad;

        // Positions at start
        m_SphPos = m_StartSphPos;           // spherical position initialization
        m_TargetSphPos = m_SphPos;          // target position initialization

        SetSphericalPosition(m_SphPos);     // camera position initialization

        m_PreviousMousePos = Input.mousePosition;   // previous mouse position initialization
    }

    // Update is called once per frame
    void LateUpdate()
    {
        // Please implement the following steps:
        // Let's compute the mouse motion vector (coordinates are in in pixels)
        // 1 - retrieve the current mouse position and store it into a local variable
        Vector3 m_MousePos = Input.mousePosition;
        // 2 - compute the mouse motion vector by subtracting the previous mouse position (m_PreviousMousePos) from the current mouse position
        Vector3 m_MouseMotion = m_MousePos - m_PreviousMousePos;
        // 3 - update the previous mouse position variable with the current mouse position
        m_PreviousMousePos = m_MousePos;
        // Let's compute the new spherical position of the camera
        // 4 - compute the spherical displacement of the camera during the frame
        Spherical m_sphMotion;
        if (Input.GetMouseButton(1) == true)
            //m_sphMotion = CoordConvert.CartesianToSpherical(m_MouseMotion);
            m_sphMotion = new Spherical(0,
                                        m_MouseMotion.x * m_SphSpeed.phi,
                                        m_MouseMotion.y * m_SphSpeed.theta);
        else
            m_sphMotion = new Spherical(0, 0, 0);


        // 5 - compute the new spherical position of the camera, taking into account the min and max limits for rho and phi
        /*
        m_SphPos.phi -= m_sphMotion.phi * Time.deltaTime;
        m_SphPos.theta -= m_sphMotion.theta * Time.deltaTime;
        m_SphPos.r -= Input.mouseScrollDelta.y * Time.deltaTime * 10;
        */
        m_TargetSphPos.r -= Input.mouseScrollDelta.y * m_SphSpeed.r;
        m_TargetSphPos.phi -= m_sphMotion.phi;
        m_TargetSphPos.theta -= m_sphMotion.theta;
        // 6 - assign the new position of the camera by calling the SetSphericalPosition method
        Spherical newSphPos = new Spherical(Mathf.Lerp(m_SphPos.r, m_TargetSphPos.r, Time.deltaTime * _SphLerpCoeffs.r),
                                            Mathf.Lerp(m_SphPos.phi, m_TargetSphPos.phi, Time.deltaTime * _SphLerpCoeffs.phi),
                                            Mathf.Lerp(m_SphPos.theta, m_TargetSphPos.theta, Time.deltaTime * _SphLerpCoeffs.theta));
        m_SphPos = newSphPos;
        SetSphericalPosition(newSphPos);
        //Hints:
        // Input.mouseScrollDelta.y gives you the mouse wheel increment during the frame
        // Input.mousePosition gives you the current mouse position on the screen, in pixels
        // Input.GetMouseButton(1) returns true if the right mouse button is hold pressed
        // Mathf.Clamp clamps a value within a range defined by a min and a max
    }
}
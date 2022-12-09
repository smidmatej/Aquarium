using UnityEngine;

namespace MyMathTools
{
    [System.Serializable]
    public struct Polar
    {
        public float Rho;
        public float Theta;

        public Polar(Polar pol)
        {
            this.Rho = pol.Rho;
            this.Theta = pol.Theta;
        }

        public Polar(float rho, float theta)
        {
            this.Rho = rho;
            this.Theta = theta;
        }
    }

    [System.Serializable]
    public class Cylindrical
    {
        public float r;
        public float theta;
        public float z;
        
        public Cylindrical(float r, float theta, float z)
        {
            this.r = r;
            this.theta = theta;
            this.z = z;
        }
    }

    public class Spherical
    {
        public float r;
        public float phi;
        public float theta;
        public Spherical(float r, float phi, float theta)
        {
            this.r = r;
            this.phi = phi;
            this.theta = theta;
        }

        public Spherical Lerp(Spherical target, Spherical next)
        {
            Vector3 targetVector = Vector3.Lerp(CoordConvert.SphericalToCartesian(target), CoordConvert.SphericalToCartesian(next), 0.5f);
            return CoordConvert.CartesianToSpherical(targetVector);
        }
    }

    public static class CoordConvert
    {
        public static Vector2 PolarToCartesian(Polar polar)
        {
            return polar.Rho * new Vector2(Mathf.Cos(polar.Theta), Mathf.Sin(polar.Theta));
        }

        public static Polar CartesianToPolar(Vector2 cart, bool keepThetaPositive = true )
        {
            Polar polar= new Polar(cart.magnitude,0);
            if (Mathf.Approximately(polar.Rho, 0)) polar.Theta = 0;
            else
            {
                polar.Theta = Mathf.Asin(cart.y / polar.Rho);
                if (cart.x < 0) polar.Theta = Mathf.PI - polar.Theta;
                if (keepThetaPositive && polar.Theta < 0) polar.Theta += 2 * Mathf.PI;
            }
            return polar;
        }

        public static Vector3 SphericalToCartesian(Spherical spherical)
        {

            Vector3 cart = new Vector3(Mathf.Sin(spherical.theta)*Mathf.Cos(spherical.phi),
                                        Mathf.Cos(spherical.theta), 
                                        Mathf.Sin(spherical.theta)*Mathf.Sin(spherical.phi)) * spherical.r;
            return cart;
        }

        public static Spherical CartesianToSpherical(Vector3 cart)
        {

            Spherical sph = new Spherical(0, Mathf.Atan2(cart.z, cart.x), Mathf.Atan2(Mathf.Sqrt(cart.x*cart.x + cart.z*cart.z), cart.y));
            return sph;
        }

        public static Vector3 CylindricalToCartesian(Cylindrical cyl)
        {

            Vector3 cart = new Vector3(cyl.r * Mathf.Cos(cyl.theta),
                                        cyl.r * Mathf.Sin(cyl.theta), 
                                        cyl.z);
            return cart;
        }
    }
}
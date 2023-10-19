using System;

namespace rt
{
    public class Sphere : Geometry
    {
        private Vector Center { get; set; }
        private double Radius { get; set; }

        public Sphere(Vector center, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            Radius = radius;
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            double b = line.X0.X;
            double d = line.X0.Y;
            double f = line.X0.Z;

            double a = line.Dx.X;
            double c = line.Dx.Y;
            double e = line.Dx.Z;

            double term1 = a * a + c * c + e * e;
            double term2 = 2 * a * b - 2 * a * Center.X + 2 * c * d - 2 * c * Center.Y + 2 * e * f - 2 * e * Center.Z;
            double term3 = b * b + Center.X * Center.X - 2 * b * Center.X + d * d + Center.Y * Center.Y -
                2 * d * Center.Y + f * f + Center.Z * Center.Z - 2 * f * Center.Z - Radius * Radius;

            double delta = term2 * term2 - 4 * term1 * term3;

            if (delta < 0)
                return new Intersection();

            double t1 = (-term2 + Math.Sqrt(delta)) / (2 * term1);
            double t2 = (-term2 - Math.Sqrt(delta)) / (2 * term1);

            double smallestT = Math.Min(t1, t2);
            
            //No looking behind check
            double largestT = Math.Max(t1, t2);
            double finalT = smallestT;

            if (smallestT < 0)
            {
                if (largestT < 0)
                    return new Intersection();
                finalT = largestT;
            }
            //end of check
            
            if (finalT < minDist || finalT > maxDist)
                return new Intersection();
            return new Intersection(true, true, this, line, finalT);//use smallestT to remove the check
        }

        public override Vector Normal(Vector v)
        {
            var n = v - Center;
            n.Normalize();
            return n;
        }
    }
}
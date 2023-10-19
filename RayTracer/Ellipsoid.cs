using System;


namespace rt
{
    public class Ellipsoid : Geometry
    {
        private Vector Center { get; }
        private Vector SemiAxesLength { get; }
        private double Radius { get; }
        
        
        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Material material, Color color) : base(material, color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public Ellipsoid(Vector center, Vector semiAxesLength, double radius, Color color) : base(color)
        {
            Center = center;
            SemiAxesLength = semiAxesLength;
            Radius = radius;
        }

        public override Intersection GetIntersection(Line line, double minDist, double maxDist)
        {
            double a = line.Dx.X;
            double c = line.Dx.Y;
            double e = line.Dx.Z;
            
            double b = line.X0.X;
            double d = line.X0.Y;
            double f = line.X0.Z;

            double A = SemiAxesLength.X;
            double B = SemiAxesLength.Y;
            double C = SemiAxesLength.Z;

            double term1 = ((a * a) / (A * A)) +
                           ((c * c) / (B * B)) +
                           ((e * e) / (C * C));
            
            double term2 = ((2 * a * b - 2 * a * Center.X) / (A * A)) + 
                           ((2 * c * d - 2 * c * Center.Y) / (B * B)) +
                           ((2 * e * f - 2 * e * Center.Z) / (C * C));
            
            double term3 = (b * b - 2 * Center.X * b + Center.X * Center.X) / (A * A) +
                           (d * d - 2 * Center.Y * d + Center.Y * Center.Y) / (B * B) +
                           (f * f - 2 * Center.Z * f + Center.Z * Center.Z) / (C * C) - 
                           Radius * Radius;
            
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
            
            return new Intersection();
        }
    }
}

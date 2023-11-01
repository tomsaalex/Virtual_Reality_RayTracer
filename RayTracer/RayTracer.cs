using System;
using System.Runtime.InteropServices;

namespace rt
{
    class RayTracer
    {
        private Geometry[] geometries;
        private Light[] lights;

        public RayTracer(Geometry[] geometries, Light[] lights)
        {
            this.geometries = geometries;
            this.lights = lights;
        }

        private double ImageToViewPlane(int n, int imgSize, double viewPlaneSize)
        {
            return -n * viewPlaneSize / imgSize + viewPlaneSize / 2;
        }

        private Intersection FindFirstIntersection(Line ray, double minDist, double maxDist)
        {
            var intersection = new Intersection();

            foreach (var geometry in geometries)
            {
                var intr = geometry.GetIntersection(ray, minDist, maxDist);

                if (!intr.Valid || !intr.Visible) continue;

                if (!intersection.Valid || !intersection.Visible)
                {
                    intersection = intr;
                }
                else if (intr.T < intersection.T)
                {
                    intersection = intr;
                }
            }

            return intersection;
        }

        private bool IsLit(Vector point, Light light)
        {
            Line pointToLightRay = new Line(point, light.Position);
            double pointToLightDistance = 10000;//(point - light.Position).Length();

            Intersection intersectionInFrontOfLight = FindFirstIntersection(pointToLightRay, 0.3, pointToLightDistance);
            
            if (!intersectionInFrontOfLight.Valid || !intersectionInFrontOfLight.Visible)
            {
                return true;
            }

            return Math.Abs(intersectionInFrontOfLight.Position.X - point.X) <= 0.0001 &&
                   Math.Abs(intersectionInFrontOfLight.Position.Y - point.Y) <= 0.0001 &&
                   Math.Abs(intersectionInFrontOfLight.Position.Z - point.Z) <= 0.0001;
        }

        public void Render(Camera camera, int width, int height, string filename)
        {
            var background = new Color(0.2, 0.2, 0.2, 1.0);

            var viewParallel = (camera.Up ^ camera.Direction).Normalize();
            var image = new Image(width, height);

            var vecW = camera.Direction * camera.ViewPlaneDistance;
            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    Vector rayTracingOrigin = camera.Position;
                    Vector rayTracingVector = camera.Position + 
                                              camera.Direction * camera.ViewPlaneDistance +
                                              camera.Up * ImageToViewPlane(j, height, camera.ViewPlaneHeight) +
                                              viewParallel * ImageToViewPlane(i, width, camera.ViewPlaneWidth);

                    Line sightLine = new Line(rayTracingOrigin, rayTracingVector);

                    Intersection geometryIntersection =  FindFirstIntersection(sightLine, camera.FrontPlaneDistance, camera.BackPlaneDistance);

                    Color pixelColor = new Color(0, 0, 0, 1);
                    if (geometryIntersection.Valid && geometryIntersection.Visible)
                    {
                        Vector pointOnSurface = sightLine.CoordinateToPosition(geometryIntersection.T);
                        Vector N = geometryIntersection.Normal;
                        Vector E = (camera.Position - pointOnSurface).Normalize();
                        
                        foreach (Light light in lights)
                        {

                            Vector T = (light.Position - pointOnSurface).Normalize();
                            
                            double NTDotProduct = N * T;
                            
                            Vector R = (N * NTDotProduct * 2 - T).Normalize();
                            
                            double ERDotProduct = E * R;
                            
                            
                            pixelColor += geometryIntersection.Geometry.Material.Ambient * light.Ambient;
                            
                            
                            
                            if (IsLit(pointOnSurface, light))
                            {
                                if(NTDotProduct > 0)
                                    pixelColor += geometryIntersection.Geometry.Material.Diffuse * light.Diffuse * NTDotProduct;
                                if(ERDotProduct > 0)
                                    pixelColor += geometryIntersection.Geometry.Material.Specular * light.Specular * Math.Pow(ERDotProduct, geometryIntersection.Geometry.Material.Shininess);
                            }
                        }
                    }

                    double redComponent   = Math.Clamp(pixelColor.Red, 0, 1);
                    double blueComponent  = Math.Clamp(pixelColor.Blue, 0, 1);
                    double greenComponent = Math.Clamp(pixelColor.Green, 0, 1);
                    double alphaComponent = Math.Clamp(pixelColor.Alpha, 0, 1);

                    Color newPixelColor = new Color(redComponent, greenComponent, blueComponent, alphaComponent);
                    
                    image.SetPixel(i, j, newPixelColor);
                }
            }

            image.Store(filename);
        }
    }
}
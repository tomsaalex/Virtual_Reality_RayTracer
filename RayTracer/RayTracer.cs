﻿using System;
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
            // TODO: ADD CODE HERE
            return true;
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

                    Color pixelColor = background;
                    if(geometryIntersection.Valid  && geometryIntersection.Visible)
                        pixelColor += geometryIntersection.Geometry.Color;
                    
                    image.SetPixel(i, j, pixelColor);
                }
            }

            image.Store(filename);
        }
    }
}
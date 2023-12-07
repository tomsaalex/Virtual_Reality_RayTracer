using System;
using System.Drawing.Drawing2D;
using System.IO;
using System.Numerics;
using System.Text.RegularExpressions;

namespace rt;

public class RawCtMask : Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public RawCtMask(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) :
        base(Color.NONE)
    {
        _position = position;
        _scale = scale;
        _colorMap = colorMap;

        var lines = File.ReadLines(datFile);
        foreach (var line in lines)
        {
            //TODO: possibly wrong regex? Why separate by \ and t instead of that \t tab character?
            var kv = Regex.Replace(line, "[:\\t ]+", ":").Split(":");
            if (kv[0] == "Resolution")
            {
                _resolution[0] = Convert.ToInt32(kv[1]);
                _resolution[1] = Convert.ToInt32(kv[2]);
                _resolution[2] = Convert.ToInt32(kv[3]);
            }
            else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0] * _thickness[0] * scale, _resolution[1] * _thickness[1] * scale,
            _resolution[2] * _thickness[2] * scale);
        Console.WriteLine(_v0.X + " " + _v0.Y + " " + _v0.Z);
        Console.WriteLine(_v1.X + " " + _v1.Y + " " + _v1.Z);
        var len = _resolution[0] * _resolution[1] * _resolution[2];
        _data = new byte[len];
        using FileStream f = new FileStream(rawFile, FileMode.Open, FileAccess.Read);
        if (f.Read(_data, 0, len) != len)
        {
            throw new InvalidDataException($"Failed to read the {len}-byte raw data");
        }
    }

    private ushort Value(int x, int y, int z)
    {
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public struct NutData
    {
        public Color c;
        public Vector normal;
        public double t;

        public NutData(Color c, Vector normal, double t)
        {
            this.c = c;
            this.normal = normal;
            this.t = t;
        }
    }

    public Intersection intersectsPlane(Line line, double a, double b, double c, double d, double e, double f,
        Vector referencePoint, double minDist, double maxDist)
    {
        // Note: In this function, we NEED to allow intersections with negative t. Otherwise, the model will not render if the camera is inside the box, but outside the visible model.
        // See: Frame 33

        double m, n, p, q;
        double A, B, C, D, E, F;
        double x0, y0, z0;

        x0 = referencePoint.X;
        y0 = referencePoint.Y;
        z0 = referencePoint.Z;

        A = line.Dx.X;
        C = line.Dx.Y;
        E = line.Dx.Z;

        B = line.X0.X;
        D = line.X0.Y;
        F = line.X0.Z;

        m = b * f - e * c;
        n = c * d - a * f;
        p = a * e - b * d;
        q = -x0 * b * f + x0 * e * c - y0 * c * d + y0 * a * f - z0 * a * e + z0 * b * d;

        if (Math.Abs(m * A + n * C + p * E) <= 0.000001)
            return Intersection.NONE;


        double t = -(m * B + n * D + p * F + q) / (m * A + n * C + p * E);

        //if(t < 0 /*&& t < minDist || t > maxDist*/)
        //    return Intersection.NONE;

        Vector intersectionPosition = line.CoordinateToPosition(t);
        int[] indexes = GetIndexes(intersectionPosition);


        if (indexes[0] < 0 || indexes[1] < 0 || indexes[2] < 0 || indexes[0] > _resolution[0] ||
            indexes[1] > _resolution[1] || indexes[2] > _resolution[2])
        {
            return Intersection.NONE;
        }

        Material testMaterial = new Material(new Color(1, 0, 0, 1), new Color(1, 0, 0, 1), new Color(1, 0, 0, 1), 50);

        return new Intersection(true, true, this, line, t, GetNormal(intersectionPosition), testMaterial, Color.RED);
    }

    public NutData sampleColorThroughCube(Line line, double t)
    {
        List<NutData> samplingData = new List<NutData>();

        double stepSizeT = 0.05;
        
        Vector coordinatePosition = line.CoordinateToPosition(t);
        int[] indexes = GetIndexes(coordinatePosition);
        int[] oldIndexes;
        double cummulativeAlpha = 0;
        
        while (!(indexes[0] < 0 || indexes[1] < 0 || indexes[2] < 0 || indexes[0] > _resolution[0] ||
               indexes[1] > _resolution[1] || indexes[2] > _resolution[2]))
        {
            
            oldIndexes = indexes;
            coordinatePosition = line.CoordinateToPosition(t);
            indexes = GetIndexes(coordinatePosition);
            
            if (oldIndexes[0] == indexes[0] && oldIndexes[1] == indexes[1] && oldIndexes[2] == indexes[2])
            {
                t += stepSizeT;
                continue;
            }

            samplingData.Add(new NutData(GetColor(coordinatePosition), GetNormal(coordinatePosition), t));
            t += stepSizeT;
        }
        
        double intersectionWithModelT = 0;
        bool hasIntersectedModel = false;
        Color combinedColor = new Color();

        for (int i = 1; i < samplingData.Count; i++)
        {
            Color currentColor = samplingData[i].c;
            double currentT = samplingData[i].t;
            
            if (!hasIntersectedModel)
            {
                if (currentColor.Alpha <= 0.0001)
                    intersectionWithModelT = currentT;
                else
                {
                    hasIntersectedModel = true;
                }
            }
        }
    
        for (int i = samplingData.Count - 1; i >= 0; i--)
        {
            Color currentColor = samplingData[i].c;
            combinedColor = currentColor * currentColor.Alpha + combinedColor * (1 - currentColor.Alpha);
        }

        Vector intersectionCoordinates = line.CoordinateToPosition(intersectionWithModelT);
        return new NutData(combinedColor, GetNormal(intersectionCoordinates), intersectionWithModelT);
    }
    

    public override Intersection GetIntersection(Line line, double minDist, double maxDist)
    {
        List<Intersection> intersections = new List<Intersection>();

        intersections.Add(intersectsPlane(line, 1, 0, 0, 0, 0, 1, _v0, minDist, maxDist));
        intersections.Add(intersectsPlane(line, 1, 0, 0, 0, 1, 0, _v0, minDist, maxDist));
        intersections.Add(intersectsPlane(line, 0, 0, 1, 0, 1, 0, _v0, minDist, maxDist));

        intersections.Add(intersectsPlane(line, 1, 0, 0, 0, 0, 1, _v1, minDist, maxDist));
        intersections.Add(intersectsPlane(line, 1, 0, 0, 0, 1, 0, _v1, minDist, maxDist));
        intersections.Add(intersectsPlane(line, 0, 0, 1, 0, 1, 0, _v1, minDist, maxDist));

        intersections[0].Material.Ambient = new Color(1f, 1f, 1f, 1);
        intersections[1].Material.Ambient = new Color(0f, 1f, 0f, 1);
        intersections[2].Material.Ambient = new Color(1f, 0f, 0f, 1);
        intersections[3].Material.Ambient = new Color(0f, 0f, 1f, 1);
        intersections[4].Material.Ambient = new Color(1f, 1f, 0f, 1);
        intersections[5].Material.Ambient = new Color(1f, 0f, 1f, 1);

        Intersection closestIntersection = Intersection.NONE;
        double t = Double.PositiveInfinity;

        foreach (var intersection in intersections)
        {
            if (intersection.Visible && intersection.Valid && intersection.T < t)
            {
                closestIntersection = intersection;
                t = intersection.T;
            }
        }


        if (closestIntersection.Visible && closestIntersection.Valid)
        {
            NutData nd = sampleColorThroughCube(line, closestIntersection.T);
            Color c = nd.c;
            
            // Using the T value of the intersection with the model instead of the T value of the intersection with
            // the cube causes some weird black lines to appear on the model. 
            // Probably some shadows from some weird normals calculation. Not quite sure why.
            double newT = nd.t;
            
            if (Math.Abs(c.Alpha) <= 0.00001)
                return Intersection.NONE;

            if (newT < 0 && newT < minDist || newT > maxDist)
            {
                closestIntersection = Intersection.NONE;
            }
            else
            {
                closestIntersection =
                    new Intersection(true, true, this, line, newT, nd.normal, Material.FromColor(c), c);
            }
        }


        return closestIntersection;
    }

    private int[] GetIndexes(Vector v)
    {
        return new[]
        {
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale),
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)
        };
    }

    private Color GetColor(Vector v)
    {
        int[] idx = GetIndexes(v);

        ushort value = Value(idx[0], idx[1], idx[2]);
        return _colorMap.GetColor(value);
    }

    private Vector GetNormal(Vector v)
    {
        int[] idx = GetIndexes(v);
        double x0 = Value(idx[0] - 1, idx[1], idx[2]);
        double x1 = Value(idx[0] + 1, idx[1], idx[2]);
        double y0 = Value(idx[0], idx[1] - 1, idx[2]);
        double y1 = Value(idx[0], idx[1] + 1, idx[2]);
        double z0 = Value(idx[0], idx[1], idx[2] - 1);
        double z1 = Value(idx[0], idx[1], idx[2] + 1);

        return new Vector(x1 - x0, y1 - y0, z1 - z0).Normalize();
    }
}
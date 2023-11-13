using System;
using System.Drawing.Drawing2D;
using System.IO;
using System.Text.RegularExpressions;

namespace rt;

public class RawCtMask: Geometry
{
    private readonly Vector _position;
    private readonly double _scale;
    private readonly ColorMap _colorMap;
    private readonly byte[] _data;

    private readonly int[] _resolution = new int[3];
    private readonly double[] _thickness = new double[3];
    private readonly Vector _v0;
    private readonly Vector _v1;

    public RawCtMask(string datFile, string rawFile, Vector position, double scale, ColorMap colorMap) : base(Color.NONE)
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
            } else if (kv[0] == "SliceThickness")
            {
                _thickness[0] = Convert.ToDouble(kv[1]);
                _thickness[1] = Convert.ToDouble(kv[2]);
                _thickness[2] = Convert.ToDouble(kv[3]);
            }
        }

        _v0 = position;
        _v1 = position + new Vector(_resolution[0]*_thickness[0]*scale, _resolution[1]*_thickness[1]*scale, _resolution[2]*_thickness[2]*scale);

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
        /*if (x >= -10 && x < 0) x = 0;
        if (y >= -10 && y < 0) y = 0;
        if (z >= -10 && z < 0) z = 0;
        
        if (x >= _resolution[0] && x < _resolution[0] + 10) x = _resolution[0] - 1;
        if (y >= _resolution[1] && y < _resolution[1] + 10) y = _resolution[1] - 1;
        if (z >= _resolution[2] && z < _resolution[2] + 10) z = _resolution[2] - 1;
*/
        if (x < 0 || y < 0 || z < 0 || x >= _resolution[0] || y >= _resolution[1] || z >= _resolution[2])
        {
            return 0;
        }

        return _data[z * _resolution[1] * _resolution[0] + y * _resolution[0] + x];
    }

    public Intersection intersectsPlane(Line line, double a, double b, double c, double d, double e, double f, Vector referencePoint, double minDist, double maxDist)
    {
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
        

        double t = - (m * B + n * D + p * F + q) / (m * A + n * C + p * E);
        
        //if(t < 0 && t < minDist || t > maxDist)
        //    return Intersection.NONE;

        Vector intersectionPosition = line.CoordinateToPosition(t);
        int[] indexes = GetIndexes(intersectionPosition);

        /*if (indexes[0] >= -10 && indexes[0] < 0) indexes[0] = 0;
        if (indexes[1] >= -10 && indexes[1] < 0) indexes[1] = 0;
        if (indexes[2] >= -10 && indexes[2] < 0) indexes[2] = 0;
        
        
        if (indexes[0] >= _resolution[0] && indexes[0] < _resolution[0] + 10) indexes[0] = _resolution[0] - 1;
        if (indexes[1] >= _resolution[1] && indexes[1] < _resolution[1] + 10) indexes[1] = _resolution[1] - 1;
        if (indexes[2] >= _resolution[2] && indexes[2] < _resolution[2] + 10) indexes[2] = _resolution[2] - 1;*/
        
        if (indexes[0] < 0 || indexes[1] < 0 || indexes[2] < 0 || indexes[0] > _resolution[0] ||
            indexes[1] > _resolution[1] || indexes[2] > _resolution[2])
        {
            return Intersection.NONE;
        }
        
        Material testMaterial = new Material(new Color(1, 0, 0, 1), new Color(1, 0, 0, 1), new Color(1, 0, 0, 1), 50);
        
        return new Intersection(true, true, this, line, t, GetNormal(intersectionPosition), testMaterial, Color.RED);
    }

    public Color sampleColorThroughCube(Line line, double t, int[] oldIndices)
    {
        double copyT = t;

        Vector coordinatePosition = line.CoordinateToPosition(copyT); 
        int[] indexes = GetIndexes(coordinatePosition);
        
        // TODO: I don't understand why this is correct. The index value is correct in both 0 and in _resolution[0/1/2]. Omit either one and you get weird rendering errors. Not sure why.
        if (indexes[0] < 0 || indexes[1] < 0 || indexes[2] < 0 || indexes[0] > _resolution[0] ||
            indexes[1] > _resolution[1] || indexes[2] > _resolution[2])
        {
            return new Color(1, 1, 1, 0);
        }
        
        //Maybe not necessary. This ensures a cell's color is only sampled once.
        if (oldIndices.Length != 0)
        {
            if (oldIndices[0] == indexes[0] && oldIndices[0] == indexes[0] && oldIndices[0] == indexes[0])
                return sampleColorThroughCube(line, copyT + 1, indexes);
        }
        
        Color c = GetColor(coordinatePosition);
        return c * c.Alpha + sampleColorThroughCube(line, copyT + 1, indexes) * (1 - c.Alpha);
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

        Color c = sampleColorThroughCube(line, t, new int[0]);
        closestIntersection.Material = Material.FromColor(c);
        
        // Not the most elegant way to do this, probably, but it hides the background of the MRI.
        // The partially see-though parts are still mixed with the default color though.
        if(Math.Abs(c.Blue - 1) <= 0.00001 && Math.Abs(c.Green - 1) <= 0.00001 && Math.Abs(c.Red - 1) <= 0.00001 && c.Alpha <= 0.00001)
            return Intersection.NONE;
        
        return closestIntersection;
    }
    
    private int[] GetIndexes(Vector v)
    {
        return new []{
            (int)Math.Floor((v.X - _position.X) / _thickness[0] / _scale), 
            (int)Math.Floor((v.Y - _position.Y) / _thickness[1] / _scale),
            (int)Math.Floor((v.Z - _position.Z) / _thickness[2] / _scale)};
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
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace DummyWpfUpper.Services;

public static class StlMeshLoader
{
    public static MeshGeometry3D Load(string path, double scale = 0.001)
    {
        using var stream = File.OpenRead(path);
        using var reader = new BinaryReader(stream);
        byte[] header = reader.ReadBytes(80);

        string head = Encoding.ASCII.GetString(header);
        if (head.TrimStart().StartsWith("solid", StringComparison.OrdinalIgnoreCase))
            return LoadAscii(path, scale);

        uint count = reader.ReadUInt32();
        var positions = new Point3DCollection();
        var indices = new Int32Collection();

        for (uint i = 0; i < count; i++)
        {
            reader.ReadBytes(12);
            Point3D a = ReadPoint(reader, scale);
            Point3D b = ReadPoint(reader, scale);
            Point3D c = ReadPoint(reader, scale);
            reader.ReadUInt16();

            int baseIndex = positions.Count;
            positions.Add(a);
            positions.Add(b);
            positions.Add(c);
            indices.Add(baseIndex);
            indices.Add(baseIndex + 1);
            indices.Add(baseIndex + 2);
        }

        var mesh = new MeshGeometry3D { Positions = positions, TriangleIndices = indices };
        mesh.Freeze();
        return mesh;
    }

    private static MeshGeometry3D LoadAscii(string path, double scale)
    {
        var positions = new Point3DCollection();
        var indices = new Int32Collection();
        var vertices = new List<Point3D>(3);

        foreach (string raw in File.ReadLines(path))
        {
            string line = raw.Trim();
            if (!line.StartsWith("vertex", StringComparison.OrdinalIgnoreCase))
                continue;

            string[] parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 4)
                continue;

            double x = double.Parse(parts[1], CultureInfo.InvariantCulture) * scale;
            double y = double.Parse(parts[2], CultureInfo.InvariantCulture) * scale;
            double z = double.Parse(parts[3], CultureInfo.InvariantCulture) * scale;
            vertices.Add(new Point3D(x, y, z));

            if (vertices.Count == 3)
            {
                int baseIndex = positions.Count;
                foreach (Point3D p in vertices)
                    positions.Add(p);
                indices.Add(baseIndex);
                indices.Add(baseIndex + 1);
                indices.Add(baseIndex + 2);
                vertices.Clear();
            }
        }

        var mesh = new MeshGeometry3D { Positions = positions, TriangleIndices = indices };
        mesh.Freeze();
        return mesh;
    }

    private static Point3D ReadPoint(BinaryReader reader, double scale)
    {
        float x = reader.ReadSingle();
        float y = reader.ReadSingle();
        float z = reader.ReadSingle();
        return new Point3D(x * scale, y * scale, z * scale);
    }
}

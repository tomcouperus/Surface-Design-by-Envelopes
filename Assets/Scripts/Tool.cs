using UnityEngine;

[RequireComponent(typeof(MeshRenderer))]
[RequireComponent(typeof(MeshFilter))]
public abstract class Tool : MonoBehaviour
{
    [SerializeField]
    private Mesh mesh;

    [Header("Render")]
    [SerializeField]
    [Min(1)]
    private int tSectors = 50;
    [SerializeField]
    [Min(1)]
    private int aSectors = 20;

    [Header("Settings")]
    [SerializeField]
    protected float height = 1;
    [SerializeField]
    protected float radius = 0.5f;


    protected void Awake()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        mesh = new();
        meshFilter.mesh = mesh;
    }

    protected void Start()
    {
        UpdateTool();
    }

    public void UpdateTool()
    {
        GenerateMeshData().CreateMesh(mesh);
    }

    private MeshData GenerateMeshData()
    {
        MeshData data = new MeshData(tSectors, aSectors + 1, xWrap: true);
        int vertexIndex = 0;
        for (int aIdx = 0; aIdx <= aSectors; aIdx++)
        {
            float a = (float)aIdx / aSectors;
            for (int tIdx = 0; tIdx < tSectors; tIdx++)
            {
                float t = (float)tIdx / tSectors;

                Vector3 vertex = GetToolSurfaceAt(t, a);
                Vector2 uv = new Vector2(t, a);

                data.AddVertex(vertex, uv, vertexIndex);
                bool makeTriangle = aIdx < aSectors;
                bool atFinalT = tIdx % tSectors == tSectors - 1;
                if (makeTriangle)
                {
                    int v1, v2, v3, v4;
                    v1 = vertexIndex;
                    v3 = vertexIndex + tSectors;
                    if (atFinalT)
                    {
                        v2 = vertexIndex - tSectors + 1;
                        v4 = vertexIndex + 1;
                    }
                    else
                    {
                        v2 = vertexIndex + 1;
                        v4 = vertexIndex + tSectors + 1;
                    }
                    data.AddTriangle(v1, v3, v2);
                    data.AddTriangle(v2, v3, v4);
                }
                vertexIndex++;
            }

        }
        return data;
    }

    protected abstract Vector3 GetToolSurfaceAt(float t, float a);

    public abstract float GetRadiusAt(float a);

    public abstract float GetRadiusDaAt(float a);

    public abstract float GetSphereCenterHeightAt(float a);

    public abstract float GetSphereCenterHeightDaAt(float a);

    public abstract float GetSphereRadiusAt(float a);

    public abstract float GetSphereRadiusDaAt(float a);

}

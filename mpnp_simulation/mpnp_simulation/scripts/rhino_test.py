import rhino3dm
import polyscope as ps
import numpy as np

def load_rhino_voronoi(file_path):
    model = rhino3dm.File3dm.Read(file_path)
    ps.init()
    
    for i, obj in enumerate(model.Objects): #type: ignore
        geom = obj.Geometry
        mesh_data = None

        if isinstance(geom, rhino3dm.Mesh):
            mesh_data = geom
        elif isinstance(geom, rhino3dm.Brep):
            # Try to extract the display mesh from the Brep
            joined_mesh = rhino3dm.Mesh()
            for face in geom.Faces: #type: ignore
                m = face.GetMesh(rhino3dm.MeshType.Any) #type: ignore
                if m: joined_mesh.Append(m)
            mesh_data = joined_mesh

        if mesh_data:
            # Convert to numpy
            verts = np.array([[v.X, v.Y, v.Z] for v in mesh_data.Vertices]) #type: ignore
            faces = []
            for j in range(len(mesh_data.Faces)): #type: ignore
                f = mesh_data.Faces[j] #type: ignore
                faces.append([f[0], f[1], f[2]])
                if f[2] != f[3]: # Handle Quads
                    faces.append([f[0], f[2], f[3]])
            
            faces = np.array(faces)

            # Register Mesh in Polyscope
            mesh_name = f"Cell_{i}"
            ps_mesh = ps.register_surface_mesh(mesh_name, verts, faces)

            # --- VORONOI DATA CHECK ---
            # Calculate centers and normals to verify your logic
            # (Using a simple triangle average for the center)
            tri_verts = verts[faces]
            centers = np.mean(tri_verts, axis=1)
            
            # Calculate actual normals
            v0, v1, v2 = tri_verts[:, 0], tri_verts[:, 1], tri_verts[:, 2]
            normals = np.cross(v1 - v0, v2 - v0)
            normals /= np.linalg.norm(normals, axis=1)[:, np.newaxis]

            # Visualize the normals as vectors originating from the centers
            ps_mesh.add_vector_quantity("face_normals", normals, defined_on='faces', color=(1, 0, 0))

    ps.show()

load_rhino_voronoi("src/object_rearrangement_ros2_sim/mpnp_simulation/models/voronoi.3dm")
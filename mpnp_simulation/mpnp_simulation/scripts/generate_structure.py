import numpy as np
import polyscope as ps

from scipy.spatial import Voronoi, ConvexHull
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from typing import List

@dataclass
class Surface:
    center: List[float]
    normal: List[float]

class Polygon:
    def __init__(self, vertices, block_id, world_pos, world_quat):
        self.block_id = block_id
        # Convert vertices to local frame (centered at 0,0,0)
        self.local_vertices = vertices - world_pos
        self.pos = np.array(world_pos)
        self.quat = np.array(world_quat) # [x, y, z, w]
        self.surfaces = self._generate_surfaces()

    def _generate_surfaces(self):
        # Use ConvexHull on local vertices to find faces and normals
        hull = ConvexHull(self.local_vertices)
        surfaces = []
        for i, eq in enumerate(hull.equations):
            normal = eq[:3] # Outward normal
            # Find face center by averaging vertices in this simplex
            face_verts = self.local_vertices[hull.simplices[i]]
            center = np.mean(face_verts, axis=0)
            surfaces.append(Surface(center.tolist(), normal.tolist()))
        return surfaces

    def get_world_vertices(self):
        """Applies current rotation and translation to local vertices."""
        rotation = R.from_quat(self.quat)
        rotated_verts = rotation.apply(self.local_vertices)
        return rotated_verts + self.pos

def generate_voronoi_wall(width=1.0, thickness=0.2, height=1.0, num_blocks=10):
    # 1. Randomly seed points inside the wall volume
    seeds = np.random.uniform(
        [0, 0, 0], 
        [width, thickness, height], 
        (num_blocks, 3)
    )

    # 2. Add 'Dummy' points far away to ensure all inner cells are bounded
    # (Voronoi needs a boundary or cells go to infinity)
    boundary = 2.0
    dummies = np.array([
        [width/2, thickness/2, -boundary], [width/2, thickness/2, height+boundary],
        [-boundary, thickness/2, height/2], [width+boundary, thickness/2, height/2],
        [width/2, -boundary, height/2], [width/2, thickness+boundary, height/2]
    ])
    all_points = np.vstack([seeds, dummies])

    vor = Voronoi(all_points)

    blocks = []
    for i in range(num_blocks):
        # Get indices of vertices for this cell
        region_idx = vor.point_region[i]
        vert_indices = vor.regions[region_idx]

        if -1 in vert_indices or len(vert_indices) == 0:
            continue # Skip infinite cells

        cell_verts = vor.vertices[vert_indices]
        center = seeds[i]
        blocks.append(Polygon(cell_verts, f"block_{i}", center, [0, 0, 0, 1]))

    return blocks

def generate_voronoi_wall_hull(width=1.0, thickness=0.2, height=1.0, num_blocks=10):
    unique_faces = []
    seen_normals = []
    angle_threshold = 0.98  # Cosine of angle threshold for merging faces

    # 1. Randomly seed points inside the wall volume
    seeds = np.random.uniform(
        [0, 0, 0], 
        [width, thickness, height], 
        (num_blocks, 3)
    )

    # 2. Add 'Dummy' points far away to ensure all inner cells are bounded
    # (Voronoi needs a boundary or cells go to infinity)
    boundary = 2.0
    dummies = np.array([
        [width/2, thickness/2, -boundary], [width/2, thickness/2, height+boundary],
        [-boundary, thickness/2, height/2], [width+boundary, thickness/2, height/2],
        [width/2, -boundary, height/2], [width/2, thickness+boundary, height/2]
    ])
    all_points = np.vstack([seeds, dummies])

    vor = Voronoi(all_points)

    blocks = []
    for i in range(num_blocks):
        # Get indices of vertices for this cell
        region_idx = vor.point_region[i]
        vert_indices = vor.regions[region_idx]

        if -1 in vert_indices or len(vert_indices) == 0:
            continue # Skip infinite cells

        cell_verts = vor.vertices[vert_indices]
        hull = ConvexHull(cell_verts)
        for i, eq in enumerate(hull.equations):
            normal = eq[:3]
            is_duplicate = False

            for sn in seen_normals:
                # Dot product check: 1.0 means identical, 0.0 means perpendicular
                if np.dot(normal, sn) > angle_threshold:
                    is_duplicate = True
                    break

            if not is_duplicate:
                # Calculate the face center (mean of vertices in this simplex)
                face_verts = hull.points[hull.simplices[i]]
                center = np.mean(face_verts, axis=0)

                unique_faces.append({
                    'normal': normal,
                    'center': center,
                    'equation': eq
                })
                seen_normals.append(normal)

        return unique_faces

"""
    Try out stability scoring here. Use the projection -> union method to get a score.
"""


def visualize_wall(blocks, shrink_factor=0.95):
    """
    Visualizes the Voronoi wall in Polyscope.
    shrink_factor: < 1.0 creates a small gap between blocks to see them individually.
    """
    ps.init()
    ps.set_up_dir("z_up")

    for block in blocks:
        # 1. Get vertices in world space
        world_verts = block.get_world_vertices()

        # 2. Apply shrink (optional, for visual clarity)
        center = block.pos
        shrunk_verts = center + (world_verts - center) * shrink_factor

        # 3. Register with Polyscope
        # We need the hull simplices for the mesh faces
        hull = ConvexHull(block.local_vertices)

        mesh = ps.register_surface_mesh(
            block.block_id, 
            shrunk_verts, 
            hull.simplices,
            enabled=True
        )

        # 4. Visualize the Surface Normals for one block as a test
        # We'll put small arrows at the face centers
        centers = np.array([s.center for s in block.surfaces]) + block.pos
        normals = np.array([s.normal for s in block.surfaces])

        pc = ps.register_point_cloud(f"{block.block_id}_faces", centers)
        pc.add_vector_quantity("normals", normals, enabled=True, color=(1, 0, 0))

    ps.show()

def main():
    # --- Execution ---
    wall_blocks = generate_voronoi_wall(num_blocks=10)
    visualize_wall(wall_blocks)

if __name__ == "__main__":  
    main()
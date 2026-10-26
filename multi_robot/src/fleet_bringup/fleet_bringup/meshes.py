"""Deterministic closed prism meshes; concave collisions use convex pieces."""
from pathlib import Path
import numpy as np
from fleet_mapping.shapes import triangulate, polygon_area


def write_prism(path, points, height=1.2):
    points = np.asarray(points, float)
    if polygon_area(points) < 0:
        points = points[::-1]
    bottom = np.column_stack((points, np.full(len(points), -height/2)))
    top = np.column_stack((points, np.full(len(points), height/2)))
    facets = []
    for triangle in triangulate(points):
        facets.extend([top[triangle], bottom[triangle[::-1]]])
    for i in range(len(points)):
        j = (i+1) % len(points)
        facets.extend([[bottom[i], bottom[j], top[j]], [bottom[i], top[j], top[i]]])
    lines = ["solid obstacle"]
    for facet in facets:
        a, b, c = np.asarray(facet)
        normal = np.cross(b-a, c-a)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            raise ValueError("退化 STL 三角面")
        normal /= norm
        lines.append("  facet normal " + " ".join(f"{v:.9g}" for v in normal))
        lines.append("    outer loop")
        lines.extend("      vertex " + " ".join(f"{v:.9g}" for v in p) for p in facet)
        lines.extend(["    endloop", "  endfacet"])
    lines.append("endsolid obstacle")
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines)+"\n")

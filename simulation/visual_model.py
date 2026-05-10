import os
import numpy as np
from Basilisk.utilities import macros, vizSupport


def add_box_triangles(vertices, faces, center, size):
    """Append a box mesh (12 triangles) to the vertex/face lists for OBJ export."""
    cx, cy, cz = center
    sx, sy, sz = size
    hx, hy, hz = 0.5 * sx, 0.5 * sy, 0.5 * sz

    box_vertices = [
        [cx - hx, cy - hy, cz - hz],
        [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz],
        [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz],
        [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz],
        [cx - hx, cy + hy, cz + hz],
    ]

    base = len(vertices) + 1
    vertices.extend(box_vertices)

    box_faces = [
        [base + 0, base + 1, base + 2], [base + 0, base + 2, base + 3],
        [base + 4, base + 6, base + 5], [base + 4, base + 7, base + 6],
        [base + 0, base + 4, base + 5], [base + 0, base + 5, base + 1],
        [base + 1, base + 5, base + 6], [base + 1, base + 6, base + 2],
        [base + 2, base + 6, base + 7], [base + 2, base + 7, base + 3],
        [base + 3, base + 7, base + 4], [base + 3, base + 4, base + 0],
    ]
    faces.extend(box_faces)


def add_cylinder_triangles(vertices, faces, center, radius, height, n_segs=16, axis="z"):
    """Append a capped cylinder to the vertex/face lists."""
    cx, cy, cz = center
    base = len(vertices) + 1
    if axis not in ("x", "y", "z"):
        raise ValueError(f"Unsupported cylinder axis '{axis}'")

    axis_start = -0.5 * height  # [m]
    axis_end = 0.5 * height  # [m]

    for axis_offset in (axis_start, axis_end):
        for k in range(n_segs):
            ang = 2.0 * np.pi * k / n_segs
            if axis == "z":
                vertices.append([
                    cx + radius * np.cos(ang),
                    cy + radius * np.sin(ang),
                    cz + axis_offset,
                ])
            elif axis == "y":
                vertices.append([
                    cx + radius * np.cos(ang),
                    cy + axis_offset,
                    cz + radius * np.sin(ang),
                ])
            else:
                vertices.append([
                    cx + axis_offset,
                    cy + radius * np.cos(ang),
                    cz + radius * np.sin(ang),
                ])

    if axis == "z":
        cap_a = [cx, cy, cz + axis_start]
        cap_b = [cx, cy, cz + axis_end]
    elif axis == "y":
        cap_a = [cx, cy + axis_start, cz]
        cap_b = [cx, cy + axis_end, cz]
    else:
        cap_a = [cx + axis_start, cy, cz]
        cap_b = [cx + axis_end, cy, cz]

    vertices.append(cap_a)
    vertices.append(cap_b)
    bot_cap = base + 2 * n_segs
    top_cap = base + 2 * n_segs + 1

    for k in range(n_segs):
        k1 = k
        k2 = (k + 1) % n_segs
        faces.append([base + k1, base + k2, base + n_segs + k2])
        faces.append([base + k1, base + n_segs + k2, base + n_segs + k1])
        faces.append([bot_cap, base + k2, base + k1])
        faces.append([top_cap, base + n_segs + k1, base + n_segs + k2])


def append_component_box(vertices, faces, face_materials, center, size, material_name):
    """Append a box component and tag all generated faces with one material."""
    start_face_count = len(faces)
    add_box_triangles(vertices, faces, center, size)
    added_face_count = len(faces) - start_face_count
    if added_face_count > 0:
        face_materials.extend([material_name] * added_face_count)


def add_panel_triangles(vertices, faces, center, size):
    """Append a panel as a thin box."""
    add_box_triangles(vertices, faces, center, size)


def add_xz_face_cell_grid(
    vertices,
    faces,
    face_materials,
    face_center_x_m,
    face_surface_y_m,
    face_center_z_m,
    face_span_x_m,
    face_span_z_m,
    cell_span_x_m,
    cell_span_z_m,
    cell_thickness_m,
    n_cols,
    n_rows,
    normal_sign,
    material_name,
):
    """Append a grid of solar-cell tiles on a face parallel to the x-z plane."""
    x_start_m = face_center_x_m - 0.5 * face_span_x_m + 0.5 * cell_span_x_m
    z_start_m = face_center_z_m - 0.5 * face_span_z_m + 0.5 * cell_span_z_m
    cell_center_y_m = face_surface_y_m + normal_sign * 0.5 * cell_thickness_m

    for row_idx in range(n_rows):
        for col_idx in range(n_cols):
            cell_center_x_m = x_start_m + col_idx * cell_span_x_m
            cell_center_z_m = z_start_m + row_idx * cell_span_z_m
            append_component_box(
                vertices,
                faces,
                face_materials,
                center=[cell_center_x_m, cell_center_y_m, cell_center_z_m],
                size=[cell_span_x_m, cell_thickness_m, cell_span_z_m],
                material_name=material_name,
            )


def write_obj(path, vertices, faces, face_materials, panel_open):
    """Write OBJ+MTL with component materials and double-sided triangles."""
    mtl_name = os.path.splitext(os.path.basename(path))[0] + ".mtl"
    mtl_path = os.path.join(os.path.dirname(path), mtl_name)

    with open(mtl_path, "w", encoding="utf-8") as mtl_file:
        mtl_file.write("newmtl body_mat\nKd 0.68 0.68 0.70\nKa 0.30 0.30 0.30\nKs 0.06 0.06 0.06\n\n")
        mtl_file.write("newmtl antenna_mat\nKd 0.50 0.50 0.52\nKa 0.22 0.22 0.22\nKs 0.03 0.03 0.03\n\n")
        if panel_open:
            mtl_file.write("newmtl panel_mat\nKd 0.10 0.44 0.92\nKa 0.08 0.10 0.20\nKs 0.03 0.04 0.08\n\n")
        else:
            mtl_file.write("newmtl panel_mat\nKd 0.56 0.56 0.58\nKa 0.24 0.24 0.24\nKs 0.03 0.03 0.03\n\n")
        mtl_file.write("newmtl cell_mat\nKd 1.00 1.00 1.00\nKa 0.90 0.90 0.90\nKs 0.02 0.02 0.02\n\n")

    normals = []
    face_records = []
    for tri, mtl in zip(faces, face_materials):
        p0 = np.array(vertices[tri[0] - 1], dtype=float)
        p1 = np.array(vertices[tri[1] - 1], dtype=float)
        p2 = np.array(vertices[tri[2] - 1], dtype=float)
        n = np.cross(p1 - p0, p2 - p0)
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-12:
            n = np.array([0.0, 0.0, 1.0])
        else:
            n /= n_norm
        normals.append(n)
        face_records.append((tri, mtl, len(normals)))

    with open(path, "w", encoding="utf-8") as obj_file:
        obj_file.write(f"mtllib {mtl_name}\n")
        for v in vertices:
            obj_file.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for n in normals:
            obj_file.write(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")

        current_mtl = None
        for tri, mtl, n_idx in face_records:
            if mtl != current_mtl:
                obj_file.write(f"usemtl {mtl}\n")
                current_mtl = mtl
            obj_file.write(f"f {tri[0]}//{n_idx} {tri[1]}//{n_idx} {tri[2]}//{n_idx}\n")
            obj_file.write(f"f {tri[0]}//{n_idx} {tri[2]}//{n_idx} {tri[1]}//{n_idx}\n")


def build_satellite_obj(path, panels_open, body_size_x_m, body_size_y_m, body_size_z_m):
    """Build full spacecraft mesh (body + antenna + panels) and write OBJ."""
    verts = []
    faces = []
    face_materials = []

    def mark_component(material_name, start_face_count):
        added = len(faces) - start_face_count
        if added > 0:
            face_materials.extend([material_name] * added)

    start_faces = len(faces)
    add_box_triangles(
        verts,
        faces,
        center=[0.0, 0.0, 0.0],
        size=[body_size_x_m, body_size_y_m, body_size_z_m],
    )
    mark_component("body_mat", start_faces)

    ant_length = 0.08
    ant_radius = 0.005
    ant_cx = 0.0
    start_faces = len(faces)
    add_cylinder_triangles(
        verts,
        faces,
        center=[ant_cx, -ant_length, 0.0],
        radius=ant_radius,
        height=ant_length,
        axis="y",
    )
    mark_component("antenna_mat", start_faces)

    comms_ant_cx = 0.5 * body_size_x_m  # [m]
    comms_ant_cy = 0.0  # [m]
    comms_ant_cz = 0.0  # [m]
    start_faces = len(faces)
    add_cylinder_triangles(
        verts,
        faces,
        center=[comms_ant_cx - 0.05, comms_ant_cy+ ant_length, comms_ant_cz ],
        radius=ant_radius,
        height=ant_length,
        axis="y",
    )
    mark_component("antenna_mat", start_faces)

    solar_cell_cols = 2  # [-]
    solar_cell_rows = 3  # [-]
    solar_cell_area_m2 = 27.0e-4  # [m^2]
    solar_cell_span_x_m = 0.030  # [m]
    solar_cell_span_z_m = solar_cell_area_m2 / solar_cell_span_x_m  # [m]
    solar_cell_thickness_m = 0.0012  # [m]
    panel_span_x_m = 0.60 * body_size_x_m  # [m]
    panel_span_z_m = body_size_z_m  # [m]
    panel_thickness_m = 0.003  # [m]
    panel_open_x_m = -0.5 * body_size_x_m - 0.5 * panel_span_x_m  # [m]
    panel_open_y_m = 0.5 * body_size_y_m + 0.5 * panel_thickness_m  # [m]
    panel_face_sign = -1.0  # [-]

    start_faces = len(faces)
    add_panel_triangles(
        verts,
        faces,
        center=[panel_open_x_m, panel_face_sign * panel_open_y_m, 0.0],
        size=[panel_span_x_m, panel_thickness_m, panel_span_z_m],
    )
    add_panel_triangles(
        verts,
        faces,
        center=[panel_open_x_m + body_size_y_m + panel_span_x_m, panel_face_sign * panel_open_y_m, 0.0],
        size=[panel_span_x_m, panel_thickness_m, panel_span_z_m],
    )
    mark_component("panel_mat", start_faces)

    if panels_open:
        pass
    else:
        pass
    bus_face_center_x_m = 0.0  # [m]
    bus_face_center_y_m = 0.5 * body_size_y_m  # [m]
    bus_face_center_z_m = 0.0  # [m]
    bus_face_sign = 1.0  # [-]
    write_obj(path, verts, faces, face_materials, panel_open=panels_open)


def apply_visual_model(viz, spacecraft_tag, model_path):
    """Replace spacecraft visual with a single custom model path."""
    vizSupport.customModelList = []
    vizSupport.createCustomModel(
        viz,
        modelPath=model_path,
        rotation=[0.0, -90.0 * macros.D2R, 0.0],
        scale=[1.0, 1.0, 1.0],
        simBodiesToModify=[spacecraft_tag],
    )
    try:
        viz.settings.dataFresh = 1
    except Exception:
        pass

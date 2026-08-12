# Backend: FastAPI (Python)

from fastapi import FastAPI, File, UploadFile
from fastapi.middleware.cors import CORSMiddleware
import open3d as o3d
import numpy as np
import trimesh
import os

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def process_stl(file_path):
    try:
        mesh = o3d.io.read_triangle_mesh(file_path)
        if not mesh.has_triangles():
            return None, "Error: STL file has no triangles. Try repairing the file."
        
        mesh.compute_vertex_normals()
        pcd = mesh.sample_points_poisson_disk(1000)
        waypoints = np.asarray(pcd.points)
    except Exception:
        try:
            mesh = trimesh.load_mesh(file_path)
            waypoints, _ = trimesh.sample.sample_surface(mesh, 1000)
        except Exception as err:
            return None, f"Error processing STL file: {str(err)}"

    return waypoints.tolist(), None


@app.post("/upload/")
async def upload_file(file: UploadFile = File(...)):
    file_path = f"./temp_{file.filename}"
    try:
        contents = await file.read()
        with open(file_path, "wb") as f:
            f.write(contents)
        waypoints, error = process_stl(file_path)
    except Exception as e:
        waypoints, error = None, str(e)
    finally:
        if os.path.exists(file_path):
            os.remove(file_path)

    return {"waypoints": waypoints, "error": error}

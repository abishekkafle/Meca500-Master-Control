import streamlit as st
import open3d as o3d
import numpy as np
import trimesh
import os
import plotly.graph_objects as go

def process_stl(file_path):
    try:
        # Load STL using Open3D
        mesh = o3d.io.read_triangle_mesh(file_path)
        if not mesh.has_triangles():
            return None, "Error: STL file has no triangles. Try repairing the file."
        
        mesh.compute_vertex_normals()
        pcd = mesh.sample_points_poisson_disk(1000)
        waypoints = np.asarray(pcd.points)
    except:
        # Backup processing with Trimesh
        mesh = trimesh.load_mesh(file_path)
        waypoints, _ = trimesh.sample.sample_surface(mesh, 1000)
    
    return waypoints, None

def plot_point_cloud(points):
    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2],
        mode='markers',
        marker=dict(size=3, color=points[:, 2], colorscale='Viridis', opacity=0.8)
    ))
    
    fig.update_layout(
        title='Extracted Point Cloud Visualization',
        scene=dict(
            xaxis_title='X Axis',
            yaxis_title='Y Axis',
            zaxis_title='Z Axis',
            aspectmode='data'
        ),
        margin=dict(l=0, r=0, b=0, t=40)
    )
    
    st.plotly_chart(fig)

st.title("Meca 500 Path Planner")

uploaded_file = st.file_uploader("Upload STL/OBJ file", type=["stl", "obj"])

if uploaded_file is not None:
    file_path = f"./temp_{uploaded_file.name}"
    with open(file_path, "wb") as f:
        f.write(uploaded_file.getbuffer())
    
    st.success("File uploaded successfully!")
    
    waypoints, error = process_stl(file_path)
    
    if error:
        st.error(error)
    else:
        st.success(f"Extracted {len(waypoints)} waypoints!")
        
        # Display waypoints
        st.dataframe(waypoints)
        
        # Visualize point cloud
        plot_point_cloud(waypoints)
        
        # Allow download
        waypoint_txt = "\n".join([f"{x},{y},{z}" for x, y, z in waypoints])
        st.download_button(label="Download Waypoints", data=waypoint_txt, file_name="waypoints.txt", mime="text/plain")
    
    os.remove(file_path)
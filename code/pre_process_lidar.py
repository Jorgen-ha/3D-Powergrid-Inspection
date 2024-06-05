""" 
This script will take in all the point clouds generated by the LiDAR,
and extract only the points relevant to us (masts + ground).
If no mast is found, the file is discarded. After saving all the filtered
point clouds to .las files, they are converted to .ply (needed for open3d).
The final output is two folders; one with the filtered .las files, and one
with the .ply files.
"""
import open3d as o3d
import pandas as pd
import numpy as np
import laspy
import pdal
import glob
import ast
import sys
import utm
import os 

def filter_classes(las_file, out_name):
    """Function filtering a las file, keeping only the points classified as
    ground (2) or masts (15).

    Args:
        las_file (str): path to the las file
        out_name (str): path to the output file

    Returns:
        str: path to the output file
    """
    with laspy.open(las_file) as f:
        with laspy.open(out_name, mode="w", header=f.header) as filtered_file:
            for points in f.chunk_iterator(1_000_000):
                filtered_file.write_points(points[(points.classification == 2) | (points.classification == 15)])
    
    return out_name

def filter_all_las_pcds(las_dir_path, output_path):
    """Function filtering all the las files in a directory, keeping only 
    the points classified as ground (2) or masts (15).

    Args:
        las_dir_path (str): path to the directory holding all the las files
        output_path (_type_): path to an existing directory to store the filtered las files

    Returns:
        str: path to the output directory
    """
    all_paths = glob.glob(las_dir_path + "*.las")
    filenames = [path.split("/")[-1] for path in all_paths]
    for path, filename in zip(all_paths, filenames):
        out_name = output_path + filename
        filter_classes(path, out_name)
    
    return output_path

def generate_las_to_ply_json(input_name, output_name):
    """Function generating a json string for PDAL to convert a las file to a ply file.
    
    Args:
        input_name (str): path to the input las file
        output_name (str): path to the output ply file
    Returns:
        str: json string for PDAL pipeline
    """
    json_str = f"""
    [
        {{
            "type" : "readers.las",
            "filename" : "{input_name}"
        }},
        {{
            "type" : "writers.ply",
            "storage_mode" : "little endian",
            "filename" : "{output_name}"
        }}
    ]"""
    
    return json_str

def merge_all_las(in_dir, out_name):
    """Function merging all given las files into one.

    Args:
        in_dir (path): path to the directory holding all the las files
        out_name (str): path to the output file

    Returns:
        str: path to the output file
    """
    all_in_names = glob.glob(in_dir + "*.las")
    command = f"pdal merge {' '.join(all_in_names)} {out_name}"
    os.system(command)
    
    return out_name

def las_to_ply_dir(las_dir_path, output_path):
    """Function converting all las files inside a directory to ply files
    inside a different directory using PDAL.

    Args:
        las_dir_path (str): path to the directory holding all the las files
        output_path (str): path to an existing directory to store the ply files

    Returns:
        str: path to the output directory
    """
    for paths in las_dir_path: 
        in_name = paths.split("/")[-1]
        out_name = output_path + in_name.split(".")[0] + ".ply"
        json_str = generate_las_to_ply_json(in_name, out_name)
        
        # Create and execute a PDAL pipeline
        pipeline = pdal.Pipeline(json_str)
        pipeline.execute()
        
    return output_path

def las_to_ply_file(las_file, out_name):
    """Function converting a las file to a ply file using PDAL.

    Args:
        las_file (str): path of the las file
        out_name (str): output path of the ply file

    Returns:
        str: path of the output file
    """
    json_str = generate_las_to_ply_json(las_file, out_name)
    pipeline = pdal.Pipeline(json_str)
    pipeline.execute()
    
    return out_name

def gps_to_utm(img_list_path):
    """Function translating all GPS coordinates of the masts to UTM
    coordinates (nearest ten round off). 

    Args:
        img_list_path (str): full path to the mast_image_list.csv file

    Returns:
        utm_coords (np.array): UTM coordinates of the masts
    """
    csv = pd.read_csv(img_list_path)
    
    # Extract the mast names (only the first two letters to separate them)
    mast_names = csv['asset_id'].apply(lambda x: x[:2]).unique()

    # Extract the GPS coordinates of the masts (a string, we keep lat, lon, alt)
    coordinates = csv['asset_pos'].apply(lambda x: x[33:-1]).unique()
    coordinates = np.asarray([ast.literal_eval(s) for s in coordinates])
    
    # Convert the coordinates to UTM
    utm_coords_zone = [utm.from_latlon(lat, lon, 32, 'V') for lon, lat, _ in coordinates]
    utm_lat_lon = np.asarray(utm_coords_zone)[:,:2].astype(float)
    utm_coords = np.concatenate((utm_lat_lon, coordinates[:,2].reshape(-1,1)), axis=1)
        
    return utm_coords, mast_names

def find_extract_mast(utm_coords, mast_names, ply_filtered_merged, output_dir):
    """Function to find the relevant mast in the classified and merged 
    LiDAR point cloud holding all the masts, before extracting to its own file.

    Args:
        utm_coords (np.array): UTM coordinates of the center of the masts
        mast_names (np.array): names of the masts
        ply_filtered_merged (str): path to the ply file holding all the masts
        output_dir (str): path to the directory to store the cropped masts

    Returns:
        str: path to the output directory
    """
    all_lidars_merged = np.asarray(o3d.io.read_point_cloud(ply_filtered_merged).points)
    for coord, name in zip(utm_coords, mast_names):
        east, north, z = coord
        """Given the coordinate, extract the points in a bounding box defined
        around this point (which denotes the center of the mast's top bar)."""
        filter_east = np.logical_and(all_lidars_merged[:,0] > east-4, all_lidars_merged[:,0] < east+4)
        filter_north = np.logical_and(all_lidars_merged[:,1] > north-4, all_lidars_merged[:,1] < north+4)
        filter_z = np.logical_and(all_lidars_merged[:,2] > z-15, all_lidars_merged[:,2] < z+2)
        filter = np.logical_and(np.logical_and(filter_east, filter_north), filter_z)
        mast_points = all_lidars_merged[filter]
        if len(mast_points) == 0:
            print(f"No mast found for {name}")
            continue
        # Save the mast points to a new ply file
        mast_pcd = o3d.geometry.PointCloud()
        mast_pcd.points = o3d.utility.Vector3dVector(mast_points)
        o3d.io.write_point_cloud(f"{output_dir}{name}_{int(east)}_{int(north)}.ply", mast_pcd)
        
    return output_dir


""" Script should be ran with input <las_dir_path> (path to the directory holding all lidar point clouds),
 <img_list_csv_path> (path to the given mast_image_list.csv for the images) and <output_dir> (the 
 directory to store all the output. Created if not existing) as arguments. 
 Example: python3 pre-process-lidar.py ../data/classified_used/ mast_image_list.csv ../data/output-test/"""
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python3 pre-process-lidar.py [las_dir] [img_list_csv_path] [output_dir]")
        sys.exit(1)
    las_dir_path = sys.argv[1]
    img_list = sys.argv[2]
    output_dir = sys.argv[3]
    if not os.path.exists(output_dir):
        os.system(f"mkdir {output_dir} && mkdir {output_dir}lidar_mast_ground/")
    if not os.path.exists(output_dir + "lidar_mast_ground/"):
        os.system(f"mkdir {output_dir}lidar_mast_ground/")
    if not os.path.exists(output_dir + "all_merged/"):
        os.system(f"mkdir {output_dir}all_merged/")
    
    all_las_dir = filter_all_las_pcds(las_dir_path, output_dir)
    out_name = merge_all_las(all_las_dir, output_dir + "all_merged/all_filtered_merged.las")
    ply_filtered_merged = las_to_ply_file(out_name, output_dir + "all_merged/all_filtered_merged.ply")
    utm_coords, mast_names = gps_to_utm(img_list)
    masts_cropped_dir = output_dir + "lidar_mast_ground/"
    find_extract_mast(utm_coords, mast_names, ply_filtered_merged, masts_cropped_dir)
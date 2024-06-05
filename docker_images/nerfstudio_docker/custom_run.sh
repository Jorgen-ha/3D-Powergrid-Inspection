: << Description 
This is a simple script for inputting images to the NeRF
frameworks in Nerfstudio. 
It simply process the images with Colmap, to get the necessary camera poses
for the nerf training. Then it trains using the sought for model, before
finally allowing the user to view the results in the viewer or export rendered
videos or pointclouds.
All commands have detailed helper functions, and sub commands as well. 
Simply do: 
ns-process-data -h (or --help)
ns-process-data images -h (or --help)
Description

# Start by processing the data (which is of type 'images')
# if the output directory doesn't exist, it will be created
ns-process-data images --data PATH-TO-IMAGES \
    --output-dir PATH-WHERE-OUTPUT-IS-WANTED \
    --matching-method exhaustive --sfm-tool colmap

# Train a model using the processed data and the NeRF framework you want.
# Default is 'nerfacto', but 'instant-ngp' is popular also
# Normals must be predicted in order to export pointclouds
ns-train TYPE-OF-NERF-FRAMEWORK \
    --data PATH-WHERE-OUTPUT-IS-WANTED \
    --pipeline.model.predict-normals True \
    --output-dir PATH-WHERE-OUTPUT-IS-WANTED/outputs \
    --pipeline.datamanager.train-num-images-to-sample-from NUMBER \
    --viewer.quit-on-train-completion True 

# View the trained model, if the viewer was closed upon completion
ns-viewer --load-config PATH-WHERE-OUTPUT-IS-WANTED/outputs/METHOD/DATE_TIME/config.yml

: << c
Below are commands as they are default in Nerfstudio from the viewer. 
All will not work with all frameworks, so the documentation should be 
consulted. 
c

# Export a pointcloud (only possible if normals were predicted during training)
ns-export pointcloud \
    --load-config PATH-WHERE-OUTPUT-IS-WANTED/outputs/METHOD/DATE_TIME/config.yml \
    --output-dir PATH-WHERE-OUTPUT-IS-WANTED/exports/pcd/ \
    --num-points 1000000 --remove-outliers True \
    --normal-method open3d --use_bounding_box False --save-world-frame False \
    --obb_center 0.0000000000 0.0000000000 -0.0257857927 \
    --obb_rotation 0.0000000000 0.0000000000 0.0000000000 \
    --obb_scale 1.0000000000 1.0000000000 1.0000000000


# Render result
ns-render camera-path \
    --load-config nerfstudio/instant-ngp/config.yml \
    --camera-path-filename nerfstudio/instant-ngp/camera_poses.json \
    --output-path nerfstudio/instant-ngp/output/result_video.mp4

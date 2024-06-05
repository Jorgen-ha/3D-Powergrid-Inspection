# Script performing dense reconstruction from an already computed sparse model


gpu=$1
WORKSPACE=$2
IMAGES=$3
if [ $# -ne 3 ]; then
    echo "Usage: $(basename ${0}) <gpu_id(0 or 1)> <workspace_dir_name_> (dir of the sparse model inside 'colmap-testing') <image_dir_name> (inside 'colmap-testing/images')"
    echo "Workspace name MUST be without trailing '/'. The parent of the sparse folder for the respective model."
    exit 1
fi

echo "IMPORTANT!! Podman version must be 4.1.0 or higher!"

BASE=/home/colmap-testing

# Check if workspace exists, if not create it and other needed folders
if [ ! -d "$BASE/$WORKSPACE/dense" ]; then
    mkdir $BASE/$WORKSPACE/output $BASE/$WORKSPACE/dense $BASE/$WORKSPACE/dense/0
else
    echo "Workspace already exists, proceeding..."
fi

if [ ! -d "$BASE/images/$IMAGES" ]; then
    echo "Couldn't find image folder $IMAGES"
    exit 1
fi

# Remove any prior podman instance with this name
podman rm colmap-dense

# Run the colmap image in detached mode in the background
# This also pulls the image if not already present
podman run --name colmap-dense -d \
    --device nvidia.com/gpu=$gpu \
    -w /working -v $BASE:/working \
    -it colmap-modified:latest

# Perform dense reconstruction in steps (undistort, patchmatch, fuse)
podman exec colmap-dense colmap image_undistorter \
    --image_path images/$IMAGES \
    --input_path $WORKSPACE/sparse/0 \
    --output_path $WORKSPACE/dense 
    # --max_image_size 32

podman exec colmap-dense colmap patch_match_stereo \
    --workspace_path $WORKSPACE/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.max_image_size 2000 

podman exec colmap-dense colmap stereo_fusion \
    --workspace_path $WORKSPACE/dense \
    --input_type geometric \
    --output_path $WORKSPACE/dense/0/fused.ply 

cp $BASE/$WORKSPACE/dense/0/fused.ply $BASE/$WORKSPACE/output/fused.ply

# Remove/reset the podman image colmap-dense
podman stop colmap-dense && podman rm colmap-dense

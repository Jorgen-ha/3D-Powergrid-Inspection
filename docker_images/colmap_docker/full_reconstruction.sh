# Script running Colmaps dense reconstruction


gpu=$1
WORKSPACE=$2
IMAGES=$3
GEO=$4
if [ $# -ne 4 ]; then
    echo "Usage: $(basename ${0}) <gpu_id(0 or 1)> <workspace_dir_name_> (created inside 'colmap-testing') <image_dir_name> (inside 'colmap-testing/images') <1 or 0> (whether to align model or not)"
    echo "Workspace name MUST be without trailing '/'. If aligning, put 'geo-reg.txt' inside 'colmap-testing' folder."
    exit 1
fi

echo "IMPORTANT!! Podman version must be 4.1.0 or higher!"

BASE=/home/colmap-testing

# Check if workspace exists, if not create it and other needed folders
if [ ! -d "$BASE/$WORKSPACE" ]; then
    mkdir $BASE/$WORKSPACE $BASE/$WORKSPACE/output ~/scp/$WORKSPACE\
    $BASE/$WORKSPACE/sparse $BASE/$WORKSPACE/dense $BASE/$WORKSPACE/dense/0
else
    echo "Workspace already exists, proceeding..."
fi

if [ $GEO -eq 1 ]; then
    if [ ! -f "$BASE/geo-reg.txt" ]; then
        echo "geo-reg.txt not found. Please put it in the 'colmap-testing' folder."
        exit 1
    else
        mv $BASE/geo-reg.txt $BASE/$WORKSPACE/geo-reg.txt
    fi
else 
    echo "Not aligning model with geo coordinates."
fi

# Remove any prior podman instance with this name
podman rm colmap-img

# Run the colmap image in detached mode in the background
# This also pulls the image if not already present
podman run --name colmap-img -d \
    --device nvidia.com/gpu=$gpu \
    -w /working -v $BASE:/working \
    -it colmap-modified:latest

# Make sure any prior database is properly cleaned
podman exec colmap-img colmap database_cleaner \
    --type all \
    --database_path $WORKSPACE/database.db

img_size=3200

# Execute manual reconstruction
podman exec colmap-img colmap feature_extractor \
   --database_path $WORKSPACE/database.db \
   --image_path images/$IMAGES \
   --ImageReader.single_camera 1 \
   --SiftExtraction.max_image_size $img_size  # (default: 3200) 

# Exhaustive matching
podman exec colmap-img colmap exhaustive_matcher \
   --database_path $WORKSPACE/database.db 

# Maps the matched features and performs bundle adjustment
podman exec colmap-img colmap mapper \
    --database_path $WORKSPACE/database.db \
    --image_path images/$IMAGES \
    --output_path $WORKSPACE/sparse

if [ $GEO -eq 1 ]; then
    # Align the sparse reconstruction with geo coordinates
    podman exec colmap-img colmap model_aligner \
        --input_path $WORKSPACE/sparse/0 \
        --output_path $WORKSPACE/sparse/0 \
        --ref_is_gps 1 \
        --ref_images_path $WORKSPACE/geo-reg.txt \
        --alignment_type ecef \
        --alignment_max_error 1.5
fi  


# Perform dense reconstruction in steps (undistort, patchmatch, fuse)
podman exec colmap-img colmap image_undistorter \
    --image_path images/$IMAGES \
    --input_path $WORKSPACE/sparse/0 \
    --output_path $WORKSPACE/dense 
    
podman exec colmap-img colmap patch_match_stereo \
    --workspace_path $WORKSPACE/dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.max_image_size 2000 \
    --PatchMatchStereo.geom_consistency true \
    --PatchMatchStereo.filter true

podman exec colmap-img colmap stereo_fusion \
    --workspace_path $WORKSPACE/dense \
    --input_type geometric \
    --output_path $WORKSPACE/dense/0/fused.ply 
    # --StereoFusion.max_image_size $img_size # ?

# Copy relevant point clouds to output folder and scp folder
cp $BASE/$WORKSPACE/dense/0/fused.ply $BASE/$WORKSPACE/output/fused.ply
# cp $BASE/$WORKSPACE/dense/0/fused.ply ~/scp/$WORKSPACE/fused.ply

# Remove/reset the podman image colmap-img
podman stop colmap-img && podman rm colmap-img

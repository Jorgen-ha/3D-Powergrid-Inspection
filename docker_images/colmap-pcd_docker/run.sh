# Simple script to run the colmap-pcd
podman run --rm --name colmap-pcd --device nvidia.com/gpu=0 \
    -w /working -e DISPLAY=$DISPLAY \
    -v /home/colmap-pcd:/working \
    -v ~/.Xauthority:/root/.Xauthority:rw \
    --net=host -it colmap-pcd:latest

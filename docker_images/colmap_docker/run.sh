# Simple script to run the colmap-img container, with GUI support
podman run --rm --name colmap-img --device nvidia.com/gpu=0 \
    -w /working -e DISPLAY=$DISPLAY \
    -v /home/colmap:/working \
    -v ~/.Xauthority:/root/.Xauthority:rw \
    --net=host -it colmap-modified:latest

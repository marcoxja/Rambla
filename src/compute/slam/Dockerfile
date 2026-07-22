FROM ros:jazzy

# ros-jazzy-image-transport-plugins provides compressed_image_transport,
# which rtabmap_launch's own internal republish_rgb node needs (compressed:=
# true in modal_app.py's _rtabmap_launch_cmd) to decode the JPEG
# /camera/image_raw/compressed topic that observation batches now record.
#
# ros-jazzy-nav2-map-server provides map_saver_cli, run against the live
# /map topic (M5 Phase 1) to produce the Nav2-style map.yaml/image sidecar
# that rtabmap-reprocess -g2's PGM-only export doesn't - see
# map-artifact.md's known-gap paragraph.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-jazzy-rtabmap-ros \
      ros-jazzy-rosbag2 \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-nav2-map-server && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]

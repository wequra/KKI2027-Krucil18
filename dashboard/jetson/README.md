# Jetson GCS media bridge

`gcs_media_bridge_node.py` is a ROS 2 node for the camera-to-GCS path. It
subscribes to ROS image topics and does not open `/dev/video*`. The surface
stream is sent to MediaMTX with GStreamer RTSP; still images are JPEG encoded
and sent to the existing `POST /api/v1/images/` endpoint.

The mission hook is intentionally minimal: a transition on
`/asv/mission_status` with `std_msgs/msg/String` equal to
`SURFACE_IMAGING` or `UNDERWATER_IMAGING` queues the corresponding upload.
The services `/mark_surface_image` and `/mark_underwater_image` are also
available for manual tests. A service call only succeeds when a frame has
already been received on the configured topic.

## Jetson setup

Install the ROS packages for the active ROS distribution and GStreamer
plugins (package names vary slightly by JetPack image):

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-std-msgs ros-$ROS_DISTRO-std-srvs \
  python3-opencv python3-gi gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

On Jetson, ensure the GStreamer installation provides `x264enc` and
`rtspclientsink` (`gst-inspect-1.0 x264enc rtspclientsink`). If the image
uses a hardware H.264 encoder instead, substitute that element in the node's
pipeline while keeping `rtph264pay` and `rtspclientsink`.

Copy `bridge_params.yaml` and replace `<GCS_IP>` with the laptop/server LAN
address. `localhost` is valid only when the bridge and GCS run on the same
machine; it must not be used on the Jetson for a laptop GCS.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
python3 jetson/gcs_media_bridge_node.py \
  --ros-args --params-file jetson/bridge_params.yaml
```

If `/asv/camera_debug` does not exist, change only
`surface_stream_ros_topic` to `/camera/image_raw`. The underwater topic is
deliberately configurable and is never substituted with the surface topic.

## Manual capture tests

```bash
ros2 service call /mark_surface_image std_srvs/srv/Trigger "{}"
ros2 service call /mark_underwater_image std_srvs/srv/Trigger "{}"
```

The response must say that the capture was queued. Then verify the API:

```bash
curl -sS -H 'X-API-Key: test_key' \
  'http://<GCS_IP>:8000/api/v1/images/?image_type=SURFACE&limit=1'
curl -sS -H 'X-API-Key: test_key' \
  'http://<GCS_IP>:8000/api/v1/images/?image_type=UNDERWATER&limit=1'
```

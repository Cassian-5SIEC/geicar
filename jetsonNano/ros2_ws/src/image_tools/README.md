
Image Tools (image_saver)
=========================

This is a small usage guide for the `image_tools` package's `image_saver` node. It explains how to build the workspace, run the node, remap topics (useful for multiple cameras), and common tips for saving images.

Prerequisites
-------------

- ROS 2 installed (the workspace this repo uses — verify your distro, e.g. Foxy, Galactic, Humble).
- A built workspace containing this package (see Build section).
- A camera node publishing sensor_msgs/Image on a ROS 2 topic.

Build
-----

From the root of your ROS 2 workspace (where `src/` lives):

```
colcon build --packages-select image_tools
source install/setup.bash
```

Running image_saver
-------------------

`image_saver` subscribes to a ROS 2 image topic (a `sensor_msgs/Image`) and saves incoming images to disk.

Basic example (remap the topic to the camera topic):

```
ros2 run image_tools image_saver
```

Notes:
- By default images are saved in the current working directory. Run the command from the folder where you want files stored, or move files after saving.
- By default both camera are saved.

Verifying saved images
----------------------

- Saved images are ordinary image files and can be opened with any image viewer.
- If no files appear, ensure the camera node is publishing and that you remapped to the correct topic.

Stopping
--------

Stop the node with Ctrl-C. Files already written are preserved.

Troubleshooting
---------------

- If you see no messages, run `ros2 topic list` to check available topics and ensure the camera is publishing.
- If images are corrupted or empty, check the camera node's encoding (the saver expects a valid image stream) and any transport (compressed topics require a compatible subscriber).

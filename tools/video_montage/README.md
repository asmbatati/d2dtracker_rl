# Interception training montage (Gazebo GUI-style video)

Renders a video of the RL interception policy improving across checkpoints, using
the **offscreen GPU camera** approach (a camera sensor rendered headless on the
GPU via nvidia EGL), decoupled from control.

Why decoupled: on this box the live Gazebo GUI can't be screen-captured (Wayland
-> x11grab is black; the gz screenshot service is ~1 fps), and running a camera
sensor *inside* the control sim starves the flight controller (the policy crashes).
So: record trajectories in the proven no-camera sim, then replay the drone poses
in a separate camera-only world and capture the 30 Hz image stream.

## Pipeline
1. `record_traj.py`  -> per-checkpoint trajectories into `traj/*.npz`
   (runs the real eval, speed_factor 4; policies behave as evaluated).
2. `render_traj.py` + `playback.sdf` -> replays each trajectory in a camera-only
   gz world (2 x500 models teleported along the recorded poses, follow-cam),
   capturing frames into `frames/<clip>/`.
   Needs nvidia EGL: `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`.
3. `assemble.sh` -> ffmpeg per-clip caption (drawtext) + concat -> montage mp4.

Run each via a wrapper that `source /opt/ros/jazzy/setup.bash; source install/setup.bash`
and exports `MONTAGE_DIR` (output root, default `./montage_out`), `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
a private `ROS_DOMAIN_ID`, and `PX4_DIR` (default: the in-tree `drone_interception_ws/PX4-Autopilot`).
`CKPT_DIR` overrides the checkpoint folder (default `checkpoints/intercept`).

The montage built on 23 Jun 2026 is `drone_interception_ws/interception_training_montage.mp4`
(clips in `interception_montage_clips/`); it is not in git.

Camera framing: edit the follow offset in `render_traj.py`
(`cam = mid + np.array([-8,-7,4])`) and FOV in `playback.sdf`.

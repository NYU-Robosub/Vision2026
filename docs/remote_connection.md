# Remote connection to the Jetson from macOS

Use Tailscale and SSH to reach the Jetson, then an SSH-tunneled VNC session to
view RViz while running the existing ZED and RTAB-Map stack.

These instructions assume Tailscale, the VNC server/desktop, ROS 2 Humble, and
the project workspaces are already installed on the Jetson. You need the SSH
credentials and the separate VNC password.

**This workflow starts mapping and saves an RTAB-Map database. It is not the
live-only chair-detection workflow.** Coordinate access to the shared camera
before starting a run.

## Part 1: From your Mac

### 1. Check Tailscale

Make sure Tailscale is connected on your Mac. The menu-bar icon should say
Connected, and the Jetson (`100.71.125.7`) should appear in the device list.

### 2. SSH into the Jetson

Open a Mac Terminal tab and run:

```bash
ssh robosub@100.71.125.7
```

The prompt should become `robosub@ubuntu:~$`.

### 3. Check VNC and existing camera processes

In that Jetson SSH session, run:

```bash
vncserver -list
ps aux | grep -E "zed|rtabmap|rviz" | grep -v grep | cut -c1-120
```

`vncserver -list` should show display `:1`. If no session exists, start it:

```bash
vncserver :1 -localhost yes -geometry 1600x900 -depth 24 -SecurityTypes VncAuth
```

For a fresh mapping run, the process check should show no existing ZED,
RTAB-Map, or RViz processes. If it shows any, check with the person using the
Jetson before continuing. Do not kill another person's session or start a
second camera stack. This process-name check is a quick check, not a guarantee
that no other application holds the camera.

### 4. Open the SSH tunnel

Open a **second Mac Terminal tab** with Cmd+T. Run this on the Mac, not inside
the Jetson SSH session:

```bash
ssh -N -L 5901:localhost:5901 robosub@100.71.125.7
```

After authentication, the command normally prints nothing and remains running.
Leave this tab open while using VNC. If it reports a forwarding or port error,
resolve that before continuing.

### 5. Open the VNC viewer

In Finder, press **Cmd+K**, enter the following address, and click **Connect**:

```text
vnc://localhost:5901
```

Enter the **VNC password** when prompted. If the desktop is a gray background,
right-click it and open a terminal from the menu. If a full desktop appears,
open its terminal application.

## Part 2: In the terminal inside the VNC window

Run the following commands in the **Jetson terminal inside VNC** so RViz opens
on that desktop.

### 6. Load the software

Source the workspaces in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/Vision2026/ros_ws/install/setup.bash
```

### 7. Check the required packages

Each command should print an installed package path:

```bash
ros2 pkg prefix zed_wrapper
ros2 pkg prefix rtabmap_slam
ros2 pkg prefix vision_bringup
```

If a package is not found, stop and check the workspace setup/build before
launching. In this installation, the ZED wrapper is supplied by `~/ros2_ws`.

### 8. Create a folder for test maps

This command is safe to repeat:

```bash
mkdir -p ~/maps_test
```

### 9. Launch mapping

Use a **new database name for each new map**:

```bash
cd ~/Vision2026
bash scripts/run_front.sh mode:=mapping database_path:=/home/robosub/maps_test/NEW_NAME.db
```

Replace `NEW_NAME` with a descriptive name such as `remote_test3`. Reusing an
existing database in mapping mode can extend that map rather than start fresh.

The command starts the camera and RTAB-Map and opens RViz on the VNC desktop
with the repository's layout. Initial startup can take a while. Leave this
terminal running throughout the test.

### 10. Scan the scene

Have the person at the camera sweep it slowly across textured walls. Observe
the accumulated cloud in RViz and check that previously seen surfaces remain
visible as the camera turns away.

## Stopping

### 11. Stop the stack cleanly

Press **Ctrl+C once** in the launch terminal and wait for the prompt to return,
allowing RTAB-Map to save and shut down.

Check the map files:

```bash
ls -la ~/maps_test
```

A database file confirms that a file was written; it does not by itself verify
map quality or successful relocalization.

When finished viewing, disconnect the VNC viewer and press Ctrl+C in the Mac
SSH-tunnel tab. Closing the viewer or tunnel alone does not stop the ROS stack
or the VNC server. Leave shared VNC sessions running unless the team agrees
they should be stopped.

## Foxglove connection

For live visualization without VNC, use the
[Foxglove over Tailscale guide](foxglove_connection.md). It documents installation,
the independent bridge launch, topic selection, and troubleshooting. This VNC
guide remains a fallback for RViz and other Jetson desktop applications.

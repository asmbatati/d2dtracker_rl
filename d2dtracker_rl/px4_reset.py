"""Episode-reset helpers for PX4 SITL in the loop.

PX4 cannot be cheaply restarted per episode (process start + EKF convergence is
slow). The primary strategy teleports both Gazebo model entities to fresh poses
via the Gazebo Harmonic transport ``set_pose`` service and keeps the interceptor
armed in OFFBOARD; a land+rearm path is provided as a fallback.

The teleport is a ``gz service`` call (a runtime side effect, not a file write).
"""
import subprocess
import time


def gz_set_entity_pose(world, entity, x, y, z, timeout=2.0):
    """Teleport a Gazebo model entity. Returns True on apparent success."""
    req = (f'name: "{entity}" '
           f'position {{ x: {x} y: {y} z: {z} }} '
           f'orientation {{ x: 0 y: 0 z: 0 w: 1 }}')
    cmd = [
        'gz', 'service', '-s', f'/world/{world}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', str(int(timeout * 1000)),
        '--req', req,
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout + 1.0)
        return 'true' in out.stdout.lower()
    except (subprocess.SubprocessError, FileNotFoundError):
        return False


def ensure_armed_offboard(ros, hold_xyz, settle_steps=20, dt=0.05):
    """Stream a hold setpoint, request OFFBOARD and arm, then settle.

    ``ros`` is a DroneRosInterface. Returns True if armed+OFFBOARD was reached.
    """
    x, y, z = hold_xyz
    for _ in range(settle_steps):
        ros.publish_position_target(x, y, z)
        st = ros.interceptor_state()
        if st.mode != 'OFFBOARD':
            ros.set_mode('OFFBOARD')
        if not st.armed:
            ros.arm(True)
        time.sleep(dt)
    st = ros.interceptor_state()
    return st.armed and st.mode == 'OFFBOARD'


def reset_teleport(ros, world, interceptor_entity, target_entity,
                   interceptor_xyz, target_xyz, settle_steps=20, dt=0.05):
    """Primary reset: teleport both entities and re-establish OFFBOARD/armed."""
    ok_i = gz_set_entity_pose(world, interceptor_entity, *interceptor_xyz)
    ok_t = gz_set_entity_pose(world, target_entity, *target_xyz)
    armed = ensure_armed_offboard(ros, interceptor_xyz, settle_steps, dt)
    return ok_i and ok_t and armed


def reset_land_rearm(ros, hold_xyz, settle_steps=40, dt=0.05):
    """Fallback reset: LOITER, then re-enter OFFBOARD and arm at the hold pose."""
    ros.set_mode('AUTO.LOITER')
    time.sleep(0.5)
    return ensure_armed_offboard(ros, hold_xyz, settle_steps, dt)

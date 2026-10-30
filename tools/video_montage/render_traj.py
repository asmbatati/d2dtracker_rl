import os, time, subprocess, math, glob
os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'
os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
os.environ.setdefault('GZ_PARTITION', 'aerogym_playback')
import numpy as np, cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

D = os.environ.get('MONTAGE_DIR', 'montage_out')
HERE = os.path.dirname(os.path.abspath(__file__))
PX4 = os.environ.get('PX4_DIR', os.path.join(HERE, '..', '..', '..', '..', '..', 'PX4-Autopilot'))
OUT = D + '/frames'; os.makedirs(OUT, exist_ok=True)
WORLD = 'playback'
ORDER = ['static__sac_intercept_static_20000_steps', 'static__sac_intercept_static_60000_steps',
         'static__sac_intercept_static_100000_steps', 'static__model_static_refine_40k',
         'constant_velocity__model_cv_40k', 'evasive__model_evasive_40k']

srvenv = dict(os.environ)
srvenv.pop('DISPLAY', None)
srvenv['GZ_SIM_RESOURCE_PATH'] = f"{PX4}/Tools/simulation/gz/models"
srv = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v2', os.path.join(HERE, 'playback.sdf')],
                       env=srvenv, stdout=open(D + '/playback.log', 'w'), stderr=subprocess.STDOUT)
time.sleep(10)
node = Node()
latest = {}
def on_img(msg):
    latest['m'] = msg
node.subscribe(Image, 'video_cam', on_img)

def quat_yp(pitch, yaw):
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cp*cy, -sp*sy, sp*cy, cp*sy)

def setpose(name, x, y, z, q):  # q = (w,x,y,z)
    p = Pose(); p.name = name
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = [float(v) for v in q]
    node.request(f'/world/{WORLD}/set_pose', p, Pose, Boolean, 400)

# wait for first frame
for _ in range(50):
    if 'm' in latest: break
    time.sleep(0.2)
print('first frame:', 'm' in latest, flush=True)

manifest = []
for key in ORDER:
    npz = np.load(f'{D}/traj/{key}.npz', allow_pickle=True)
    frames = npz['frames']; caption = str(npz['caption'])
    cdir = f'{OUT}/{key}'; os.system(f'rm -rf {cdir}; mkdir -p {cdir}')
    fi = 0
    for row in frames:
        if not np.all(np.isfinite(row)):
            continue  # episode separator
        ip = row[0:3]; iq = row[3:7]  # x,y,z,w
        tp = row[7:10]; tv = row[10:12]
        # interceptor quat is (x,y,z,w); set_pose wants (w,x,y,z)
        iqw = (iq[3], iq[0], iq[1], iq[2])
        tyaw = math.atan2(tv[1], tv[0]) if (abs(tv[0]) + abs(tv[1])) > 0.05 else 0.0
        setpose('icept', ip[0], ip[1], ip[2], iqw)
        setpose('tgt', tp[0], tp[1], tp[2], quat_yp(0.0, tyaw))
        mid = 0.5 * (ip + tp); mid[2] = max(mid[2], 2.0)
        cam = mid + np.array([-8.0, -7.0, 4.0])
        d = mid - cam
        yaw = math.atan2(d[1], d[0]); pitch = math.atan2(cam[2] - mid[2], math.hypot(d[0], d[1]))
        setpose('video_cam', cam[0], cam[1], cam[2], quat_yp(pitch, yaw))
        time.sleep(0.05)
        m = latest.get('m')
        if m is not None:
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            cv2.imwrite(f'{cdir}/f_{fi:05d}.png', cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)); fi += 1
    manifest.append((key, caption, fi))
    print(f'[{key}] caption="{caption}" frames={fi}', flush=True)

import json
open(f'{OUT}/manifest.json', 'w').write(json.dumps(manifest))
srv.terminate()
try: srv.wait(timeout=5)
except Exception: srv.kill()
print('RENDER_DONE', flush=True)

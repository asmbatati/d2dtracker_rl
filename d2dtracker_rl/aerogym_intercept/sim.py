"""Two-vehicle SimInstance for interception training.

Extends AeroGym-PX4's single-vehicle :class:`SimInstance` with a second PX4
vehicle (the target) sharing the SAME gz world: the interceptor PX4 boots
first and owns the gz server; the target PX4 is spawned after a delay with the
same ``PX4_GZ_WORLD`` so it only adds its model (the proven pattern from
drone_interception_sim). One /clock bridge; two MAVROS stacks (``uav`` for the
interceptor — AeroGym's convention — and ``target``).

Rank mapping: logical worker rank ``r`` occupies PX4 instances ``2r``
(interceptor) and ``2r+1`` (target), so MAVLink ports never collide between
the pair or across workers. Keep ``2r+1 <= 9`` (PX4's instance port map breaks
above that — see AeroGym TRAINING.md).

Interception-specific code lives HERE (d2dtracker_rl); aerogym_px4 stays a
generic single-vehicle playground.
"""
import os
import time

from aerogym_px4.sim.instance import AUTOSTART, MODEL, SimInstance

TARGET_NS = 'target'


class InterceptSimInstance(SimInstance):
    """One isolated interceptor+target stack per training worker."""

    def __init__(self, rank=0, run_id='intercept', target_spawn=(10.0, 0.0, 0.2),
                 **kwargs):
        # logical rank r -> px4 instances 2r / 2r+1
        self.pair_rank = int(rank)
        super().__init__(rank=2 * self.pair_rank, run_id=run_id, **kwargs)
        if 2 * self.pair_rank + 1 > 9:
            raise ValueError('pair rank too high: PX4 instance ports only '
                             'follow the 14540+i convention for i <= 9')
        self.target_instance = 2 * self.pair_rank + 1
        self.target_entity = f'{MODEL}_{self.target_instance}'
        self.target_spawn = tuple(float(v) for v in target_spawn)
        self.target_fcu_url = (f'udp://:{14540 + self.target_instance}'
                               f'@127.0.0.1:{14557 + self.target_instance}')

    def boot(self):
        """Interceptor stack first (owns the gz server), then the target."""
        super().boot()                      # sweep, world, px4(2r), clock, mavros
        time.sleep(8.0)                     # let the gz server settle (spawn-delay
        self._spawn_target_px4()            # pattern from drone_interception_sim)
        time.sleep(4.0)
        self._spawn_target_mavros()
        if not self.alive():
            dead = [k for k, p in self._procs.items() if p.poll() is not None]
            raise RuntimeError(f'InterceptSim[{self.pair_rank}] boot failed: {dead}')

    def _spawn_target_px4(self):
        models = os.path.join(self.px4_dir, 'Tools', 'simulation', 'gz', 'models')
        worlds = os.path.join(self._share, 'worlds')
        x, y, z = self.target_spawn
        cmd = (
            f'cd {self.px4_dir}/build/px4_sitl_default && '
            f'export PX4_GZ_MODELS={models} && '
            f'export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:{models}:{worlds}" && '
            f'HEADLESS=1 '
            f'PX4_SIM_SPEED_FACTOR={self.speed_factor:g} '
            f'PX4_SYS_AUTOSTART={AUTOSTART} '
            f'PX4_GZ_MODEL={MODEL} '
            f'PX4_UXRCE_DDS_NS={TARGET_NS} '
            f"PX4_GZ_MODEL_POSE='{x:g},{y:g},{z:g}' "
            f'PX4_GZ_WORLD={self.world} '
            f'./bin/px4 -i {self.target_instance} -d')
        self._spawn('target_px4', cmd, shell=True)

    def _spawn_target_mavros(self):
        # ns-matched param files (aerogym's wildcards are /uav/**; the target
        # needs /target/** or timesync/conn hardening silently doesn't apply)
        from ament_index_python.packages import get_package_share_directory
        cfg = os.path.join(get_package_share_directory('d2dtracker_rl'),
                           'config', 'mavros')
        self._spawn('target_mavros', [
            'ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
            '-r', f'__ns:=/{TARGET_NS}/mavros',
            '--params-file', os.path.join(cfg, 'target_px4_pluginlists.yaml'),
            '--params-file', os.path.join(cfg, 'target_px4_config.yaml'),
            '-p', f'fcu_url:={self.target_fcu_url}',
            '-p', f'tgt_system:={self.target_instance + 1}',
            '-p', 'tgt_component:=1',
            '-p', 'fcu_protocol:=v2.0',
            '-p', 'use_sim_time:=true'])

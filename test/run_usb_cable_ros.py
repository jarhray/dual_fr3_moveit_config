"""Own a bounded integration launch and clean up only its processes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--viewer', action='store_true')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[3]
    log = Path('/tmp/usb_cable_demo_launch.log')
    command = ['ros2', 'launch', 'dual_fr3_moveit_config', 'usb_cable.launch.py',
        f'maniskill_python:={workspace}/.venv/bin/python',
        f'use_rviz:={str(args.viewer).lower()}', f'maniskill_viewer:={str(args.viewer).lower()}']
    env = dict(os.environ, USB_DEMO_CHECK_VIEWER='1' if args.viewer else '0')
    with log.open('w') as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            checked = subprocess.run([str(workspace / '.venv/bin/python'), str(Path(__file__).with_name('check_usb_cable_ros.py'))],
                                     env=env, timeout=600)
            assert checked.returncode == 0, f'ROS check failed; see {log}'
            assert process.poll() is None, f'Demo exited; see {log}'
            assert 'process has died' not in log.read_text(), f'Launch child failed; see {log}'
            assert 'Error retrieving file' not in log.read_text(), f'RViz could not load the USB mesh; see {log}'
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


if __name__ == '__main__':
    main()

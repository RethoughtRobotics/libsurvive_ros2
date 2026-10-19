"""Shared helpers for reading config/vive_devices.yaml and libsurvive Joy layout."""

import os

import yaml

# Indices follow libsurvive's enum SurviveButton (survive_types.h). The driver
# packs touch bits after the 16 press bits.
BUTTON_MAX = 16


def default_config_path():
    here = os.path.dirname(os.path.realpath(__file__))
    candidates = [os.path.join(here, '..', 'config', 'vive_devices.yaml')]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(os.path.join(get_package_share_directory('libsurvive_ros2'), 'config', 'vive_devices.yaml'))
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return os.path.normpath(path)
    raise FileNotFoundError('vive_devices.yaml not found; pass --config')


def load_vive_config(path=None):
    """Return the ros__parameters dict from vive_devices.yaml."""
    with open(path or default_config_path()) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']

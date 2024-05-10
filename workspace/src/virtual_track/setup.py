#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
        packages=['dr_cvar_module_application',
            'dr_cvar_module_application.dr_cvar_safety_filtering_ros',
            'dr_cvar_module_application.como_tracks',
            'backend',
            'statistics',
            'vehicle_data',
            'tools'],
        package_dir={'': 'src'}
    )

setup(**setup_args)

# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup, find_packages

# Package meta-data.
NAME = 'rl_agent'
REQUIRES_PYTHON = '>=3.5.0'

setup(
    name=NAME,
    version='0.0.1',
    zip_safe=False,
    packages=find_packages(),
    python_requires=REQUIRES_PYTHON,
    install_requires=[
        'boto3>=1.15.3',
        'futures==3.1.1',
        'gym==0.10.5',
        'kubernetes==7.0.0',
        'minio==4.0.5',
        'numpy==1.16.0',
        'pandas==0.22.0',
        'Pillow>=6.2.2',
        'pygame==1.9.3',
        'PyYAML==5.4',
        'redis==2.10.6',
        'rospkg==1.1.7',
        'scipy==0.19.0',
        'tensorflow==2.5.1',
        'rl-coach-slim==0.11.1',
    ],
    entry_points = {
        'console_scripts': [
            'run_rollout_rl_agent=markov.rollout_worker:main',
            'run_local_rl_agent=envs.local_worker:main'
        ],
    }
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vtnf_camera'
submodules = "vtnf_camera/utils"
submodules2 = "vtnf_camera/Img2Depth"
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, submodules2, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wkdo',
    maintainer_email='wkdo@stanford.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub = vtnf_camera.cam_pub:main',
            'dtv2_cam_pub = vtnf_camera.dtv2_cam_pub:main',
        ],
    },
)

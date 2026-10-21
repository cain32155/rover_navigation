import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cac_sim'


def data_files():
    """Install every non-python resource (launch/world/config/urdf/rviz) into share/.

    Walking the dirs instead of hand-listing files means new worlds/configs added
    later are picked up automatically on the next colcon build.
    """
    files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    for folder in ('launch', 'config', 'worlds', 'urdf', 'meshes', 'rviz', 'models', 'tools'):
        for path in glob(os.path.join(folder, '**', '*'), recursive=True):
            if os.path.isfile(path):
                dest = os.path.join('share', package_name, os.path.dirname(path))
                files.append((dest, [path]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='diyasjabin@gmail.com',
    description='Gazebo/ROS2 simulation for the Caterpillar Autonomy Challenge 2027',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_spawner = cac_sim.obstacle_spawner:main',
            'perception_node = cac_sim.perception_node:main',
            'mission_state_machine = cac_sim.mission_state_machine:main',
            'marker_localizer = cac_sim.marker_localizer:main',
            'wall_filter = cac_sim.wall_filter:main',
            'arena_boundary_publisher = cac_sim.arena_boundary_publisher:main',
            'crater_detector = cac_sim.crater_detector:main',
            'material_visualizer = cac_sim.material_visualizer:main',
        ],
    },
)

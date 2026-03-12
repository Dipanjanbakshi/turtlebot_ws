from setuptools import find_packages, setup

package_name = 'mechabot_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dipanjan',
    maintainer_email='dipanjanbakshi998@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'read_lidar = mechabot_scripts.read_lidar:main',
            'read_imu = mechabot_scripts.read_imu:main',
            'read_camera = mechabot_scripts.read_camera:main',
            'object_avoidance = mechabot_scripts.object_avoidance:main',
            'wall_follower = mechabot_scripts.wall_follower:main',
            'maze_solver = mechabot_scripts.maze_solver:main',
            'detect_marker = mechabot_scripts.detect_marker:main',
            'auto_docking_undocking = mechabot_scripts.auto_docking_undocking:main',
            'docking_with_patrolling = mechabot_scripts.docking_with_patrolling:main',

        ],
    },
)

from setuptools import setup

package_name = 'py_waypoint_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sethmess',
    maintainer_email='sethmess@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_logger = py_waypoint_logger.waypoint_logger:main',
            'waypoint_visualizer = py_waypoint_logger.waypoint_visualizer:main',
        ],
    },
)

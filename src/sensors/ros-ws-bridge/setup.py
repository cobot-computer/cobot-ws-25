from setuptools import find_packages, setup

package_name = 'ros_ws_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/hook', [
            'hook/ament_prefix_path.dsv',
            'hook/ament_prefix_path.sh',
            'hook/ament_prefix_path.ps1',
        ]),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ros_ws_bridge = ros_ws_bridge.bridge:main',
        ],
    },
)

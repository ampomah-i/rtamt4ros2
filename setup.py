from setuptools import setup, find_packages

package_name = 'rtamt4ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'README.md']),
        ('share/' + package_name + '/launch', ['rtamt4ros2/launch/stl_monitor.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'rtamt'],
    zip_safe=True,
    maintainer='Immanuel Ampomah Mensah',
    maintainer_email='ia324@cornell.edu',
    description='RTAMT-based runtime monitoring tools for ROS 2 systems',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stl_monitor_node = rtamt4ros2.monitor_node:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'opencv_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['opencv_ros', 'opencv_ros.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'orange = opencv_ros.orange:main',  # Example entry point
            'coffee = opencv_ros.coffee:main',  # Coffee entry point
        ],
    },
    install_requires=['setuptools', 'opencv-python', 'cv_bridge', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='CYH',
    description='TDK camera opencv example',
)
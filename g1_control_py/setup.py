from setuptools import setup

package_name = 'g1_control_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam Choy',
    maintainer_email='sam@aukilabs.com',
    description='ROS 2 package to subscribe to /cmd_vel topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_translator = g1_control_py.cmd_vel_translator:main',
            'lowstate_transltor = g1_control_py.lowstate_translator:main'
        ],
    },
)
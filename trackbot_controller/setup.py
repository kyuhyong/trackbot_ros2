from setuptools import find_packages, setup

package_name = 'trackbot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_keyboard.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_keyboard.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kyuhyong',
    maintainer_email='kyuhyong@gmail.com',
    description='Keyboard teleop launcher for Trackbot (/cmd_vel).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = trackbot_controller.teleop_keyboard:main',
        ],
    },
)

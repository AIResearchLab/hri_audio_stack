from setuptools import find_packages, setup

package_name = 'my_robot_voice'

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
    maintainer='buddhi',
    maintainer_email='buddhigamage217@gmail.com',
    description='Voice listener using ROS 2 and threading',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_listener_action_server = my_robot_voice.voice_listener_action_server:main',
            'voice_command_client = my_robot_voice.voice_command_client:main',
        ],
    },
)

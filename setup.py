from setuptools import find_packages, setup

package_name = 'hri_audio_stack'

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
            'voice_listener_action_server = hri_audio_stack.voice_listener_action_server:main',
            'voice_listener_client = hri_audio_stack.voice_listener_client:main',
            'voice_speaker_action_server = hri_audio_stack.voice_speaker_action_server:main',
            'voice_speaker_client = hri_audio_stack.voice_speaker_client:main',
        ],
    },
)

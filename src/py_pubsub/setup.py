from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='riser14',
    maintainer_email='gvnkha001@myuct.ac.za',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_mpc = py_pubsub.nmpc_0:main',
            'control_nmpc = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'waypoint follow = py_pubsub.waypoint_follower:main',
            'pid = py_pubsub.pid_control:main',
            'pose_sub = py_pubsub.pose_subscriber:main',
            'capture = py_pubsub.camera_capture:main',
            'compare_mean_error = py_pubsub.compare_pictures:main',
            'compare_feature = py_pubsub.compare_images_orb:main',
            'drift_measure = py_pubsub.drift_measure:main',
            'full_record = py_pubsub.full_record:main',
            'camera_record = py_pubsub.camera_recording:main',
            'visualisation = py_pubsub.path_visual:main',
        ],
    },
)

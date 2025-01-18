from setuptools import find_packages, setup

package_name = 'my_tf'

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
    maintainer='pw',
    maintainer_email='pw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_tf_1= my_tf.my_tf_1:main',
            'my_tf_2 = my_tf.my_tf_2:main',
            'child_frame = my_tf.child_frame:main',
            'frame_trail_publisher_marker = my_tf.frame_trail_publisher_marker:main',
            'frame_trail_publisher_nav = my_tf.frame_trail_publisher_nav:main',
            'distance_world_child_publisher = my_tf.distance_world_child_publisher:main',
            'frame_trail_publisher_marker_2 = my_tf.frame_trail_publisher_marker_2:main',
        ],
    },
)

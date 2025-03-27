from setuptools import find_packages, setup

package_name = 'controller_tutorials'

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
            'simple_rotate = controller_tutorials.simple_rotate:main',
            'control_rotate = controller_tutorials.control_rotate:main',
            'move_turtle = controller_tutorials.move_turtle:main',
            'move_turtle_state_machine = controller_tutorials.move_turtle_state_machine:main',
            'move_turtle_behavior_tree = controller_tutorials.move_turtle_behavior_tree:main',
        ],
    },
)

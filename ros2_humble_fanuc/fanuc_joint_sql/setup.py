from setuptools import find_packages, setup

package_name = 'fanuc_joint_sql'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'joint_states_sql = fanuc_joint_sql.joint_states_listener_sql:main',
		'moveit_goal_setting = fanuc_joint_sql.moveit_goal_setting:main',
		'moveit_fram_to_world = fanuc_joint_sql.moveit_fram_to_world:main',
		'joint_gui_pub = fanuc_joint_sql.joint_gui_pub:main',
        ],
    },
)

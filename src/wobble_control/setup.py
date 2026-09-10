from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wobble_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vish',
    maintainer_email='vish@todo.todo',
    description='Cascaded PID balance and 4-bar posture controllers for Wobble robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'balance_controller = wobble_control.balance_controller:main',
            'course_navigator = wobble_control.course_navigator:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'ar4_teleop_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Эта строка говорит системе установить все .launch.py файлы из папки launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slaik',
    maintainer_email='slaik@todo.todo',
    description='AR4 Teleop GUI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_gui = ar4_teleop_gui.teleop_gui:main',
        ],
    },
)

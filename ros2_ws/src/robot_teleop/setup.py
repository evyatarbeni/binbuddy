from setuptools import setup, find_packages

setup(
    name='robot_teleop',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_teleop']),
        ('share/robot_teleop', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='Custom keyboard teleop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'binbuddy_teleop = robot_teleop.binbuddy_teleop:main',
        ],
    },
)

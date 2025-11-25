from setuptools import setup, find_packages

package_name = 'crane_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='Crane position control with action server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crane_controller = crane_control.crane_controller:main',
        ],
    },
)

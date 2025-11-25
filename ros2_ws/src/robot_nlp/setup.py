from setuptools import setup, find_packages

setup(
    name='robot_nlp',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_nlp']),
        ('share/robot_nlp', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evyatar Beni',
    maintainer_email='evyatarbeni@gmail.com',
    description='NLP command interface',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nlp_commander = robot_nlp.nlp_commander:main',
        ],
    },
)

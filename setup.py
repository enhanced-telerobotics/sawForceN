from setuptools import setup


setup(
    name='force_n',
    version='0.1.0',
    packages=['saw_force_n'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/force_n']),
        ('share/force_n', ['package.xml', 'README.md']),
        ('share/force_n/launch', ['launch/force_publisher.launch.py',
                                 'launch/force_predictor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'force_sensor = saw_force_n.sensor:main',
        'force_publisher = saw_force_n.manager:main',
        'force_predictor = saw_force_n.predict:main',
    ]},
)

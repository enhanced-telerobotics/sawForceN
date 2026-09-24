from setuptools import setup
setup(name='force_n', version='0.1.0', packages=['saw_force_n'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/force_n']),
                  ('share/force_n', ['package.xml', 'README.md']),
                  ('share/force_n/launch', ['launch/single_arm.launch.py'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['force_n = saw_force_n.node:main']})

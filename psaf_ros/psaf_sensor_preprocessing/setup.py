from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['psaf_sensor_preprocessing'],
    package_dir={'': 'src/python'}
)
setup(**d)
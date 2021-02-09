from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['psaf_abstraction_layer'],
    package_dir={'': 'src/python'}
)
setup(**d)
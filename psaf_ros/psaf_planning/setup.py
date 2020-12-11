from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['psaf_planning','SMP'],
    package_dir={'': 'src',
                 'SMP': 'external/commonroad-search/SMP'}
)
setup(**d)
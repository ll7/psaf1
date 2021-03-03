from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['psaf_global_planner', 'SMP', 'opendrive2lanelet'],
    package_dir={'': 'src',
                 'SMP': 'external/commonroad-search/SMP',
                 'opendrive2lanelet': 'external/opendrive2lanelet/opendrive2lanelet'
                 }
)
setup(**d)
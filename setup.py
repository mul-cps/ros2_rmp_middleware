from setuptools import setup
from Cython.Build import cythonize

package_name = 'rmp220_middleware'

files = package_name + "/*.py"

setup(
    #ext_modules=cythonize(files,compiler_directives={'language_level' : "3"},force=True,quiet=True),
    ext_modules = cythonize(files,force=True,quiet=True),
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', "wheel",  "Cython"],
    zip_safe=True,
    maintainer='bjorn',
    maintainer_email='bjoern.ellensohn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmp220_middleware = rmp220_middleware.rmp220_middleware:main'
        ],
    },
)

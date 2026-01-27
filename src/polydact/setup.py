from pathlib import Path

from setuptools import find_packages, setup

def recursive_files(prefix, path):
    """
    Recurse over path returning a list of tuples suitable for use with setuptools data_files.

    :param prefix: prefix path to prepend to the path
    :param path: Path to directory to recurse. Path should not have a trailing '/'
    :return: List of tuples. 1st element of each tuple is destination path, 2nd element is a list
             of files to copy to that path
    """
    return [(str(Path(prefix)/subdir),
            [str(file) for file in subdir.glob('*') if not file.is_dir()])
            for subdir in Path(path).glob('**')]

package_name = 'polydact'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *recursive_files('share/' + package_name, 'launch'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miguel Pegues',
    maintainer_email='MiguelP@u.northwestern.edu',
    description='This package is for the control of a supernumerary finger prosthetic.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

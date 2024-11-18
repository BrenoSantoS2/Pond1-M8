from setuptools import find_packages, setup

package_name = 'navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brenosantos',
    maintainer_email='brenosantos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remote_control = navigation_pkg.remote_control:main',
            'reactive_navigation = navigation_pkg.reactive_navigation:main',
            'map_navigation = navigation_pkg.map_navigation:main',
        ],
    },
)

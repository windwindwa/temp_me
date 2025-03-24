from setuptools import setup, find_packages

package_name = 'mycobot_control_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='MyCobot 280 slow motion trajectory sender',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slow_move_node = mycobot_control_pkg.slow_move_node:main',
        ],
    },
)

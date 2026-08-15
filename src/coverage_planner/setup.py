from setuptools import setup

package_name = 'coverage_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmad',
    maintainer_email='ahmad@example.com',
    description='Boustrophedon single-agent coverage path planner',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner_node = coverage_planner.coverage_planner_node:main',
            'bcd_coverage_node = coverage_planner.bcd_coverage_node:main',
            'bcd_geometric_node = coverage_planner.bcd_geometric_planner:main'
        ],
    },
)

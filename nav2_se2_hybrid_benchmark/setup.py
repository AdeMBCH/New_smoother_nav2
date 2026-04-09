from setuptools import setup

package_name = 'nav2_se2_hybrid_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/goals_tb3_world.yaml']),
        ('share/' + package_name + '/launch', ['launch/benchmark_tb3.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nav2 Dev',
    maintainer_email='dev@example.com',
    description='Benchmark harness for comparing Nav2 smoothing variants.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'run_benchmark = nav2_se2_hybrid_benchmark.run_benchmark:main',
        ],
    },
)

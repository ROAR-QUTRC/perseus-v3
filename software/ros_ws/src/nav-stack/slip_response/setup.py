from setuptools import find_packages, setup

package_name = 'slip_response'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omar Kassab',
    maintainer_email='Okassab2003@gmail.com',
    description='Anti-slip response decision prototype for the mobility watchdog',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'response_node = slip_response.response_node:main',
        ],
    },
)

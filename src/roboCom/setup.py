from setuptools import setup

package_name = 'roboCom'

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
    maintainer='Dyllon Dunton',
    maintainer_email='duntondyllon@gmail.com',
    description='Lunabotics 2023 RoverBot Communications and GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'controller = roboCom.controllerCom:main',
        	'robot = roboCom.robotCom:main',
        ],
    },
)

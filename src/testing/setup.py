from setuptools import find_packages, setup

package_name = 'testing'

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
    maintainer='davidnie',
    maintainer_email='davidnie0418@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_test = testing.vision_test:main',
            'brain_vision_test = testing.brain_vision_test:main',
            'end_effector_test = testing.end_effector_test:main',
            'brain_routine_test = testing.brain_routine_test:main',
            'ooi_test = testing.ooi_test:main',
        ],
    },
)

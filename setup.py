from setuptools import setup

package_name = 'ros2_unification_2019'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'src.hecu_unidriver',
        'src.recu_unidriver',
	'src.aecu_unidriver'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Endre Eros',
    author_email='endre@todo.com',
    maintainer='Endre Eros',
    maintainer_email='endre@todo.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 part for unification 2019',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hecu_unidriver = src.hecu_unidriver:hecu_unidriver',
            'recu_unidriver = src.recu_unidriver:recu_unidriver',
	    'aecu_unidriver = src.aecu_unidriver:aecu_unidriver'
        ],
    },
)

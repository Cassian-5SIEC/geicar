from setuptools import find_packages, setup

package_name = 'my_image_tools'

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
    maintainer='Thomas Chourret',
    maintainer_email='chourret.thomas@gmail.com',
    description='Image Tools Packages',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'saver_node = my_image_tools.dual_image_saver:main',
        ],
    },
)

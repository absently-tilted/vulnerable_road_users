from setuptools import find_packages, setup

package_name = 'vulnerable_road_users'

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
    maintainer='charles',
    maintainer_email='charraff@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'img_publisher = vulnerable_road_users.webcam_pub:main',
             'img_subscriber = vulnerable_road_users.webcam_sub:main',
             'alert_sub = vulnerable_road_users.alert_sub:main'
        ],
    },
)

from setuptools import setup

package_name = 'inventory_management'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petern25',
    maintainer_email='petern25@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_publisher = inventory_management.image_publisher:main",
            "image_subscriber = inventory_management.image_subscriber:main",
            "serial_transcriber = inventory_management.serial_transcriber:main",
            "serial_imu = inventory_management.serial_imu:main"
        ],
    },
)

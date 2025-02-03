from setuptools import find_packages, setup

package_name = 'yolo_image'

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
    maintainer='root',
    maintainer_email='jyhh1992@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'compressed_image_pub = yolo_image.compressed_image_pub:main',
                'compressed_image_sub = yolo_image.compressed_image_sub:main',
                'yolo_detect = yolo_image.yolo_detect:main',
                'yolo_image_sub = yolo_image.yolo_image_sub:main',
        ],
    },
)

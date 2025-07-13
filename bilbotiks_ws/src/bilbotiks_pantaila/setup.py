from setuptools import find_packages, setup

package_name = 'bilbotiks_pantaila'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={
        # Incluir todos los archivos dentro de la carpeta aurpegia_irudiak
        'bilbotiks_pantaila': ['aurpegia_irudiak/*'],
    },
    install_requires=['setuptools', 'rclpy', 'pygame', 'cv_bridge', 'cv2', 'numpy'],
    zip_safe=True,
    maintainer='bilbotiks',
    maintainer_email='javierarambarricalvo@gmail.com',
    description='Roverraren pantailak kudeatzeko paketea',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aurpegia = bilbotiks_pantaila.aurpegia:main',
            'kamera = bilbotiks_pantaila.kamera:main',
            'pertzepzioa = bilbotiks_pantaila.pertzepzioa:main',
            'kontrola = bilbotiks_pantaila.kontrola:main',
        ],
    },
)

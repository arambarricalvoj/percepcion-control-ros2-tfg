from setuptools import find_packages, setup

package_name = 'bilbotiks_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/bilbotiks_controller/config', ['config/params.yaml', 'config/modelo_100epochs.keras']),
        ('share/bilbotiks_controller/launch', ['launch/bilbotiks.launch.py', 'launch/kontrola.py', 'launch/gidatzea.py']),
        ('share/bilbotiks_controller/launch', ['launch/pertzepzioa.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'sensor_msgs', 'adafruit-circuitpython-servokit', 'adafruit-circuitpython-bno055', 'ydlidar', 'cv2', 'cv_bridge', 'tf2_ros'],
    zip_safe=True,
    maintainer='bilbotiks',
    maintainer_email='javierarambarricalvo@gmail.com',
    description='Roverra kontrolatzeko paketea',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motorrak_roboclaw = bilbotiks_controller.motorrak_roboclaw:main',
            'servoak = bilbotiks_controller.servoak:main',
            'imu = bilbotiks_controller.imu:main',
            'lidar = bilbotiks_controller.lidar:main',
            'kamera = bilbotiks_controller.kamera:main',
            'kamera_sub = bilbotiks_controller.kamera_sub:main',
            'pertzepzio_proba_wrapper = bilbotiks_controller.pertzepzio_proba_wrapper:main',
            'pertzepzio_proba = bilbotiks_controller.pertzepzio_proba:main',
            'kontrol_proba_wrapper = bilbotiks_controller.kontrol_proba_wrapper:main',
            'gidatze_proba_wrapper = bilbotiks_controller.gidatze_proba_wrapper:main'
        ],
    },
)

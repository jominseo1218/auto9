from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rviz_visualization'

setup(
    name=package_name,
    version='0.9.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # 런치 파일이 ../config/integrated_view.rviz 를 상대경로로 찾으므로
        # 두 디렉터리를 share/<패키지명> 아래 같은 구조로 설치한다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='조민서',
    maintainer_email='jom23760@gmail.com',
    description='9주차 RViz2 심화 시각화와 커스텀 마커 파이프라인',
    license='MIT',
    entry_points={
        'console_scripts': [
            # 실행 파일 이름을 .py 까지 포함해 등록한다.
            # 제공된 rviz_advanced.launch.py 가 executable='custom_marker_publisher.py'
            # 로 지정하고 있어 런치 파일을 수정하지 않고 그대로 쓰기 위함이다.
            'custom_marker_publisher.py = '
            'rviz_visualization.custom_marker_publisher:main',
            # 축약형도 함께 등록 (ros2 run 으로 직접 띄울 때 편의)
            'custom_marker_publisher = '
            'rviz_visualization.custom_marker_publisher:main',
        ],
    },
)

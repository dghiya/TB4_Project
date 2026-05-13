from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_gemini_er'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        # Gemini Robotics-ER 1.6 client. NOT 'google-generativeai' (different
        # SDK / different API surface).
        'google-genai',
    ],
    zip_safe=True,
    maintainer='Fazil Khan',
    maintainer_email='fazhara1@jh.edu',
    description='Gemini Robotics-ER perception node — phase 1 debug TF publisher',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gemini_body_tf_node = ros2_gemini_er.gemini_body_tf_node:main',
        ],
    },
)

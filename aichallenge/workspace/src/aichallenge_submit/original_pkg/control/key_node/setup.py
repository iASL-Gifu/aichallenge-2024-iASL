from setuptools import setup

package_name = 'key_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'pyautogui',  # pyautoguiを依存関係として指定
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS 2 package for simulating keyboard input using pyautogui.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_simulator = key_node.key_node:main',
        ],
    },
)

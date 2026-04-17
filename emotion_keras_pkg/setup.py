from setuptools import find_packages, setup

package_name = 'emotion_keras_pkg'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emotion_keras = emotion_keras_pkg.emotion_keras:main'
            'emotion_out_tiago = emotion_keras_pkg.emotion_output_tiago:main'
        ],
    },
)

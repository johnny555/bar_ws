from setuptools import find_packages, setup
import os 

package_name = 'jupyter-scaffold'

# Function to read the requirements.txt file
def read_requirements():
    requirements_path = os.path.join(os.path.dirname(__file__), 'requirements.txt')
    with open(requirements_path, 'r') as f:
        requirements = f.read().splitlines()
    return requirements

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        ('share/' + package_name+ '/launch', ['launch/start_jupyter.launch.py']),
        
    ],
    install_requires=read_requirements(),
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='badwolf.johnnyv@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

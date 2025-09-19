from setuptools import find_packages
from setuptools import setup

package_name = 'cabot_ui_plugins'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    zip_safe=True,
    author='Daisuke Sato',
    author_email='daisukes@cmu.edu',
    maintainer='Daisuke Sato',
    maintainer_email='daisukes@cmu.edu',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description='cabot_ui plugins',
    long_description='',
    license='MIT',
    entry_points={
        'cabot_ui.plugins': [
            'navigation = cabot_ui_plugins.navigation:Navigation',
            'feature = cabot_ui_plugins.feature:Feature',
            'description = cabot_ui_plugins.description:Description',
            'speaker = cabot_ui_plugins.speaker:Speaker',
        ],
        'navcog_map.plugins': [
            'navigation = cabot_ui_plugins.navigation:NavigationMenu',
        ],
    }
)

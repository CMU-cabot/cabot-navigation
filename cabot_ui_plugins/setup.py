from setuptools import find_packages
from setuptools import setup

package_name = 'cabot_ui_plugins'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['tests']),
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
    tests_require=['pytest'],
    entry_points={
        'cabot_ui.plugins': [
            'navigation = cabot_ui_plugins.navigation:Navigation',
            'feature = cabot_ui_plugins.feature:Feature',
            'description = cabot_ui_plugins.description:Description',
            'speaker = cabot_ui_plugins.speaker:Speaker',
            'phone = cabot_ui_plugins.phone_navigation.phone_navigation:PhoneNavigation',
        ],
        'event_mapper.plugins': [
            'event_mapper1 = cabot_ui_plugins.event_mapper:EventMapper1',
            'event_mapper2 = cabot_ui_plugins.event_mapper:EventMapper2',
        ],
        'navcog_map.plugins': [
            'navigation = cabot_ui_plugins.navigation:NavigationMenu',
            'phone = cabot_ui_plugins.phone_navigation.menu:PhoneMenu',
        ],
    }
)

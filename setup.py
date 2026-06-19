import os
from glob import glob
from setuptools import setup

package_name = 'campus_delivery_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dahyun',
    maintainer_email='dahyunkwak99@gmail.com',
    description='Campus delivery robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'train_ppo = campus_delivery_robot.train_ppo:main',
        'campus_env = campus_delivery_robot.campus_env:main',
        'gt_pose_pub = campus_delivery_robot.gt_pose_publisher:main', 
        'eval_rl = campus_delivery_robot.eval:main',
        'eval_rl_v2 = campus_delivery_robot.eval_v2:main',
        'eval_rl_v3 = campus_delivery_robot.eval_v3:main',
        'eval_rule = campus_delivery_robot.rule_based_eval:main',
        'eval_rule_v2 = campus_delivery_robot.rule_based_eval_v2:main', 
        'eval_rule_v3 = campus_delivery_robot.rule_based_eval_v3:main',
        'train_ppo_v2 = campus_delivery_robot.train_ppo_v2:main',
        'train_ppo_v3 = campus_delivery_robot.train_ppo_v3:main',
    ],
},
)

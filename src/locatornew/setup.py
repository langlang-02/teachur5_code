# setup.py：ROS1 Python 包的标准配置文件
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 调用 ROS 工具函数，生成 distutils 配置（适配 ROS 编译系统）
setup_args = generate_distutils_setup(
    # 1. 配置包名：必须与 package.xml 中的 <name> 标签一致（这里是 locatornew）
    packages=['locatornew'],
    # 2. 配置 Python 模块所在目录：你的 .py 脚本都在 scripts 文件夹下
    package_dir={'': 'scripts'},
    # 3. 配置可执行脚本：列出所有需要编译为可执行文件的 .py 脚本
    scripts=[
        'scripts/location_service.py',
        'scripts/analyze.py',
        'scripts/calculate_rotate.py',
        'scripts/matrix_utils.py',
        'scripts/robot_controller.py',
    ]
)

# 执行 setup，完成 Python 模块配置
setup(** setup_args)
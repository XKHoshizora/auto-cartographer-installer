#!/bin/bash

# 检测系统语言
LANGUAGE=$(echo $LANG | cut -d_ -f1)

# 定义消息，中文环境下输出中文，其他情况输出英文
if [ "$LANGUAGE" == "zh" ]; then
    MSG_UPDATE="更新系统包"
    MSG_INSTALL_TOOLS="安装必要工具"
    MSG_CREATE_WS="创建 cartographer_ws 工作区"
    MSG_INIT_WS="初始化并合并 cartographer_ros 依赖"
    MSG_INIT_ROSDEP="初始化 rosdep 并更新"
    MSG_IGNORE_LIBABSL="注释掉 libabsl-dev 依赖"
    MSG_INSTALL_DEPS="安装依赖项"
    MSG_INSTALL_ABSEIL="手动安装 abseil-cpp"
    MSG_REMOVE_ABSEIL="卸载 ROS 提供的 abseil-cpp 以防止冲突"
    MSG_REMOVE_MACROS="删除线程注解宏"
    MSG_BUILD="构建并安装"
    MSG_ADD_BASHRC="添加 source 命令到 ~/.bashrc"
    MSG_DONE="Cartographer ROS 编译完成，并已更新 ~/.bashrc。请重新打开终端，或手动执行 'source ~/.bashrc' 以使更改生效。"
else
    MSG_UPDATE="Updating system packages"
    MSG_INSTALL_TOOLS="Installing necessary tools"
    MSG_CREATE_WS="Creating cartographer_ws workspace"
    MSG_INIT_WS="Initializing and merging cartographer_ros dependencies"
    MSG_INIT_ROSDEP="Initializing rosdep and updating"
    MSG_IGNORE_LIBABSL="Commenting out libabsl-dev dependencies"
    MSG_INSTALL_DEPS="Installing dependencies"
    MSG_INSTALL_ABSEIL="Manually installing abseil-cpp"
    MSG_REMOVE_ABSEIL="Removing ROS-provided abseil-cpp to avoid conflicts"
    MSG_REMOVE_MACROS="Removing thread annotation macros"
    MSG_BUILD="Building and installing"
    MSG_ADD_BASHRC="Adding source command to ~/.bashrc"
    MSG_DONE="Cartographer ROS compilation complete, and ~/.bashrc updated. Please reopen your terminal or manually run 'source ~/.bashrc' to apply the changes."
fi

# 开始执行
echo $MSG_UPDATE
sudo apt-get update

echo $MSG_INSTALL_TOOLS
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

echo $MSG_CREATE_WS
mkdir -p ~/cartographer_ws/src
cd ~/cartographer_ws

echo $MSG_INIT_WS
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

echo $MSG_INIT_ROSDEP
sudo rosdep init || echo "rosdep 已经初始化，继续..." # 此处保持输出中文或英文都可理解
rosdep update

echo $MSG_IGNORE_LIBABSL
sed -i 's/<build_depend>libabsl-dev<\/build_depend>/<!-- <build_depend>libabsl-dev<\/build_depend> -->/' src/cartographer/package.xml
sed -i 's/<exec_depend>libabsl-dev<\/exec_depend>/<!-- <exec_depend>libabsl-dev<\/exec_depend> -->/' src/cartographer/package.xml

echo $MSG_INSTALL_DEPS
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO}

echo $MSG_INSTALL_ABSEIL
src/cartographer/scripts/install_abseil.sh

echo $MSG_REMOVE_ABSEIL
sudo apt-get remove -y ros-${ROS_DISTRO}-abseil-cpp

echo $MSG_REMOVE_MACROS
find ./src/cartographer -type f -name "*.h" -o -name "*.cc" | xargs sed -i 's/LOCKS_EXCLUDED([^)]*)//g; s/GUARDED_BY([^)]*)//g; s/EXCLUSIVE_LOCKS_REQUIRED([^)]*)//g'
find ./src/cartographer_ros -type f -name "*.h" -o -name "*.cc" | xargs sed -i 's/GUARDED_BY([^)]*)//g; s/LOCKS_EXCLUDED([^)]*)//g; s/EXCLUSIVE_LOCKS_REQUIRED([^)]*)//g'

echo $MSG_BUILD
catkin_make_isolated --install --use-ninja

# 检查是否已经在 .bashrc 中添加了 source 命令
if ! grep -Fxq "source ~/cartographer_ws/install_isolated/setup.bash" ~/.bashrc
then
    # 如果没有添加，则添加
    echo $MSG_ADD_BASHRC
    echo "source ~/cartographer_ws/install_isolated/setup.bash" >> ~/.bashrc
fi

# 立即刷新终端
source ~/.bashrc

echo $MSG_DONE

#!/bin/bash

HOME_DIR=/home/$USER/
CURRENT_DIR=$(pwd)

# red color
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_END="\033[0m"

echo
echo
echo
echo -e "$COLOR_RED ------------------------------------------------------------------------ $COLOR_END"
echo -e "$COLOR_RED |                  | Type package install directory |                  | $COLOR_END"
echo -e "$COLOR_RED ------------------------------------------------------------------------ $COLOR_END"
echo " ex) /home/jy or /home/jy/Library ..."
read INSTALL_DIR
echo "Check install directory : $INSTALL_DIR"
echo "."
echo "."
echo -e "$COLOR_RED ------------------------------------------------------------------------- $COLOR_END"
echo -e "$COLOR_RED |                   | Type Raisim install directory |                   | $COLOR_END"
echo -e "$COLOR_RED ------------------------------------------------------------------------- $COLOR_END"
echo "ex) /home/jy/raisimLib/install or /home/wj/Library/raisimLib/build/install"
read RAI_INSTALL_DIR

echo -e "$COLOR_GREEN --------------------- $COLOR_END"
echo -e "$COLOR_GREEN |   INSTALL 00/14   | $COLOR_END"
echo -e "$COLOR_GREEN |  start installing | $COLOR_END"
echo -e "$COLOR_GREEN --------------------- $COLOR_END"

sudo apt-get update -y
sudo apt-get upgrade -y

echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
echo -e "$COLOR_GREEN |    INSTALL 03/14    | $COLOR_END"
echo -e "$COLOR_GREEN |        Cmake        | $COLOR_END"
echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
cd $INSTALL_DIR
sudo apt-get install libssl-dev -y
wget https://github.com/Kitware/CMake/releases/download/v3.21.0/cmake-3.21.0.tar.gz
tar -xvf cmake-3.21.0.tar.gz
cd cmake-3.21.0
./bootstrap && make && sudo make install
cd .. && sudo rm -rf cmake-3.21.0 && sudo rm -rf cmake-3.21.0.tar.gz

echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
echo -e "$COLOR_GREEN |    INSTALL 05/14    | $COLOR_END"
echo -e "$COLOR_GREEN |        CLion        | $COLOR_END"
echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
sudo snap install clion --classic

echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
echo -e "$COLOR_GREEN |    INSTALL 10/14    | $COLOR_END"
echo -e "$COLOR_GREEN | simplescreenrecorder| $COLOR_END"
echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
sudo apt-get install simplescreenrecorder -y

echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
echo -e "$COLOR_GREEN |    INSTALL 08/14    | $COLOR_END"
echo -e "$COLOR_GREEN |       Python3       | $COLOR_END"
echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
sudo apt-get install python3 python3-pip -y

echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
echo -e "$COLOR_GREEN |    INSTALL 11/14    | $COLOR_END"
echo -e "$COLOR_GREEN |        Eigen        | $COLOR_END"
echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
sudo apt-get install libeigen3-dev

echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
echo -e "$COLOR_GREEN |    INSTALL 14/14    | $COLOR_END"
echo -e "$COLOR_GREEN |        Raisim       | $COLOR_END"
echo -e "$COLOR_GREEN ----------------------- $COLOR_END"
cd $INSTALL_DIR
git clone https://github.com/raisimTech/raisimLib.git
cd raisimLib
mkdir build
mkdir install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$RAI_INSTALL_DIR -DRAISIM_EXAMPLE=ON
make install -j4
sudo apt-get install minizip ffmpeg -y
sudo apt-get install vulkan-utils -y
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$INSTALL_DIR/raisimLib/raisim/linux/lib" >> ~/.bashrc
echo "export PYTHONPATH=$PYTHONPATH:$INSTALL_DIR/raisimLib/raisim/linux/lib" >> ~/.bashrc
echo " --------------------------------------------------------------------------------------------------- "
echo " |     For link raisim, cmake option : -DCMAKE_PREFIX_PATH=$INSTALL_DIR/raisimLib/raisim/linux     | "
echo " --------------------------------------------------------------------------------------------------- "
echo
echo
echo -e "$COLOR_GREEN --------------------- $COLOR_END"
echo -e "$COLOR_GREEN |   INSTALL 14/14   | $COLOR_END"
echo -e "$COLOR_GREEN |     installed     | $COLOR_END"
echo -e "$COLOR_GREEN --------------------- $COLOR_END"

echo -e "$COLOR_GREEN -------------- $COLOR_END"
echo -e "$COLOR_GREEN | setup bash | $COLOR_END"
echo -e "$COLOR_GREEN -------------- $COLOR_END"
echo "alias gb='gedit ~/.bashrc'" >> ~/.bashrc
echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
source ~/.bashrc

cd $CURRENT_DIR
cd ..
sudo rm -rf initial-setup

echo -e "$COLOR_RED NEED TO REBOOT ! $COLOR_END"
echo -e "$COLOR_RED NEED TO REBOOT ! $COLOR_END"
echo -e "$COLOR_RED NEED TO REBOOT ! $COLOR_END"
echo -e "$COLOR_RED NEED TO REBOOT ! $COLOR_END"
echo -e "$COLOR_RED NEED TO REBOOT ! $COLOR_END"

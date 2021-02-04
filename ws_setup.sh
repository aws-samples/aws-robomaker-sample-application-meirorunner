#!/bin/bash

echo "###############################################################################"
echo "Workshop environment setup starting.."
echo "###############################################################################"

# Wait if apt is running. 
while :
do
    count=`ps -ef | grep apt.systemd.daily | grep lock_is_held | grep -v grep | wc -l`
    if [ $count = 0 ]; then
        break
    else
        echo "System update is running.. Wait until the completion"
        sleep 10
    fi
done

sudo apt-get update
source /opt/ros/$ROS_DISTRO/setup.sh
rosdep update

sudo pip3 install -U awscli
sudo pip3 install -U colcon-common-extensions colcon-ros-bundle
sudo pip3 install boto3

STACK_NAME=meirorunner`echo $C9_USER|tr -d [\.\\-=_@]` 

export SETUPTOOLS_USE_DISTUTILS=stdlib
aws cloudformation deploy --template-file ./cf/meirorunner.template.json --stack-name $STACK_NAME --capabilities CAPABILITY_IAM
python3 ./ws_setup.py $STACK_NAME

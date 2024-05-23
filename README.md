# ros_telegram_bot
A telegram bot made for ros

## Download
```bash
cd ~/catkin_ws/src
git clone https://github.com/MDP-Group-16/ros_telegram_bot
sudo pip install python-telegram-bot --upgrade
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Setup
You need to get a TOKEN talking to the @botfather bot as [described here](https://core.telegram.org/bots#6-botfather).

Create a config/token.yaml with the your own token. See the [template_token.yaml](config/template_token.yaml) as an example.


## run code
choose one of the options
```bash
roslaunch telegram_bot echobot.launch
roslaunch telegram_bot webcam_camera.launch
roslaunch telegram_bot robot_camera.launch
roslaunch telegram_bot mirte.launch
```

# ros_telegram_bot
A telegram bot made for ros. 
The Mirte launch is made for the Mirte robot. More information about Mirte can be found [here](https://www.mirte.org), the Master version is not yet released outside of the course.

Current features for the Mirte robot:
- [x] Show battery percentage
- [x] Show camera image

> :warning: **Warning** 
> This project is still in development and is not yet ready for production use. However, support and pull requests are welcome.

## Download
```bash
cd ~/catkin_ws/src
git clone https://github.com/MDP-Group-16/ros_telegram_bot
sudo pip install python-telegram-bot --upgrade
cd ~/catkin_ws
rosdep install --from-paths ./src/ --ignore-packages-from-source --rosdistro noetic -y
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

## commands usage
You can use the following commands: 
- 'start' or 'clean': Start the cleaning process.
- 'stop': Stop the cleaning process.
- 'pause': Pause the cleaning process.
- 'picture' or 'camera': Get a picture from the robot's camera.
- 'battery' or 'power': Get the current battery status.
- 'map': Get the current map as an image.
- 'progress': Get the current progress of the robot.
- 'say <text>': Make the robot say something.
- 'topic <topic_name>': Get the latest message from the specified ROS topic.
- You will receive a notification when the progress reaches 100%.

The start, stop and pause commands will change to the cleaning state. This will publish a rostopic command '/mirte/progress'.
Thiese commands correspond to the following states:
- 0: Stop
- 1: Cleaning
- 2: Paused


## Course Usage

This project is developed as part of the TU-Delft course `RO47007 Multidisciplinary Project 2023/24` by group 16. However, while this project is open source and can be used by anyone. Due to the nature of the project and course development, usage comes with the following conditions and social contract.  

- **Attribution**: If you use this project is used in your project, you must attribute the original authors. This means that in documentation and presentation you must mention the original authors.
- **Credit**: Any contribution to this project you will be credited in the project documentation and presentation of the original authors. Credit will only be given if merged into the main branch. And of significant `Meaningful contribution`. If you have any other ideas, please let us know. Contributors will be mentioned in the [Contributors](#contributors) section of the documentation.

This is to ensure that the original authors and contributors are credited for their work. While following the condition of unique work and plagiarism for the course.

## Dependencies
- [python-telegram-bot](https://python-telegram-bot.org/)
- [sound_play](http://wiki.ros.org/sound_play)

## Contributors

<table style="border:0px">
  <tbody>
    <tr style="border:0px">
      <td align="center" valign="top" width="14.28%" style="border:0px"><a href="https://github.com/nickdubbel"><img src="https://avatars.githubusercontent.com/u/23498728?v=4" width="100px;" alt="Nick"/><br /><sub><b>Nick</b></sub></a><br /></td>
    </tr>
  </tbody>
</table>

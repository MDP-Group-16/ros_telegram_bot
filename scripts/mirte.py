#!/usr/bin/env python
# -*- coding: utf-8 -*-

from telegram import Update
from telegram.ext import Application, CommandHandler, ContextTypes, MessageHandler, filters
import logging
import rospy
from sensor_msgs.msg import Image, BatteryState
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32 , Int64
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
import rostopic
import asyncio

# sound
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# Enable logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)

logger = logging.getLogger(__name__)


class TelegramBot(object):
    def __init__(self, token):

        # Set CvBridge
        self.bridge = CvBridge()

        # Initialize Telegram bot
        self.application = Application.builder().token(token).build()
        
        # Store user chat_ids for sending messages
        self.chat_ids = set()

        # Add handlers for Telegram commands and messages
        self.application.add_handler(CommandHandler('start', self.start_command))
        # self.application.add_handler(CommandHandler('list_commands', self.list_params_command))
        self.application.add_handler(MessageHandler(filters.TEXT, self.pub_received))
        

        # ROS subscriber for progress topic
        progress_topic = rospy.get_param('/telegram/progress_topic', '/mirte/progress')
        self.progress_sub = rospy.Subscriber(progress_topic, Float32, self.progress_callback)
        self.progress_reached = False

        #  ros publisher
        self.prefered_state_pub = rospy.Publisher('/mirte/prefered_state', Int64, queue_size=10)

        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0

        # Start Telegram bot
        self.application.run_polling(allowed_updates=Update.ALL_TYPES)

    async def start_command(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
        welcome_message = (
            "Welcome to the ROS Telegram Bot!\n\n"
            "You can use the following commands:\n"
            "- 'clean' or 'start': Start the cleaning process.\n"
            "- 'stop': Stop the cleaning process.\n"
            "- 'pause': Pause the cleaning process.\n"
            "- 'picture' or 'camera': Get a picture from the robot's camera.\n"
            "- 'battery' or 'power': Get the current battery status.\n"
            "- 'map': Get the current map as an image.\n"
            "- 'progress': Get the current progress of the robot.\n"
            "- 'say <text>': Make the robot say something.\n"
            "- 'topic <topic_name>': Get the latest message from the specified ROS topic.\n"
            "- You will receive a notification when the progress reaches 100%.\n"

        )
        await update.message.reply_text(welcome_message)

        # Store chat_id for sending messages
        self.chat_ids.add(update.message.chat_id)

    # async def list_params_command(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
    #     params = rospy.get_param_names()
    #     filtered_params = [param for param in params if param.startswith('/telegram')]
    #     if filtered_params:

    #         params_message = "Available ROS parameters:\n" + "\n".join(filtered_params)
    #     else:
    #         params_message = "No parameters available that start with /telegram."
    #     await update.message.reply_text(params_message)


    def get_image(self):
        rospy.loginfo("Getting image...")
        camera_topic = rospy.get_param('/telegram/camera_topic', '/webcam/image_raw')
        image_msg = rospy.wait_for_message(camera_topic, Image, timeout=5.0)
        rospy.loginfo("Got image!")

        try:
            cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            img_file_path = "/tmp/telegram_last_image.jpg"
            cv2.imwrite(img_file_path, cv2_img)
            rospy.loginfo("Saved to: " + img_file_path)
            return img_file_path
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: " + str(e))
            return None

    def get_battery_state(self):
        battery_topic = rospy.get_param('/telegram/battery_topic', '/battery')
        battery_msg = rospy.wait_for_message(battery_topic, BatteryState, timeout=2.0)
        return battery_msg

    def get_topic_info(self, topic_name):
        try:
            topic_class, _, _ = rostopic.get_topic_class(topic_name)
            if topic_class is None:
                rospy.logerr("Topic type for '%s' could not be determined.", topic_name)
                return None
            topic_msg = rospy.wait_for_message(topic_name, topic_class, timeout=2.0)
            return topic_msg
        except Exception as e:
            rospy.logerr("Error retrieving topic info: %s", str(e))
            return None

    def get_map_image(self):
        rospy.loginfo("Getting map...")
        map_topic = rospy.get_param('/telegram/map_topic', '/map')
        map_msg = rospy.wait_for_message(map_topic, OccupancyGrid)
        rospy.loginfo("Got map!")

        try:
            width = map_msg.info.width
            height = map_msg.info.height
            data = np.array(map_msg.data).reshape((height, width))

            # Normalize the values to 0-255
            data = (data - data.min()) * (255 / (data.max() - data.min()))
            data = data.astype(np.uint8)

            # Convert to a color image
            color_map = cv2.cvtColor(data, cv2.COLOR_GRAY2BGR)
            img_file_path = "/tmp/telegram_map_image.jpg"
            cv2.imwrite(img_file_path, color_map)
            rospy.loginfo("Saved to: " + img_file_path)
            return img_file_path
        except Exception as e:
            rospy.logerr("Error processing map: " + str(e))
            return None

    def progress_callback(self, msg):
        if msg.data < 100.0:
            self.progress_reached = False
        if msg.data >= 100.0 and not self.progress_reached:
            self.progress_reached = True
            asyncio.run(self.broadcast_progress_reached(self.chat_ids))
            rospy.loginfo("Progress reached 100%.")

    async def broadcast_progress_reached(self, chat_ids):
        for chat_id in chat_ids:
            await self.application.bot.send_message(chat_id=chat_id, text="The robot is done with cleaning")

    # Define a few command handlers
    async def pub_received(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
        rospy.loginfo("Received: " + str(update))

        # Store chat_id for sending messages
        self.chat_ids.add(update.message.chat_id)

        text = update.message.text.lower()

        if text.startswith('topic '):
            topic_name = text.split('topic ')[1].strip()
            topic_msg = self.get_topic_info(topic_name)
            if topic_msg:
                await update.message.reply_text(f"Latest message from {topic_name}:\n{topic_msg}")
            else:
                await update.message.reply_text(f"Sorry, I couldn't retrieve the message from {topic_name}.")

        elif text.startswith('say '):
            say_text = text.split('say ')[1].strip()
            self.soundhandle.say(say_text, self.voice, self.volume)
            await update.message.reply_text("Saying: " + say_text)

        elif 'clean' in text or 'start' in text:
            rospy.loginfo("Cleaning...")
            self.prefered_state_pub.publish(1)
            await update.message.reply_text("Cleaning...")

        elif 'stop' in text:
            rospy.loginfo("Stop cleaning...")
            self.prefered_state_pub.publish(0)
            await update.message.reply_text("Stop cleaning...")

        elif 'pause' in text:
            rospy.loginfo("Pause cleaning...")
            self.prefered_state_pub.publish(2)
            await update.message.reply_text("Pause cleaning...")
    
        elif 'picture' in text or 'camera' in text:
            img_file_path = self.get_image()
            if img_file_path:
                await update.message.reply_photo(photo=open(img_file_path, 'rb'), caption="This is what I see!")
            else:
                await update.message.reply_text("Sorry, I couldn't retrieve the image.")

        elif 'battery' in text or 'power' in text:
            battery_msg = self.get_battery_state()
            await update.message.reply_text("Battery percentage: " + str(battery_msg.percentage) + "%")

        elif 'progress' in text:
            progress_topic = rospy.get_param('/telegram/progress_topic', '/mirte/progress')
            progress_msg = self.get_topic_info(progress_topic)
            await update.message.reply_text("Progress: " + str(progress_msg) + "%")
        
        elif 'map' in text:
            img_file_path = self.get_map_image()
            if img_file_path:
                await update.message.reply_photo(photo=open(img_file_path, 'rb'), caption="This is the current map!")
            else:
                await update.message.reply_text("Sorry, I couldn't retrieve the map.")
                
        else:
            await update.message.reply_text("Try commands like: 'picture', 'battery', 'progress', 'map', or 'topic <topic_name>'")

    def error(self, bot, update, error):
        logger.warn('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':
    rospy.init_node('telegram_bot_camera')
    token = rospy.get_param('/telegram/token', None)
    if token is None:
        rospy.logerr('Token is not set. Please set the token in the parameter server. "/telegram/token"')
        exit(1)
    TelegramBot(token)

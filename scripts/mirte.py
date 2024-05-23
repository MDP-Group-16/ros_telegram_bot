#!/usr/bin/env python
# -*- coding: utf-8 -*-


from telegram import ForceReply, Update
from telegram.ext import Application, CommandHandler, ContextTypes, MessageHandler, filters

import logging
import rospy
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import BatteryState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Enable logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)

logger = logging.getLogger(__name__)


class telegram(object):
    def __init__(self, token):

        # Set CvBridge
        self.bridge = CvBridge()

        # Create the Application and pass it your bot's token.
        application = Application.builder().token(token).build()

        # on non command i.e message - echo the message on Telegram
        application.add_handler(MessageHandler(filters.TEXT, self.pub_received))

        # Run the bot until the user presses Ctrl-C
        application.run_polling(allowed_updates=Update.ALL_TYPES)

    def get_image(self):
        rospy.loginfo("Getting image...")
        camera_topic = rospy.get_param('/telegram/camera_topic', '/webcam/image_raw')
        image_msg = rospy.wait_for_message(
            camera_topic,
            Image)
        rospy.loginfo("Got image!")

        cv2_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_file_path = "/tmp/telegram_last_image.jpg"
        cv2.imwrite(img_file_path, cv2_img)
        rospy.loginfo("Saved to: " + img_file_path)
        return img_file_path

    

    # Define a few command handlers
    async def pub_received(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
        rospy.loginfo("Received: " + str(update))
        image_words = ['what do you see',
                                 'picture',
                                 'camera']
        battery_words = ['battery',
                            'power',
                            'percentage',]
        found_word = False
        for v in image_words:
            if v in update.message.text.lower():
                img_file_path = self.get_image()
                await update.message.reply_photo(photo=open(img_file_path, 'rb'),
                                           caption="This is what I see!")
                found_word = True
                break
        for v in battery_words:
            if v in update.message.text.lower():
                battery_topic = rospy.get_param('/telegram/battery_topic', '/battery')
                battery_msg = rospy.wait_for_message(
                    battery_topic,
                    BatteryState)
                await update.message.reply_text("Battery percentage: " + str(battery_msg.percentage))
                found_word = True
                break
        if not found_word:
            update.message.reply_text("Try any of: " +
                                      str(image_words))

    def error(self, bot, update, error):
        logger.warn('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':
    rospy.init_node('telegram_bot_camera')
    token = rospy.get_param('/telegram/token', None)
    if token is None:
        rospy.logerr('Token is not set. Please set the token in the parameter server. "/telegram/token"')
        exit(1)
    telegram(token)
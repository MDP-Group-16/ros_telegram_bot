#!/usr/bin/env python
# -*- coding: utf-8 -*-


from telegram import ForceReply, Update
from telegram.ext import Application, CommandHandler, ContextTypes, MessageHandler, filters

import logging
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Enable logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.INFO)

logger = logging.getLogger(__name__)


class ImageReply(object):
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

    # For some reason the image is grayscale...
    def get_image_compressed(self):
        rospy.loginfo("Getting image...")
        camera_topic = rospy.get_param('/telegram/camera_topic', '/webcam/image_raw')
        image_msg = rospy.wait_for_message(
            camera_topic,
            CompressedImage)
        rospy.loginfo("Got image!")

        # Image to numpy array
        np_arr = np.fromstring(image_msg.data, np.uint8)
        # Decode to cv2 image and store
        cv2_img = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        img_file_path = "/tmp/telegram_last_image.png"
        cv2.imwrite(img_file_path, cv2_img)
        rospy.loginfo("Saved to: " + img_file_path)
        return img_file_path

    # Define a few command handlers
    async def pub_received(self, update: Update, context: ContextTypes.DEFAULT_TYPE) -> None:
        rospy.loginfo("Received: " + str(update))
        valid_necessary_words = ['what do you see',
                                 'picture',
                                 'camera']
        found_word = False
        for v in valid_necessary_words:
            if v in update.message.text.lower():
                img_file_path = self.get_image()
                await update.message.reply_photo(photo=open(img_file_path, 'rb'),
                                           caption="This is what I see!")
                found_word = True
                break
        if not found_word:
            update.message.reply_text("Try any of: " +
                                      str(valid_necessary_words))

    def error(self, bot, update, error):
        logger.warn('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':
    rospy.init_node('telegram_bot_camera')
    token = rospy.get_param('/telegram/token', None)
    if token is None:
        rospy.logerr('Token is not set. Please set the token in the parameter server. "/telegram/token"')
        exit(1)
    cp = ImageReply(token)
    rospy.spiget_imagen()
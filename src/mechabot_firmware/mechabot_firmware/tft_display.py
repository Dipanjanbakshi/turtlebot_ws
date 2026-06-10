#!/usr/bin/env python3

import subprocess
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import board
import digitalio

from PIL import Image, ImageDraw, ImageFont
from adafruit_rgb_display import st7735


# ---------------- DISPLAY SETUP ----------------

cs_pin = None
dc_pin = digitalio.DigitalInOut(board.D24)
reset_pin = digitalio.DigitalInOut(board.D22)

BAUDRATE = 24000000
spi = board.SPI()

disp = st7735.ST7735R(
    spi,
    rotation=270,
    cs=cs_pin,
    dc=dc_pin,
    rst=reset_pin,
    baudrate=BAUDRATE,
    width=128,
    height=160,
)

if disp.rotation % 180 == 90:
    height = disp.width
    width = disp.height
else:
    width = disp.width
    height = disp.height

image = Image.new("RGB", (width, height))
draw = ImageDraw.Draw(image)

font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)


# ---------------- ROS NODE ----------------

class TFTDashboard(Node):

    def __init__(self):

        super().__init__('tft_dashboard')

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.battery = 75.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Float32, '/battery_percent', self.battery_callback, 10)

        thread = threading.Thread(target=self.display_loop)
        thread.daemon = True
        thread.start()

    def cmd_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z

    def battery_callback(self, msg):
        self.battery = msg.data


# ---------------- SYSTEM INFO ----------------

    def get_ip(self):
        try:
            cmd = "hostname -I | cut -d' ' -f1"
            return subprocess.check_output(cmd, shell=True).decode().strip()
        except:
            return "NO NET"

    def get_cpu(self):
        try:
            cmd = "top -bn1 | grep load | awk '{printf \"%.2f\", $(NF-2)}'"
            return float(subprocess.check_output(cmd, shell=True).decode())
        except:
            return 0.0


# ---------------- BAR DRAW FUNCTION ----------------

    def draw_bar(self, x, y, value, max_value, color):

        bar_width = 80
        filled = int((value / max_value) * bar_width)

        draw.rectangle((x, y, x + bar_width, y + 8), outline=(40,40,40))
        draw.rectangle((x, y, x + filled, y + 8), fill=color)


# ---------------- DISPLAY LOOP ----------------

    def display_loop(self):

        while True:

            draw.rectangle((0,0,width,height), fill=(0,0,0))

            cpu = self.get_cpu()
            ip = self.get_ip()

            y = 0

            # Header
            draw.rectangle((0,0,width,22), fill=(0,120,200))
            draw.text((5,3),"TURTLEBOT HUD",font=font_title,fill=(255,255,255))

            y = 26

            # CPU BAR
            draw.text((4,y),"CPU",font=font_small,fill=(255,180,0))
            self.draw_bar(40,y,cpu,2,(255,120,0))
            y += 16

            # BATTERY BAR
            draw.text((4,y),"BAT",font=font_small,fill=(0,255,140))
            self.draw_bar(40,y,self.battery,100,(0,255,120))
            y += 20

            # IP ADDRESS
            draw.text((4,y),f"IP {ip}",font=font_small,fill=(0,200,255))
            y += 16

            # LINEAR SPEED
            draw.text((4,y),f"LIN {self.linear_speed:.2f} m/s",font=font_small,fill=(255,255,0))
            y += 14

            # ANGULAR SPEED
            draw.text((4,y),f"ANG {self.angular_speed:.2f} rad/s",font=font_small,fill=(255,100,255))

            # FOOTER
            draw.rectangle((0,height-16,width,height),fill=(20,20,20))
            draw.text((5,height-14),"ROS ACTIVE",font=font_small,fill=(0,255,150))

            disp.image(image)

            time.sleep(0.2)


# ---------------- MAIN ----------------

def main():

    rclpy.init()

    node = TFTDashboard()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
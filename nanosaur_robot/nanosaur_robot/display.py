# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Display
import Adafruit_SSD1306

import atexit
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


class Display:

    def __init__(self, node, timer_period=1, i2c_address=0x3C) -> None:
        self.node = node
        self.i2c_address = i2c_address
        # 128x32 display with hardware I2C:
        # setting gpio to 1 is hack to avoid platform detection
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1, i2c_address=i2c_address)
        # Init display
        self.disp.begin()
        # Clear display.
        self.disp.clear()
        self.disp.display()
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        # Load default font.
        self.font = ImageFont.load_default()
        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        self.top = padding
        self.bottom = self.height-padding
        # Move left to right keeping track of the current x position for drawing shapes.
        self.x = 0
        # Init display timer
        self.timer = self.node.create_timer(timer_period, self.display_callback)
        # Configure all motors to stop at program exit
        atexit.register(self._close)

    def display_callback(self):
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        # Draw text
        self.draw.text((self.x, self.top), f"ID: {self.i2c_address}", font=self.font, fill=255)

        self.disp.image(self.image)
        self.disp.display()
    
    def _close(self):
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.disp.image(self.image)
        self.disp.display()
# EOF

import time
from neopixel import *

led_count = 24
led_pin = 18
led_freq_Hz = 800000 # (800KHz)
led_dma = 5   #DMA channel
led_brightness = 255 # maximum (8 bit)
Led_Invert = False # True inverts signal (for NPN transistors)
       
if __name__ = '__main__':
    strip = Adafruit_NeoPixel(led_count, led_pin, led_freq_Hz, led_dma, led_invert, led_brightness)
    strip.begin()
        for i in strip.numPixels:
            strip.setPixelColor(i, 0, 0, 255) # sets each led to green 
            if i == strip.numPixels:
                strip.show()

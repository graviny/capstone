#import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice as PWM
import time

Left_R = 19
Left_G = 0
Left_B = 13

Right_R = 1
Right_G = 5
Right_B = 6

colors = [0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0X6F00D2, 0xFF5809]

def setup():
  global L_R, L_G, L_B, R_R, R_G, R_B

  L_R = PWM(pin=Left_R, initial_value=1.0, frequency=2000)
  L_G = PWM(pin=Left_G, initial_value=1.0, frequency=2000)
  L_B = PWM(pin=Left_B, initial_value=1.0, frequency=2000)

  R_R = PWM(pin=Right_R, initial_value=1.0, frequency=2000)
  R_G = PWM(pin=Right_G, initial_value=1.0, frequency=2000)
  R_B = PWM(pin=Right_B, initial_value=1.0, frequency=2000)



def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def setAllColor(col):   # For example : col = 0x112233
  R_val = (col & 0xff0000) >> 16
  G_val = (col & 0x00ff00) >> 8
  B_val = (col & 0x0000ff) >> 0

  R_val = map(R_val, 0, 255, 0, 1.00)
  G_val = map(G_val, 0, 255, 0, 1.00)
  B_val = map(B_val, 0, 255, 0, 1.00)
  
  L_R.value = 1.0-R_val
  L_G.value = 1.0-G_val
  L_B.value = 1.0-B_val

  R_R.value = 1.0-R_val
  R_G.value = 1.0-G_val
  R_B.value = 1.0-B_val

def setAllRGBColor(R,G,B):   # For example : col = 0x112233

  R_val = map(R, 0, 255, 0, 1.00)
  G_val = map(G, 0, 255, 0, 1.00)
  B_val = map(B, 0, 255, 0, 1.00)
  
  L_R.value = 1.0-R_val
  L_G.value = 1.0-G_val
  L_B.value = 1.0-B_val

  R_R.value = 1.0-R_val
  R_G.value = 1.0-G_val
  R_B.value = 1.0-B_val

def loop():
  while True:
    for col in colors:
      setAllColor(col)
      time.sleep(0.5)

def destroy():
  L_R.stop()
  L_G.stop()
  L_B.stop()
  R_R.stop()
  R_G.stop()
  R_B.stop()

if __name__ == "__main__":

  setup()
  try:
    loop()
  except KeyboardInterrupt:
    destroy()

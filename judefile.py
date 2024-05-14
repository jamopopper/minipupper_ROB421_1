from UDPComms import Publisher
import time 
from MangDang.mini_pupper.display import Display

drive_pub = Publisher(8830) 

if __name__ == "__main__":
    disp = Display()
    while(time.sleep(10)):
        disp.show_image("img/elbow.jpg")

    

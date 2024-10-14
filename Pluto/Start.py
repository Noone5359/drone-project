import matplotlib
import multiprocessing
from multiprocessing import Pipe
import time 
from Pluto.PYpluto.Control.PID import PID_main
from Pluto.PYpluto.Cam.marker import marker_publisher

def build_conn():

    connCam,connDrone1 = Pipe(duplex = True)


    p1 = multiprocessing.Process(target=marker_publisher, args=( [connCam]))
    p2 = multiprocessing.Process(target=PID_main, args=([connDrone1]))



    #first detect pose , then takeoff
    p1.start()
    print('\n-------Starting process Camera-------')
    time.sleep(2)

    print("\n-----Starting Drone1--------")
    p2.start()

    
    p1.join()
    p2.join()
    


if __name__ == "__main__":

    
    build_conn()  #starts camera file and drone1 file , also builds connection btw the two

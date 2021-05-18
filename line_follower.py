import time
import cv2
import numpy
import wiringpi


def set_fps(fps_count):
    if(cap.set(5,fps_count)):
        print("FPS succesfully set to "+str(fps_count))
    else:
        print("FPS setting FAILED!")
    
def set_serial(left,right):
    line=str(int(left))+' '+str(int(right))+'\n'
    wiringpi.serialPuts(serial,line)
    
def process_angle_error(angle,error):
    global clock
    clock+=1
    if(clock>divider):
        clock=0
        KP=1.25
        if abs(angle)<15: #10
            KP=1.75
        AP=1.25
        speed=100
        steering = error * KP + angle * AP
        if steering == 0:
            port1=speed
            port2=speed
        else:
            if steering > 0:
                steering = 100 - steering
                port2=speed
                port1=speed*steering/100
            else:
                steering = steering * -1
                steering = 100 - steering;
                port1=speed
                port2=speed*steering/100
        max_backward_speed=50
        if port1<-max_backward_speed:
            port1=-max_backward_speed #right
        if port2<-max_backward_speed:
            port2=-max_backward_speed #left
        set_serial(port2,port1)
        
def remove_noise(followed_line,errode_iterations,dilate_iterations):
    kernel = numpy.ones((3,3), numpy.uint8)
    followed_line = cv2.erode(followed_line, kernel, iterations=errode_iterations) #5
    followed_line = cv2.dilate(followed_line, kernel, iterations=dilate_iterations) #10
    return followed_line

def compute_angle(width, height, angle):
    if angle < -90 or (width > height and angle < 0):
        return 90 + angle
    if width < height and angle > 0:
        return (90 - angle) * -1 
    return angle

def print_numbers(frame, angle, distance,intersection):
    angle_pos=(0,97*resolution[1]//100)
    error_pos=(0,12*resolution[1]//100)
    inters_pos=(93*resolution[0]//100,97*resolution[1]//100)
    cv2.putText(frame, str(angle), angle_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1)
    cv2.putText(frame, str(distance), error_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 1)
    if(intersection):
       cv2.putText(frame, "S", inters_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
     

def pre_process_frame(frame):
    #wykrywanie niebieskiej linii
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    lower_bound_blue = numpy.array([100,40,40]) 
    upper_bound_blue = numpy.array([140,255,255]) 
    followed_line = cv2.inRange(hsv, lower_bound_blue, upper_bound_blue)
    followed_line = remove_noise(followed_line,5,5)
    cv2.imshow(WINDOW_2, followed_line)
    #return clean image used to find rectangles
    return followed_line 
   
def process_frame(frame, resolution, last_followed):
    x_res, y_res = resolution
    x_last, y_last = last_followed
    distance, angle = 0, 0
    followed_line=pre_process_frame(frame)
    image, contours, hierarchy = cv2.findContours(followed_line, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        if len(contours) == 1:
            selection = cv2.minAreaRect(contours[0])
        else:
            possible_selections = []
            off_bottom_selections = []
       
            for i in range(len(contours)):
                selection = cv2.minAreaRect(contours[i])
                (x, y), (width, height), angle = selection
                (x_bottom, y_bottom) = cv2.boxPoints(selection)[0] #first vertex is always the closest to the bottom
                if y_bottom >= y_res - 1:
                    last_followed_distance = ((x_last - x) ** 2 + (y_last - y) ** 2) ** 0.5
                    off_bottom_selections.append((last_followed_distance, i))
                
                possible_selections.append((y_bottom, i))
            
            off_bottom_selections = sorted(off_bottom_selections)
            if len(off_bottom_selections) > 0:
                last_followed_distance, i = off_bottom_selections[0]
                selection = cv2.minAreaRect(contours[i])
            else:
                possible_selections = sorted(possible_selections)
                y_bottom, i = possible_selections[-1]
                
                selection = cv2.minAreaRect(contours[i])
        
        (x, y), (width, height), angle = selection
        
        if width*height>0.6*resolution[0]*resolution[1]:
            intersection=True
            angle=0
        else:
            intersection=False
            angle = int(compute_angle(width, height, angle))
        #right-angle turns detection routine
        if width*height>0.2*resolution[0]*resolution[1] and not width*height>0.6*resolution[0]*resolution[1]:
            if abs(angle)>70:
                if angle<0 and x>resolution[0]*0.5 and y>resolution[0]*0.5:
                    print("Right!")
                    angle=80
                elif angle>0 and x<resolution[0]*0.5 and y>resolution[0]*0.5:
                    print("Left!")
                    angle=-80      
        middle = int(x_res / 2)
        distance = int(x - middle)
        box = cv2.boxPoints(selection)
        box = numpy.int0(box)
        
        box_color=(0, 0, 255) #red normal
        if intersection:
            box_color= (0, 255, 255) #yellow on intersection
        cv2.drawContours(frame, [box], 0, box_color, 3)
        cv2.line(frame, (int(x), int(resolution[1]*0.6)), (int(x), int(resolution[1]*0.4)), (255,0,0), 1)
        print_numbers(frame, angle, distance,intersection)
        process_angle_error(angle, 100*distance/(x_res/2.0)) #process angle accepts percents now only!
    return distance, angle, (x_last, y_last)

def finish():
    global serial
    cap.release()
    cv2.destroyAllWindows()
    wiringpi.serialPuts(serial,"0 0\n")
    wiringpi.serialClose(serial)
    exit()

def manual_steering():
    set_serial(0,0)
    manual_steering_speed=100
    while(True):
        ret, frame = cap.read()
        cv2.imshow(PROGRAM_NAME, frame)
        pre_process_frame(frame)
        key=cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            set_serial(0,0)
        if key == ord('w'):
            set_serial(manual_steering_speed,manual_steering_speed)
        if key == ord('s'):
            set_serial(-manual_steering_speed,-manual_steering_speed)
        if key == ord('a'):
            set_serial(-manual_steering_speed*0.8,manual_steering_speed*0.8)
        if key == ord('d'):
            set_serial(manual_steering_speed*0.8,-manual_steering_speed*0.8)
        if key == ord('e'):
            break
        if key == ord('q'):
            finish()
            
def main_loop():
    last_followed = int(resolution[0] / 2), int(resolution[1] / 2)
    while(True):
        ret, frame = cap.read()
        distance, angle, last_followed = process_frame(frame, resolution, last_followed)
        temp_frame=cv2.resize(frame,(resolution[0]*2,2*resolution[1]))
        cv2.imshow(PROGRAM_NAME, temp_frame)
        key=cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' '):
            manual_steering()

#parameters/defaluts
PROGRAM_NAME = "LF grupa 30"
WINDOW_2 = "Grey_scale"
#camera------
resolution = (160,120)
CONST_FPS=20
cap = cv2.VideoCapture("/dev/video0") #replace with a path to your webcam
cap.set(3, resolution[0]) #needed to set resolution 3-width;4-height
cap.set(4, resolution[1])
cap.set(10,1.0)    #brightness
#FPS---------------------
set_fps(CONST_FPS) #sets fps to given value
clock=0 #global clock value for the divider under
divider=CONST_FPS/10 #sending serial data every 0.1s
#serial initialization---------
wiringpi.wiringPiSetup()
serial = wiringpi.serialOpen('/dev/ttyS0',9600)
cv2.namedWindow(WINDOW_2)
main_loop()
finish()
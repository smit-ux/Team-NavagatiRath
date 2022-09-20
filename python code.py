import apriltag
import cv2 
import time
import math
import serial
import vlc
import threading

def thread1():
    while 1:
        global ult
        if(arduino.in_waiting):
           ultra_data = arduino.read(1)
           ult=int.from_bytes(ultra_data,'big')
           arduino.reset_input_buffer()
           
           
        

tag = 28
ultra_data = 0
next_tag=28
last_tag=28
array = [28,29,13,15,10,14,5, 14,10 ,15, 13, 29, 28,0]
array_togo = [0]*50
array_togo[28]=[29,60]
array_togo[29]=[13,28]
array_togo[13]=[15,29]
array_togo[15]=[10,13]
array_togo[10]=[14,15]
array_togo[14]=[5,10]
array_togo[5]=[60,14]
array_togo[2]=[1,28]
array_togo[1]=[0,2]
array_togo[0]=[11,1]
array_togo[11]=[15,0]

ult=0

obstacle = [0]*50
obstacle[29]=[0,0]
obstacle[13]=[11,11]
obstacle[15]=[50,50]
obstacle[10]=[50,50]
obstacle[14]=[50,50]
obstacle[5]=[50,50]
obstacle[28]=[1,1]
obstacle[2]=[29,28]
obstacle[1]=[13,29]
obstacle[0]=[15,29]
obstacle[11]=[15,13]

direct = 0
i=0
print("start")
def reached():
    print('reached')
    
    
def send_angle(ang):

    integer=ang
    is_dist = 0
    sign = False
    if(integer<0):
        sign = True
    intg = int(abs(integer))
    byte = intg.to_bytes(1,'big')
    dtype = sign|(is_dist<<4)
    check = (byte[0]+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte[0]) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
    
    
def send_dist(dis):
    integer=(dis*(-1))
    
    is_dist = 1
    sign = False
    if(integer<0):
        sign = True
    intg = int(abs(integer))&0xFF
    byte = intg.to_bytes(1,'big')
    dtype = sign|(is_dist<<4)
    check = (byte[0]+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte[0]) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
    

    
def rotate():
    print('rotate')
    byte = 0xF0
    dtype = 0x77
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
def reached():
#     print('rotate')
    byte = 0x00
    dtype = 0x45
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
def data(inp):
#     print('rotate')
    
    byte = inp
    dtype = 0x15
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
def u1(inp):
#     print('rotate')
    
    byte = inp
    dtype = 0x10
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
def u2(inp):
#     print('rotate')
    
    byte = inp
    dtype = 0x20
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
def u3(inp):
#     print('rotate')
    
    byte = inp
    dtype = 0x30
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)
def serve():
#     print('rotate')
    
    byte = 0x00
    dtype = 0x05
    check = (byte+dtype)& 0xFF
    st = ''+chr(0x01)+chr(0x01) + chr(dtype) + chr(byte) + chr(check)
    byt =bytes(st,'latin-1')
    arduino.write(byt)


arduino = serial.Serial(port='/dev/ttyACM0',baudrate= 9600)


pro = vlc.MediaPlayer("123.mp3")
# p.play()
vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640);
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480);
prev_frame_time = 0
font = cv2.FONT_HERSHEY_SIMPLEX
new_frame_time = 0
count = 0
# x = threading.Thread(target=thread1)
# x.start()
time.sleep(1)
pro.play()
# u1(0x0A)
# u2(0x0A)
# u3(0x0A)
# ultra_data = arduino.read(1)
while(True):
    
    ret, image = vid.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (y,x)= gray.shape
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    count+=1
    for r in results:
        if (r.tag_id!=next_tag): continue
        count = 0
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        #print("[INFO] tag family: {}".format(tagFamily))
        (h, w) = image.shape[:2] #w:image-width and h:image-height
        cv2.line(image,(x//2,0),(x//2,y),(0,0,255),1)
        cv2.line(image,(0,y//2),(x,y//2),(255,0,0),1)
        cv2.circle(image,ptA, 5, (0, 0, 255), -1)
        cv2.circle(image,ptB, 5, (0, 0, 255), -1)
        cv2.circle(image,ptC, 5, (0, 0, 255), -1)
        cv2.circle(image,ptD, 5, (0, 0, 255), -1)
        cv2.circle(image,(0,0), 5, (0, 0, 255), -1)
        deg = math.atan2((ptC[1]-ptB[1]),(ptC[0]-ptB[0]))*57.3248
#         final = str(int(deg))
        t1 = cX-x/2
        t2 = cY-y/2
        ta = math.atan2(t1,t2)*57.3248
        cv2.line(image,(cX,cY),(x//2,y//2),(0,0,255),1)
        d = (ta-180)-(deg+90)
        t3 = math.sqrt(math.pow(t1,2)+ math.pow(t2,2))
        yd = t3*math.sin(math.radians(d))
                         
        xd = t3*math.cos(math.radians(d))
        #print(t1,t2,-t3)
        #print(xd,yd)
        p = (ptC[1]-ptB[1]),(ptC[0]-ptB[0])
        pz = math.sqrt(math.pow((ptC[1]-ptB[1]),2)+math.pow((ptC[0]-ptB[0]),2))
        q = 16/pz
#         print('angle:',deg,'x-dist:',xd*q,'y-dist:',yd*q)
        byte1=0x00
        byte2=0x00
        
        
        
        if(ta>10+90):
            byte1=0x08
        elif(ta<(-10)+90):
            byte1=0x04
        elif(t3>(10)):
            byte1=0x01
        elif(t3<(-10)):
            byte1=0x02
            
        
        
        
        if(abs(ta)<50+90 and abs(t3)<20):
            
            last_tag=next_tag
            next_tag=array_togo[last_tag][direct]
            if next_tag==60:
                serve()
                time.sleep(2)
                u3(0x0A)
                
                
            
        else:
            data(byte1|byte2)
        
        if(ult&0x02):
            next_tag=obstacle[last_tag][direct]
            
        
        
    
            
        
            
        tagFamily = str(r.tag_id)
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
      
    new_frame_time = time.time()
    fps = 1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    
    fps = str(int(fps))
    cv2.putText(image, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)
    cv2.imshow('frame', image)
    
       
    
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        
        break
    
vid.release()
# write_read(str(0))
# Destroy all the windows
cv2.destroyAllWindows()





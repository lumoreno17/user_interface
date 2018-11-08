#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import*
from phidgets_interface.msg import DeviceInfo
import Tkinter as tkinter
import cv2
import PIL.Image, PIL.ImageTk
import serial
import numpy as np
import os.path
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from battery_nh2054.msg import Battery
import datetime


class UserInterface:


    def __init__(self, window, window_title):

        # Variables definition
        self.mult_image = 5 #Multiply factor for image size
        self.hotspot_count = Int32() #Number of hotspots detected

	# Creating and Configuring the window
        self.window = window
        self.window.geometry("1500x800")
        self.window.title(window_title)
	self.background = 'white'
        self.window.configure(bg=self.background)


	#Creating and configuring the frame
        self.frame = tkinter.Frame(self.window, bg='white', width=6*120, height=6*80+50)
        self.frame.pack()
	

	

        # ----------------------- VIDEO CANVAS -------------------------------
        self.video_canvas()
        #------------------------ TITLE CANVAS -------------------------------
        #self.title_canvas()
        #------------------------ SYSTEM INTEGRITY CANVAS --------------------
        self.system_int_canvas()
 	#-------------------------CLAW ALIGNMENT CANVAS----------------------
	self.claw_canvas()


        #------------------------OCURRENCES FRAME----------------------------
        self.ocurrences_viewer()
        #----------------------CAMERA FRAME VARIABLE---------------------
        self.frame_data = np.zeros((300,400,3), dtype= np.uint8)


	#----------------------- ROS CONFIGURATION ---------------------------
	rospy.init_node('listener_tkinter2', anonymous=True)
	rospy.Subscriber('temperature', DeviceInfo, self.callback_temperature)
	rospy.Subscriber('sonar', DeviceInfo, self.callback_sonar)
        rospy.Subscriber('proximity1', DeviceInfo, self.callback_proximity1)
        rospy.Subscriber('proximity2', DeviceInfo, self.callback_proximity2)
        rospy.Subscriber('proximity3', DeviceInfo, self.callback_proximity3)
        rospy.Subscriber('proximity4', DeviceInfo, self.callback_proximity4)
        rospy.Subscriber('proximity5', DeviceInfo, self.callback_proximity5)
        rospy.Subscriber("processed_image", numpy_msg(Floats), self.show_frame)
        rospy.Subscriber("hotspot_count", Int32, self.callback_hotspot)
        rospy.Subscriber("battery_status", Battery, self.callback_battery_info)
	


	#----------------------- QUIT BUTTOM ----------------------------------
        quit_button = tkinter.Button(window, text='Close', width=16, command=window.destroy, bg = "#ffaabb")
        quit_button.place(relx=.55, rely=.97, anchor="c")

        #----------------------- ACTUATORS INFO BUTTOM ----------------------------------
        actuators_button = tkinter.Button(window, text='Actuators Info', width=16, command=window.destroy, bg = "#bbbbff")
        actuators_button.place(relx=.75, rely=.97, anchor="c")

        #------------------------ OCURRENCES BUTTOM----------------------------------
        #ocurrences_button = tkinter.Button(window, text='Ocurrences', width=16, command=window.destroy, bg = "#aaddff")
        #ocurrences_button.place(relx=.8, rely=.97, anchor="c")


	#self.window.after(1, self.gui_loop)
        self.vmax = 8400
        self.vmin = 7600
        self.window.mainloop()


    def video_canvas(self):

        self.canvas_image = tkinter.Canvas(self.window, bg = 'white' , width = self.mult_image*80, height = 6*60, bd = 0, highlightthickness =0)
        self.canvas_image.create_window((0,0), window=self.frame)
        self.canvas_image.place(relx=0.2, rely=0.3, anchor="c")

        tkinter.Label(self.canvas_image,text = "FLIR Camera", font=("Helvetica", 16), bg=self.background, fg="#284f6d").place(relx = 0.5, rely = 0.1, anchor="c")
        #tkinter.Label(self.canvas_image,text = "Binarized Image", font=("Helvetica", 14), bg=self.background, fg="#284f6d").place(relx = 0.5, rely = 0.565, anchor="c")
    

    def title_canvas(self):

        self.canvas_title = tkinter.Canvas(self.window, bg = 'white' , width = 1440, height = 50, bd = 0, highlightthickness =0)
        self.canvas_title.create_window((0,0), window=self.frame)
        self.canvas_title.place(relx=0.5, rely=0.05, anchor="c")

        #tkinter.Label(self.canvas_title,text = "Electrical Line Inspection Robot Dashboard", font=("Helvetica", 18), bg = self.background).place(relx = 0.5, 			rely = 0.5, anchor="c")                            
    

    def ocurrences_viewer(self):

        self.frame_ocurrences = tkinter.Frame(self.window, bg = 'white', width = self.mult_image*80, height = 300)
        self.frame_ocurrences.place(relx=0.2, rely=0.78, anchor="c")
        self.scrollbar = tkinter.Scrollbar(self.frame_ocurrences)
        self.scrollbar.pack(side = 'right', fill = 'y')
        self.listbox = tkinter.Listbox(self.frame_ocurrences, selectmode = 'single', width = 48, height = 15, bg = "#c4e0dc",yscrollcommand=self.scrollbar.set)
        
        self.listbox.pack(side = 'left', fill = 'y')
        #self.listbox.insert('end', "a list entry")
        self.canvas_ocurrences = tkinter.Canvas(self.window, bg = 'blue' , width = self.mult_image*80, height = 300, bd = 0, highlightthickness =0)
        self.canvas_ocurrences.create_window((0,0), window=self.frame)
        self.canvas_ocurrences.place(relx=0.2, rely=0.6, anchor="c")
        tkinter.Label(self.canvas_ocurrences,text = "Ocurrences", font=("Helvetica", 16), bg = 'white', fg= '#284f6d').pack(side = 'top')                            
        
    def claw_canvas(self):
	#1
	self.canvas_circle1 = tkinter.Canvas(width=16, height = 16, bg = 'white')
	self.claw1 = self.canvas_circle1.create_oval(3, 3, 15, 15, fill = "red",width=1)
	self.canvas_circle1.place(relx=0.465, rely=0.505, anchor="c")
        #3
	self.canvas_circle3 = tkinter.Canvas(width=16, height = 16, bg = 'white')
	self.claw3 = self.canvas_circle3.create_oval(3, 3, 15, 15, fill = "red",width=1)
	self.canvas_circle3.place(relx=0.630, rely=0.512, anchor="c")
        #5
	self.canvas_circle5 = tkinter.Canvas(width=16, height = 16, bg = 'white')
	self.claw5 = self.canvas_circle5.create_oval(3, 3, 15, 15, fill = "red",width=1)
	self.canvas_circle5.place(relx=0.827, rely=0.505, anchor="c")
        #2
	self.canvas_circle2 = tkinter.Canvas(width=16, height = 16, bg = 'white')
	self.claw2 = self.canvas_circle2.create_oval(3, 3, 15, 15, fill = "red",width=1)
	self.canvas_circle2.place(relx=0.532, rely=0.505, anchor="c")
        #4
	self.canvas_circle4 = tkinter.Canvas(width=16, height = 16, bg = 'white')
	self.claw4 = self.canvas_circle4.create_oval(3, 3, 15, 15, fill = "red",width=1)
	self.canvas_circle4.place(relx=0.763, rely=0.505, anchor="c")



    def system_int_canvas(self):

	#Battery Icon Size
	wsize_batt = 200
        hsize_batt = 300
        
	#Temperatura Icon Size
	wsize_temp = 170
        hsize_temp = 350

	#Sonar Icon Size       
        wsize_sonar = 350
        hsize_sonar = 350

	#GPS Icon Size
	wsize_gps = 350
	hsize_gps = 300

	#IMU Icon Size
	wsize_imu = 350
	hsize_imu = 350

	#Power Info Icon Size
	wsize_power = 350
	hsize_power = 350

        #--------------------------------- Leitura das imagens da GUI --------------------------------------
        script_dir = os.path.dirname(os.path.abspath(__file__)) #Set the file directory as the reference
	batt100_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/batt100.png"))
        batt75_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/batt75.png"))
        batt50_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/batt50.png"))
        batt25_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/batt25.png"))
        termometro_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/termometro.png"))
        sonar_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/sonar.png"))
	gps_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/gps.png"))
        imu_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/imu1.png"))
	power_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/power.png"))
	elir_im = PIL.Image.open(os.path.join(script_dir,"Imagens_GUI/piro.png"))


        #---------------------------------------- Canvas Creation -----------------------------------------
        self.canvas_system_int = tkinter.Canvas(self.window, bg = self.background , width = 800, height = 600, bd = 0, highlightthickness =0)
        self.canvas_system_int.create_window((0,0), window=self.frame)
        self.canvas_system_int.place(relx=0.67, rely=0.5, anchor="c")
        
        #---------------------------------------Redimensionar as imagens--------------------------------------------------
        batt100_im = batt100_im.resize((wsize_batt/6,hsize_batt/6), PIL.Image.ANTIALIAS)
        batt75_im = batt75_im.resize((wsize_batt/6,hsize_batt/6), PIL.Image.ANTIALIAS)
        batt50_im = batt50_im.resize((wsize_batt/6,hsize_batt/6), PIL.Image.ANTIALIAS)
        batt25_im = batt25_im.resize((wsize_batt/6,hsize_batt/6), PIL.Image.ANTIALIAS)
        termometro_im = termometro_im.resize((int(wsize_temp/6.3),int(hsize_temp/6.3)), PIL.Image.ANTIALIAS)
        sonar_im = sonar_im.resize((int(wsize_sonar/6.3),int(hsize_sonar/6.3)), PIL.Image.ANTIALIAS)
	gps_im = gps_im.resize((int(wsize_gps/6),int(hsize_gps/6)), PIL.Image.ANTIALIAS)
	imu_im = imu_im.resize((int(wsize_imu/6),int(hsize_imu/6)), PIL.Image.ANTIALIAS)
	power_im = power_im.resize((int(wsize_power/6),int(hsize_power/6)), PIL.Image.ANTIALIAS)
	elir_im = elir_im.resize((int(elir_im.size[0]/1.6),int(elir_im.size[1]/1.6)), PIL.Image.ANTIALIAS)
       
        #------------------------------------ Tkinter format conversion --------------------------
        self.batt100_photo =  PIL.ImageTk.PhotoImage(batt100_im)
        self.batt75_photo =   PIL.ImageTk.PhotoImage(batt75_im)
        self.batt50_photo =   PIL.ImageTk.PhotoImage(batt50_im)
        self.batt25_photo =   PIL.ImageTk.PhotoImage(batt25_im)
        self.termometro_photo =    PIL.ImageTk.PhotoImage(termometro_im)
        self.sonar_photo = PIL.ImageTk.PhotoImage(sonar_im)
	self.gps_photo = PIL.ImageTk.PhotoImage(gps_im)
	self.imu_photo = PIL.ImageTk.PhotoImage(imu_im)
	self.power_photo = PIL.ImageTk.PhotoImage(power_im)
	self.elir_photo = PIL.ImageTk.PhotoImage(elir_im)

        #------------------------------------ System Integrity Title --------------------------------------------------
        tkinter.Label(self.canvas_system_int,text = "System Integrity", font=("Helvetica", 16), bg = self.background, fg = "#284f6d").place(relx = 0.5, rely = 0.015, anchor="c")

        #--------------------------------------- Interface Icons ---------------------------------------------------------     
        # Battery                      
        self.batt100 = self.canvas_system_int.create_image(30, 60, image=self.batt100_photo, anchor=tkinter.NW)
        self.batt_remain = tkinter.Label(self.canvas_system_int,text = "100%", font=("Helvetica", 14), bg = self.background)
        self.batt_remain.place(relx = 0.059, rely = 0.25, anchor="c")

        # Termometer
        self.termometro = self.canvas_system_int.create_image(140, 60, image=self.termometro_photo, anchor=tkinter.NW)
        self.temp = tkinter.Label(self.canvas_system_int,text = "25 C", font=("Helvetica", 14),bg = self.background)
        self.temp.place(relx = 0.20, rely = 0.25, anchor="c")

        # Sonar
        self.sonar = self.canvas_system_int.create_image(230, 60, image=self.sonar_photo, anchor=tkinter.NW)
        self.distancia = tkinter.Label(self.canvas_system_int,text = "20 m", font=("Helvetica", 14), bg = self.background)
        self.distancia.place(relx = 0.325, rely = 0.25, anchor="c")

	#GPS
	self.gps = self.canvas_system_int.create_image(360, 60, image=self.gps_photo, anchor=tkinter.NW)
        self.longitude = tkinter.Label(self.canvas_system_int,text = "Longitude: -20 +3", font=("Helvetica", 14), bg = self.background)
        self.longitude.place(relx = 0.5, rely = 0.25, anchor="c")

        self.latitude = tkinter.Label(self.canvas_system_int,text = "Latitude: -20 +3", font=("Helvetica", 14), bg = self.background)
        self.latitude.place(relx = 0.5, rely = 0.3, anchor="c")

	#IMU
	self.imu = self.canvas_system_int.create_image(525, 55, image=self.imu_photo, anchor=tkinter.NW)
        self.pitch = tkinter.Label(self.canvas_system_int,text = "Pitch: -20 +3", font=("Helvetica", 14), bg = self.background)
        self.pitch.place(relx = 0.705, rely = 0.25, anchor="c")

        self.raw = tkinter.Label(self.canvas_system_int,text = "Raw: -20 +3", font=("Helvetica", 14), bg = self.background)
        self.raw.place(relx = 0.705, rely = 0.3, anchor="c")

 	self.yaw = tkinter.Label(self.canvas_system_int,text = "Yaw: -20 +3", font=("Helvetica", 14), bg = self.background)
        self.yaw.place(relx = 0.705, rely = 0.35, anchor="c")

	#Power Info
	self.power = self.canvas_system_int.create_image(670, 55, image=self.power_photo, anchor=tkinter.NW)
        self.current = tkinter.Label(self.canvas_system_int,text = "Current: 10 A", font=("Helvetica", 14), bg = self.background)
        self.current.place(relx = 0.875, rely = 0.25, anchor="c")

        # self.consumption = tkinter.Label(self.canvas_system_int,text = "Consumo: 10 KWh", font=("Helvetica", 14), bg = self.background)
        # self.consumption.place(relx = 0.895, rely = 0.3, anchor="c")

	#ELIR
	self.elir = self.canvas_system_int.create_image(130,270, image=self.elir_photo, anchor=tkinter.NW)
        self.elirstatus = tkinter.Label(self.canvas_system_int,text = "Robot Status", font=("Helvetica", 16), bg = self.background, fg="#284f6d")
        self.elirstatus.place(relx = 0.5, rely = 0.4, anchor="c")

    
    #def callback(self,data):

        #rospy.loginfo('%s', data.data)
        #self.temp.config(text=data.data)

    def callback_temperature(self,data_temperature):

        rospy.loginfo('%.2f', (data_temperature.voltage)*100)
        self.temp.config(text=str(round(data_temperature.voltage*100,2))+' C')

    def callback_sonar(self,data_sonar):

        rospy.loginfo('%.2f', (data_sonar.voltage)*100)
        self.distancia.config(text=str(round(data_sonar.voltage*100,2)) + ' cm')
      
    def callback_proximity1(self, data_proximity):
        if data_proximity.voltage==0:
                self.canvas_circle1.itemconfig(self.claw1,fill='red')
        else:
                self.canvas_circle1.itemconfig(self.claw1,fill='green')

    def callback_proximity2(self, data_proximity):
        if data_proximity.voltage==0:
                self.canvas_circle2.itemconfig(self.claw2,fill='red')
        else:
                self.canvas_circle2.itemconfig(self.claw2,fill='green')

    def callback_proximity3(self, data_proximity):
        if data_proximity.voltage==0:
                self.canvas_circle3.itemconfig(self.claw3,fill='red')
        else:
                self.canvas_circle3.itemconfig(self.claw3,fill='green')

    def callback_proximity4(self, data_proximity):
        if data_proximity.voltage==0:
                self.canvas_circle4.itemconfig(self.claw4,fill='red')
        else:
                self.canvas_circle4.itemconfig(self.claw4,fill='green')

    def callback_proximity5(self, data_proximity):
        if data_proximity.voltage==0:
                self.canvas_circle5.itemconfig(self.claw5,fill='red')
        else:
                self.canvas_circle5.itemconfig(self.claw5,fill='green')
    
    def show_frame(self,data):
        count = 0
        for k in range (0,300):
            for i in range (0,400):
                for j in range (0,3):
                    self.frame_data[k,i,j] = data.data[count]
                    count = count + 1

        self.im_detect = PIL.Image.frombytes('RGB', (self.frame_data.shape[1],self.frame_data.shape[0]), self.frame_data.astype('b').tostring())
        self.photo_detect = PIL.ImageTk.PhotoImage(image=self.im_detect)
        self.canvas_image.create_image(0, 55, image=self.photo_detect, anchor=tkinter.NW)

    def callback_hotspot(self,data):
        hotspot_count = data.data
        #self.listbox.insert('end', "a list entry")

        if data.data > 0:
                now = datetime.datetime.now()
                ocurrences_data = str(hotspot_count) + " hotspot(s) detected at " + str(now.hour) + "h" + str(now.minute)
                #print ocurrences_data
                self.listbox.insert('end', ocurrences_data)
        #else:
                #now = datetime.datetime.now()
                #ocurrences_data = str(hotspot_count) + " hotspot(s) detected at " + str(now.hour) + "h" + str(now.minute)
                #print ocurrences_data
                #self.listbox.insert('end', ocurrences_data)
    
    def callback_battery_info(self,data):
	self.current.config(text=str(round(data.current,1))+' A')
	self.batt_remain.config(text=str(round(data.capacity,1))+' Ah')

        
if __name__ == '__main__':
    
    UserInterface = UserInterface(tkinter.Tk(), "ELIR Dashboard") # Create a window and pass it to the Application object
    rospy.spin()

    

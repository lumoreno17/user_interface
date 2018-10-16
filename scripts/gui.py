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


class UserInterface:


    def __init__(self, window, window_title):

        self.mult_image = 5 #Multiply factor for image size
        
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
        self.title_canvas()
        #------------------------ SYSTEM INTEGRITY CANVAS --------------------
        self.system_int_canvas()
 	#---------------------------------------------------------------------


	#----------------------- ROS CONFIGURATION ---------------------------
	rospy.init_node('listener_tkinter2', anonymous=True)
        rospy.Subscriber('tkinter_data', String, self.callback)
	rospy.Subscriber('sonar', DeviceInfo, self.callback_sonar)


	#----------------------- QUIT BUTTOM ----------------------------------
        quit_button = tkinter.Button(window, text='Close', width=25, command=window.destroy)
        quit_button.place(relx=.5, rely=.97, anchor="c")

       
	#self.window.after(1, self.get_data)
        self.vmax = 8400
        self.vmin = 7600
        self.window.mainloop()

    def video_canvas(self):

        self.canvas_image = tkinter.Canvas(self.window, bg = 'white' , width = self.mult_image*80, height = 14*60, bd = 0, highlightthickness =0)
        self.canvas_image.create_window((0,0), window=self.frame)
        self.canvas_image.place(relx=0.2, rely=0.5, anchor="c")

        tkinter.Label(self.canvas_image,text = "IR Camera", font=("Helvetica", 16), bg=self.background).place(relx = 0.5, rely = 0.15, anchor="c")
        tkinter.Label(self.canvas_image,text = "Binarized Image", font=("Helvetica", 16), bg=self.background).place(relx = 0.5, rely = 0.565, anchor="c")
    

    def title_canvas(self):

        self.canvas_title = tkinter.Canvas(self.window, bg = 'white' , width = 1440, height = 50, bd = 0, highlightthickness =0)
        self.canvas_title.create_window((0,0), window=self.frame)
        self.canvas_title.place(relx=0.5, rely=0.05, anchor="c")

        tkinter.Label(self.canvas_title,text = "Electrical Line Inspection Robot Dashboard", font=("Helvetica", 18), bg = self.background).place(relx = 0.5, 			rely = 0.5, anchor="c")                            


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
        batt100_im = batt100_im.resize((wsize_batt/4,hsize_batt/4), PIL.Image.ANTIALIAS)
        batt75_im = batt75_im.resize((wsize_batt/4,hsize_batt/4), PIL.Image.ANTIALIAS)
        batt50_im = batt50_im.resize((wsize_batt/4,hsize_batt/4), PIL.Image.ANTIALIAS)
        batt25_im = batt25_im.resize((wsize_batt/4,hsize_batt/4), PIL.Image.ANTIALIAS)
        termometro_im = termometro_im.resize((int(wsize_temp/4.3),int(hsize_temp/4.3)), PIL.Image.ANTIALIAS)
        sonar_im = sonar_im.resize((int(wsize_sonar/4.3),int(hsize_sonar/4.3)), PIL.Image.ANTIALIAS)
	gps_im = gps_im.resize((int(wsize_gps/4),int(hsize_gps/4)), PIL.Image.ANTIALIAS)
	imu_im = imu_im.resize((int(wsize_imu/4),int(hsize_imu/4)), PIL.Image.ANTIALIAS)
	power_im = power_im.resize((int(wsize_power/4),int(hsize_power/4)), PIL.Image.ANTIALIAS)
	elir_im = elir_im.resize((int(elir_im.size[0]/1.3),int(elir_im.size[1]/1.3)), PIL.Image.ANTIALIAS)
       
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
        tkinter.Label(self.canvas_system_int,text = "System Integrity", font=("Helvetica", 18), bg = self.background).place(relx = 0.5, rely = 0.05, anchor="c")

        #--------------------------------------- Interface Icons ---------------------------------------------------------     
        # Battery                      
        self.batt100 = self.canvas_system_int.create_image(30, 60, image=self.batt100_photo, anchor=tkinter.NW)
        self.batt_remain = tkinter.Label(self.canvas_system_int,text = "100%", font=("Helvetica", 20), bg = self.background)
        self.batt_remain.place(relx = 0.075, rely = 0.3, anchor="c")

        # Termometer
        self.termometro = self.canvas_system_int.create_image(163, 60, image=self.termometro_photo, anchor=tkinter.NW)
        self.temp = tkinter.Label(self.canvas_system_int,text = "25 C", font=("Helvetica", 20),bg = self.background)
        self.temp.place(relx = 0.23, rely = 0.3, anchor="c")

        # Sonar
        self.sonar = self.canvas_system_int.create_image(260, 60, image=self.sonar_photo, anchor=tkinter.NW)
        self.distancia = tkinter.Label(self.canvas_system_int,text = "20 m", font=("Helvetica", 15), bg = self.background)
        self.distancia.place(relx = 0.38, rely = 0.3, anchor="c")

	#GPS
	self.gps = self.canvas_system_int.create_image(400, 60, image=self.gps_photo, anchor=tkinter.NW)
        self.longitude = tkinter.Label(self.canvas_system_int,text = "Longitude: -20 +3", font=("Helvetica", 15), bg = self.background)
        self.longitude.place(relx = 0.55, rely = 0.3, anchor="c")

        self.latitude = tkinter.Label(self.canvas_system_int,text = "Latitude: -20 +3", font=("Helvetica", 15), bg = self.background)
        self.latitude.place(relx = 0.55, rely = 0.35, anchor="c")

	#IMU
	self.imu = self.canvas_system_int.create_image(545, 55, image=self.imu_photo, anchor=tkinter.NW)
        self.pitch = tkinter.Label(self.canvas_system_int,text = "Pitch: -20 +3", font=("Helvetica", 15), bg = self.background)
        self.pitch.place(relx = 0.75, rely = 0.3, anchor="c")

        self.raw = tkinter.Label(self.canvas_system_int,text = "Raw: -20 +3", font=("Helvetica", 15), bg = self.background)
        self.raw.place(relx = 0.75, rely = 0.35, anchor="c")

 	self.yaw = tkinter.Label(self.canvas_system_int,text = "Yaw: -20 +3", font=("Helvetica", 15), bg = self.background)
        self.yaw.place(relx = 0.75, rely = 0.4, anchor="c")

	#Power Info
	self.power = self.canvas_system_int.create_image(690, 55, image=self.power_photo, anchor=tkinter.NW)
        self.current = tkinter.Label(self.canvas_system_int,text = "Current: 10 A", font=("Helvetica", 15), bg = self.background)
        self.current.place(relx = 0.95, rely = 0.3, anchor="c")

        self.consumption = tkinter.Label(self.canvas_system_int,text = "Consumo: 10 KWh", font=("Helvetica", 15), bg = self.background)
        self.consumption.place(relx = 0.95, rely = 0.35, anchor="c")

	#ELIR
	self.elir = self.canvas_system_int.create_image(100,230, image=self.elir_photo, anchor=tkinter.NW)
        self.elirstatus = tkinter.Label(self.canvas_system_int,text = "ELIR Status", font=("Helvetica", 15), bg = self.background)
        self.elirstatus.place(relx = 0.5, rely = 0.3, anchor="c")

    
    def callback(self,data):

        rospy.loginfo('%s', data.data)
        self.temp.config(text=data.data)

    def callback_sonar(self,data_sonar):

        rospy.loginfo('%f', data_sonar.voltage)
        self.distancia.config(text=str(data_sonar.voltage))
      


if __name__ == '__main__':
    
    UserInterface = UserInterface(tkinter.Tk(), "ELIR Dashboard") # Create a window and pass it to the Application object
    rospy.spin()

    

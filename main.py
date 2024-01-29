import tkinter as tk
from tkinter import ttk
import tkinter.font as tkFont
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import vpython as vp
from vpython import *
import numpy as np
import serial
import json
import datetime
from datetime import *
import time
import csv
import pandas as pd
import threading
import math

selected_integer_value = 0
selected_port = None
ser=None

class BNO055Data:
    def __init__(self):
        self.thetaM = 0.0
        self.phiM = 0.0
        self.thetaFold = 0.0
        self.thetaFnew = 0.0
        self.phiFold = 0.0
        self.phiFnew = 0.0
        self.thetaG = 0.0
        self.phiG = 0.0
        self.theta = 0.0
        self.phi = 0.0
        self.thetaRad = 0.0
        self.phiRad = 0.0
        self.Xm = 0.0
        self.Ym = 0.0
        self.psi = 0.0
        self.dt = 0.0
        self.millisOld = int(time.time() * 1000)


bno055 = BNO055Data()
#toRad = 2 * math.pi / 360
#toDeg = 1 / toRad

def calc_euler(data) -> np.array:
    # sprintf(buffer, "{\"euler\": [%d, %d, %d, %d, %d, %d, %d, %d, %d]}", bno055.acc_x, bno055.acc_y, bno055.acc_z,
    # bno055.gyr_x, bno055.gyr_y, bno055.gyr_z, bno055.mag_x, bno055.mag_y, bno055.mag_z);
    #     012      345      678
    # acc xyz, gyr xyz, mag xyz

    # accelerometer unit:               1 m/s^2 = 100 LSB
    # gyroscope unit:                   1 dps (degree per second) = 16 LSB
    # magnetometer unit:                1 mT = 16 LSB

    accX = data[0] / 100
    accY = data[1] / 100
    accZ = data[2] / 100
    gyrX = (data[3] / 16) * 0.01745329251 # direkt zu rps weil adruino standard in rps ist
    gyrY = (data[4] / 16) * 0.01745329251 # direkt zu rps weil adruino standard in rps ist
    gyrZ = (data[5] / 16) * 0.01745329251 # direkt zu rps weil adruino standard in rps ist
    magX = data[6] / 16
    magY = data[7] / 16
    magZ = data[8] / 16

    # https://toptechboy.com/9-axis-imu-lesson-20-vpython-visualization-of-roll-pitch-and-yaw/
    bno055.thetaM = -(math.atan2(accX / 9.81, accZ / 9.81)) / 2 / math.pi * 360
    bno055.phiM = -(math.atan2(accY / 9.81, accZ / 9.81)) / 2 / math.pi * 360
    bno055.phiFnew = 0.95 * bno055.phiFold + 0.05 * bno055.phiM
    bno055.thetaFnew = 0.95 * bno055.thetaFold + 0.05 * bno055.thetaM

    bno055.dt = (int(time.time() * 1000) - bno055.millisOld) / 1000
    bno055.millisOld = int(time.time() * 1000)

    bno055.theta = (bno055.theta + gyrY * bno055.dt) * 0.95 + bno055.thetaM * 0.05
    bno055.phi = (bno055.phi - gyrX * bno055.dt) * 0.95 + bno055.phiM * 0.05
    bno055.thetaG = bno055.thetaG + gyrY * bno055.dt
    bno055.phiG = bno055.phiG - gyrX * bno055.dt

    bno055.phiRad = bno055.phi / 360 * (2 * math.pi)
    bno055.thetaRad = bno055.theta / 360 * (2 * math.pi)

    bno055.Xm = magX * math.cos(bno055.thetaRad) - magY * math.sin(bno055.phiRad) * math.sin(bno055.thetaRad) + magZ * math.cos(bno055.phiRad) * math.sin(bno055.thetaRad)
    bno055.Ym = magY * math.cos(bno055.phiRad) + magZ * math.sin(bno055.phiRad)

    bno055.psi = math.atan2(bno055.Ym, bno055.Xm) / (2 * math.pi) * 360

    bno055.phiFold = bno055.phiFnew
    bno055.thetaFold = bno055.thetaFnew

    return np.array([bno055.phi, bno055.theta, bno055.psi, accX, accY, accZ, gyrX/0.01745329251, gyrY/0.01745329251, gyrZ/0.01745329251, magX, magY, magZ])


time_option_mapping = {
    "24 Stunden": 24 * 60,  # 24 hours in minutes
    "Eine Minute": 1,  # 1 hour in minutes
    "30 Minuten": 30,  # 30 minutes
    "48 Stunden": 48 * 60  # 48 hours in minutes
}


def filter_data(df, deltatime):
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])  # Convert 'Timestamp' to datetime object
    end_time = datetime.now()  # Get current time
    start_time = end_time - pd.Timedelta(minutes=deltatime)  # Calculate the start time as the end time minus 30 minutes
    filtered_df = df[(df['Timestamp'] >= start_time) & (df['Timestamp'] <= end_time)]  # Filter based on time range
    return filtered_df

def eulerWrite(data):

    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    with open('euler_data.csv', 'a', newline='') as csvfile:
        fieldnames = ['Timestamp', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Gyr_X', 'Gyr_Y', 'Gyr_Z', 'Mag_X', 'Mag_Y', 'Mag_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if csvfile.tell() == 0:  # Check if the file is empty, write header if true
            writer.writeheader()

        try:
            writer.writerow({
                'Timestamp': timestamp,
                'Acc_X': data['euler'][0],
                'Acc_Y': data['euler'][1],
                'Acc_Z': data['euler'][2],
                'Gyr_X': data['euler'][3],
                'Gyr_Y': data['euler'][4],
                'Gyr_Z': data['euler'][5],
                'Mag_X': data['euler'][6],
                'Mag_Y': data['euler'][7],
                'Mag_Z': data['euler'][8]
            })
        except IndexError as e:
            print(f"IndexError: {e}")
        except KeyError as e:
            print(f"keyError: {e}")


def quaternionWrite(data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    with open('quaternion_data.csv', 'a', newline='') as csvfile:
        fieldnames = ['Timestamp', 'Quaternion_W', 'Quaternion_X', 'Quaternion_Y', 'Quaternion_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if csvfile.tell() == 0:  # Check if the file is empty, write header if true
            writer.writeheader()
        # quat w x y z
        try:
            writer.writerow({
                'Timestamp': timestamp,
                'Quaternion_W': data['quaternions'][0],
                'Quaternion_X': data['quaternions'][1],
                'Quaternion_Y': data['quaternions'][2],
                'Quaternion_Z': data['quaternions'][3]
            })

        except IndexError as e:
            print(f"IndexError: {e}")


def show_frame(frame):
    frame.tkraise()

def plots():

    try:
        # Daten aus der CSV-Datei lesen
        df = pd.read_csv('euler_data.csv', parse_dates=['Timestamp'])

        filtered_data = filter_data(df, selected_integer_value)

        # Plot für Acc_X, Acc_Y und Acc_Z erstellen
        fig2 = plt.figure(figsize=(5, 3))
        plt.plot(filtered_data['Timestamp'], filtered_data['Acc_X'], label='Acc_X')
        plt.plot(filtered_data['Timestamp'], filtered_data['Acc_Y'], label='Acc_Y')
        plt.plot(filtered_data['Timestamp'], filtered_data['Acc_Z'], label='Acc_Z')
        plt.title('Acceleration Over Time')
        plt.xlabel('Timestamp')
        plt.ylabel('Acceleration (m/s^2)')
        plt.xticks(fontsize=5)
        plt.legend()
        # plt.show()

        # Plot für Gyr_X, Gyr_Y und Gyr_Z erstellen
        fig3 = plt.figure(figsize=(5, 3))
        plt.plot(filtered_data['Timestamp'], filtered_data['Gyr_X'], label='Gyr_X')
        plt.plot(filtered_data['Timestamp'], filtered_data['Gyr_Y'], label='Gyr_Y')
        plt.plot(filtered_data['Timestamp'], filtered_data['Gyr_Z'], label='Gyr_Z')
        plt.title('Gyroscope Readings Over Time')
        plt.xlabel('Timestamp')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.xticks(fontsize=5)
        plt.legend()
        # plt.show()

        # Plot für Mag_X, Mag_Y und Mag_Z erstellen
        fig4 = plt.figure(figsize=(5, 3))
        plt.plot(filtered_data['Timestamp'], filtered_data['Mag_X'], label='Mag_X')
        plt.plot(filtered_data['Timestamp'], filtered_data['Mag_Y'], label='Mag_Y')
        plt.plot(filtered_data['Timestamp'], filtered_data['Mag_Z'], label='Mag_Z')
        plt.title('Magnetometer Readings Over Time')
        plt.xlabel('Timestamp')
        plt.ylabel('Magnetic Field Strength (uT)')
        plt.xticks(fontsize=5)
        plt.legend()
        # plt.show()

        canvas2 = FigureCanvasTkAgg(fig2, master=frame_2d_systems)
        canvas2.draw()
        canvas2.get_tk_widget().place(relx=0.45, rely=0.55, anchor='ne')

        canvas3 = FigureCanvasTkAgg(fig3, master=frame_2d_systems)
        canvas3.draw()
        canvas3.get_tk_widget().place(relx=0.45, rely=0.45, anchor='se')

        canvas4 = FigureCanvasTkAgg(fig4, master=frame_2d_systems)
        canvas4.draw()
        canvas4.get_tk_widget().place(relx=0.55, rely=0.45, anchor='sw')

        plt.close('all')


    except FileNotFoundError as e:
        print()

    except UnboundLocalError as e:
        print()


def open_visualization(ser):

    # Set the scene
    scene = vp.canvas(frame=frame_3d, width=1200, height=600)
    scene.center = vector(0, 0, 0)
    scene.forward = vector(10, -1, 1)
    scene.background = color.white

    # Create the BNO055 board
    bnoO55 = box(length=0.5, width=0.38, height=0.04, opacity=.75, pos=vec(0, -0.065, 0), color=vec(0.2, 0.2, 0.2))
    bnoO55PCB = box(length=2, width=2.7, height=.13, opacity=.75, pos=vec(0, -0.13, 0), color=vec(0, 0.54, 0.69))
    bnoO55con1 = box(length=0.25, width=1.2, height=1.03, opacity=.75, pos=vec(-0.85, -0.7025, 0),
                     color=vec(0.2, 0.2, 0.2))
    bnoO55con2 = box(length=0.25, width=1.6, height=1.03, opacity=.75, pos=vec(0.85, -0.7025, 0),
                     color=vec(0.2, 0.2, 0.2))

    # Combine the BNO055 components into one object
    bnoO55board = compound([bnoO55PCB,
                            bnoO55,
                            bnoO55con1,
                            bnoO55con2
                            ],
                           pos=vec(0, 0, 0), origin=vec(0, 0, 0))

    # Create the Nordic board
    nordicdkpcb = box(length=6.4, width=13.6, height=.13, opacity=.75, pos=vec(3.2, 0, 6.8), color=vec(0, 0.8, 0.5))
    nrf52840 = box(length=0.7, width=0.7, height=0.04, opacity=.75, pos=vec(1.9, 0.085, 2), color=vec(0.2, 0.2, 0.2))

    # Create the connectors for the Nordic board
    nordicdkcon1 = box(length=.25, width=2.1, height=.7, opacity=.75, pos=vec(0.65, 0.35, 7.85), color=vec(.2, .2, .2))
    nordicdkcon2 = box(length=.25, width=2.1, height=.7, opacity=.75, pos=vec(0.65, 0.35, 5.5), color=vec(.2, .2, .2))
    nordicdkcon3 = box(length=.25, width=1.5, height=.7, opacity=.75, pos=vec(5.54, 0.35, 7.9), color=vec(.2, .2, .2))
    nordicdkcon4 = box(length=2.3, width=0.5, height=.7, opacity=.75, pos=vec(1.9, 0.35, 4), color=vec(.2, .2, .2))

    # Combine the Nordic components into one object
    nordicboard = compound([nordicdkpcb,
                            nrf52840,
                            nordicdkcon1,
                            nordicdkcon2,
                            nordicdkcon3,
                            nordicdkcon4
                            ],
                           pos=vec(-3.2, -2.55, -10), origin=vec(0, 0, 0))

    # Create the bridge board
    bridgepcb1 = box(length=5.3, width=1.8, height=.13, opacity=.75, pos=vec(2.65, 0, 0.9), color=vec(1, 0, 0))
    bridgepcb2 = box(length=1.5, width=0.9, height=.13, opacity=.75, pos=vec(0.75, 0, 2.25), color=vec(1, 0, 0))
    bridgepcb3 = box(length=0.9, width=0.3, height=.13, opacity=.75, pos=vec(4.85, 0, -0.15), color=vec(1, 0, 0))
    bridgei2ccon = box(length=.25, width=2.6, height=1.2, opacity=.75, pos=vec(0.1, -0.685, 1.3), color=vec(.2, .2, .2))
    bridgepwcon = box(length=.25, width=2.1, height=1.2, opacity=.75, pos=vec(5, -0.685, 0.78), color=vec(.2, .2, .2))

    # Combine the bridge components into one object
    bridge = compound([bridgepcb1,
                       bridgepcb2,
                       bridgepcb3,
                       bridgei2ccon,
                       bridgepwcon
                       ],
                      pos=vec(-2.65, -1.275, -0.9), origin=vec(0, 0, 0))

    # Combine all the objects into one
    fhObj = compound([bnoO55board,
                      nordicboard,
                      bridge
                      ],
                     pos=vec(0, 0, -15), origin=vec(0, 0, 0))


    # Create the second BNO055 board
    bnoO55_2 = box(length=0.5, width=0.38, height=0.04, opacity=.75, pos=vec(0, -0.065, 0), color=vec(0.2, 0.2, 0.2))
    bnoO55PCB_2 = box(length=2, width=2.7, height=.13, opacity=.75, pos=vec(0, -0.13, 0), color=vec(0, 0.54, 0.69))
    bnoO55con1_2 = box(length=0.25, width=1.2, height=1.03, opacity=.75, pos=vec(-0.85, -0.7025, 0),
                     color=vec(0.2, 0.2, 0.2))
    bnoO55con2_2 = box(length=0.25, width=1.6, height=1.03, opacity=.75, pos=vec(0.85, -0.7025, 0),
                     color=vec(0.2, 0.2, 0.2))

    # Combine the BNO055 components into one object
    bnoO55board_2 = compound([bnoO55PCB_2,
                            bnoO55_2,
                            bnoO55con1_2,
                            bnoO55con2_2
                            ],
                           pos=vec(0, 0, 0), origin=vec(0, 0, 0))

    # Create the Nordic board
    nordicdkpcb_2 = box(length=6.4, width=13.6, height=.13, opacity=.75, pos=vec(3.2, 0, 6.8), color=vec(0, 0.8, 0.5))
    nrf52840_2 = box(length=0.7, width=0.7, height=0.04, opacity=.75, pos=vec(1.9, 0.085, 2), color=vec(0.2, 0.2, 0.2))

    # Create the connectors for the Nordic board
    nordicdkcon1_2 = box(length=.25, width=2.1, height=.7, opacity=.75, pos=vec(0.65, 0.35, 7.85), color=vec(.2, .2, .2))
    nordicdkcon2_2 = box(length=.25, width=2.1, height=.7, opacity=.75, pos=vec(0.65, 0.35, 5.5), color=vec(.2, .2, .2))
    nordicdkcon3_2 = box(length=.25, width=1.5, height=.7, opacity=.75, pos=vec(5.54, 0.35, 7.9), color=vec(.2, .2, .2))
    nordicdkcon4_2 = box(length=2.3, width=0.5, height=.7, opacity=.75, pos=vec(1.9, 0.35, 4), color=vec(.2, .2, .2))

    # Combine the Nordic components into one object
    nordicboard_2 = compound([nordicdkpcb_2,
                            nrf52840_2,
                            nordicdkcon1_2,
                            nordicdkcon2_2,
                            nordicdkcon3_2,
                            nordicdkcon4_2
                            ],
                           pos=vec(-3.2, -2.55, -10), origin=vec(0, 0, 0))

    # Create the bridge board
    bridgepcb1_2 = box(length=5.3, width=1.8, height=.13, opacity=.75, pos=vec(2.65, 0, 0.9), color=vec(1, 0, 0))
    bridgepcb2_2 = box(length=1.5, width=0.9, height=.13, opacity=.75, pos=vec(0.75, 0, 2.25), color=vec(1, 0, 0))
    bridgepcb3_2 = box(length=0.9, width=0.3, height=.13, opacity=.75, pos=vec(4.85, 0, -0.15), color=vec(1, 0, 0))
    bridgei2ccon_2 = box(length=.25, width=2.6, height=1.2, opacity=.75, pos=vec(0.1, -0.685, 1.3), color=vec(.2, .2, .2))
    bridgepwcon_2 = box(length=.25, width=2.1, height=1.2, opacity=.75, pos=vec(5, -0.685, 0.78), color=vec(.2, .2, .2))

    # Combine the bridge components into one object
    bridge_2 = compound([bridgepcb1_2,
                       bridgepcb2_2,
                       bridgepcb3_2,
                       bridgei2ccon_2,
                       bridgepwcon_2
                       ],
                      pos=vec(-2.65, -1.275, -0.9), origin=vec(0, 0, 0))

    # Combine all the objects into one
    fhObj_2 = compound([bnoO55board_2,
                      nordicboard_2,
                      bridge_2
                      ],
                     pos=vec(0, 0, 15), origin=vec(0, 0, 0))


    while True:
        # Daten vom COM-Port lesen
        line = ser.readline().decode('utf-8').strip()

        try:
            data = json.loads(line)

            if 'euler' in data.keys():

                toRad = 2 * np.pi / 360

                euler = np.array(data['euler'])

                euler = calc_euler(euler)

                roll = euler[1] * toRad # theta
                pitch = euler[0] * toRad  # yaw axis
                yaw = euler[2] * toRad

                k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
                y = vector(0, 1, 0)
                s = cross(k, y)
                v = cross(s, k)
                vrot = v * cos(roll) + cross(k, v) * sin(roll)

                fhObj_2.axis = k
                fhObj_2.up = vrot


            elif 'quaternions' in data.keys():

                #quaternionWrite(data)

                scale = (1.0 / (1 << 14))
                quaternions = np.array(data['quaternions'])
                #print(f"quat: w: {quaternions[0]}, x: {quaternions[1]}, y: {quaternions[2]}, z: {quaternions[3]}")

                qw = quaternions[0]*scale
                qx = quaternions[1]*scale
                qy = quaternions[2]*scale
                qz = quaternions[3]*scale

                sqw = qw * qw
                sqx = qx * qx
                sqy = qy * qy
                sqz = qz * qz

                roll = 0
                yaw = 0
                pitch = 0

                unitLength = qw**2 + qx**2 + qy**2 + qz**2
                abcd = qw*qx + qy*qz

                # added 45 for correction of angle
                if(unitLength != 0):
                    if(abcd > (0.4995)*unitLength):
                        yaw = 2 * atan2(qy, qw) + 45
                        pitch = np.pi/2
                        roll = 0
                    elif(abcd < (-0.4995)*unitLength):
                        yaw = -2*atan2(qy, qw) + 45
                        pitch = -np.pi/2
                        roll = 0
                    else:
                        adbc = qw*qz - qx*qy
                        acbd = qw*qy - qx*qz
                        yaw = -atan2(2*adbc, 1 - 2*(qz**2+qx**2)) + 45
                        pitch = asin(2*abcd/unitLength)
                        roll = atan2(2*acbd, 1 - 2*(qy**2+qx**2))


                k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
                y = vector(0, 1, 0)
                s = cross(k, y)
                v = cross(s, k)
                vrot = v * cos(roll) + cross(k, v) * sin(roll)

                # Die Winkel an das Objekt anpassen
                fhObj.axis = k
                fhObj.up = vrot


        except json.JSONDecodeError as e:
            print(f"Fehler beim Dekodieren der JSON-Daten: {e}")
        except IndexError as e:
            print(f"IndexError: {e}")


def readData(ser):

    try:
        while True:

            line = ser.readline().decode('utf-8').strip()
            data = json.loads(line)
            if 'euler' in data:
                eulerWrite(data)

    except json.JSONDecodeError as e:
        print(f"Fehler beim Dekodieren der JSON-Daten: {e}")


def on_closing():
    try:
        ser.close()  # Close the serial port
    except Exception as e:
        print(f"Fehler beim schließen des com-ports: {e}")

    root.destroy()  # Destroy the Tkinter root window, ending the mainloop

def on_combobox_change(event):
    global selected_integer_value
    button_choice.pack(pady=10)
    selected_time_option = time_combobox.get()
    selected_integer_value = time_option_mapping.get(selected_time_option, 0)

def on_combobox_comport_select(event):
    global selected_port
    selected_port = combobox_comport.get()
    # You can perform any action with the selected COM port here, e.g., print it
    print(f"Selected COM port: {selected_port}")


def open_serial_port():
    global ser
    ser=serial.Serial(selected_port, 115200)
    thread = threading.Thread(target=readData, args=(ser,))
    thread.start()


if __name__ == '__main__':

    root = tk.Tk()
    root.geometry('1200x900')  # Setzt die Größe des Fensters auf 1200x900
    root.configure(bg='white')  # Setzt die Hintergrundfarbe auf Weiß für den hellen Modus

    # Hauptframe
    main_frame = tk.Frame(root, bg='white')
    main_frame.place(relwidth=1, relheight=1)

    # 3D-Visualisierungs-Frame
    frame_3d = tk.Frame(root, bg='white')
    frame_3d.place(relwidth=1, relheight=1)

    # 2D-Koordinatensystem-Frame
    frame_2d = tk.Frame(root, bg='white')
    frame_2d.place(relwidth=1, relheight=1)

    # 2D-Koordinatensysteme-Frame
    frame_2d_systems = tk.Frame(root, bg='white')
    frame_2d_systems.place(relwidth=1, relheight=1)

    # Welcome-Frame
    frame_welcome = tk.Frame(root, bg='white')
    frame_welcome.place(relwidth=1, relheight=1)

    # Button-Frame im Hauptframe
    button_frame = tk.Frame(main_frame, bg='white')
    button_frame.place(relx=0.5, rely=0.5, anchor='center')

    helv36 = tkFont.Font(family='Helvetica', size=24, weight='bold')

    button_3d = tk.Button(button_frame, text="3D-Visualisierung", command=lambda: open_visualization(ser), bg='white',
                          fg='black', height=10, width=40, font=helv36)
    button_3d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

    time_options = ["24 Stunden", "Eine Minute", "30 Minuten", "48 Stunden"]
    time_combobox = ttk.Combobox(frame_2d, values=time_options, state="readonly")
    time_combobox.pack(pady=10)

    # Auswahl-Button, der erscheint, wenn eine Option in der Combobox ausgewählt wurde
    button_choice = tk.Button(frame_2d, text="Auswahl", command=lambda: [show_frame(frame_2d_systems), plots()],
                              bg='white', fg='black')

    time_combobox.bind("<<ComboboxSelected>>", on_combobox_change)
    button_2d = tk.Button(button_frame, text="2D-Koordinatensystem", command=lambda: show_frame(frame_2d), bg='white',
                          fg='black', height=10, width=40, font=helv36)
    button_2d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

    # Zurück-Buttons für die Frames frame_3d und frame_2d
    back_button_3d = tk.Button(frame_3d, text="Zurück", command=lambda: show_frame(main_frame), bg='white', fg='black')
    back_button_3d.place(x=0, y=0)  # Positioniert den Button in der linken oberen Ecke

    back_button_2d = tk.Button(frame_2d, text="Zurück", command=lambda: show_frame(main_frame), bg='white', fg='black')
    back_button_2d.place(x=0, y=0)  # Positioniert den Button in der linken oberen Ecke

    # Combobox für den Zeitraum der Datenauswertung
    label_time = tk.Label(frame_2d, text="Zeitraum der Datenauswertung", font=helv36)
    label_time.pack(pady=10)

    # Zurück-Button für den Frame frame_2d_systems
    back_button_2d_systems = tk.Button(frame_2d_systems, text="Zurück", command=lambda: show_frame(frame_2d),
                                       bg='white', fg='black')
    back_button_2d_systems.place(x=0, y=0)  # Positioniert den Button in der linken oberen Ecke

    # Com-port frame elemente
    com_ports = [f"com{i}" for i in range(21)]
    combobox_comport = ttk.Combobox(frame_welcome, values=com_ports, state="readonly")
    combobox_comport.pack(pady=10)
    combobox_comport.set("Select COM port")  # Set a default value

    comport_button = tk.Button(frame_welcome, text="Auswahl", command=lambda: [show_frame(main_frame), open_serial_port()] , bg='white', fg='black')
    comport_button.pack(pady=10)

    # Bind the on_combobox_select function to the <<ComboboxSelected>> event
    combobox_comport.bind("<<ComboboxSelected>>", on_combobox_comport_select)

    frame_welcome.pack(expand=True, fill='both')

    # Zeigen Sie den Hauptframe zuerst an
    show_frame(frame_welcome)

    # Bind the closing event to the on_closing function
    root.protocol("WM_DELETE_WINDOW", on_closing)

    root.mainloop()

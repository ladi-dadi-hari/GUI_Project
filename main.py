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
import time



def show_frame(frame):
    frame.tkraise()

def open_scene():

    ser = serial.Serial('com10', 115200)

    # 3D-Visualisierungs-Frame
    frame_3d = tk.Frame(root, bg='white')
    frame_3d.place(relwidth=1, relheight=1)
    frame_3d.pack()

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

    try:
        while True:
            # Daten vom COM-Port lesen
            line = ser.readline().decode('utf-8').strip()

            if not line:
                continue

            # Überprüfen, ob die Daten im JSON-Format vorliegen

            try:
                data = json.loads(line)

                if 'euler' in data:

                    toRad = 2 * np.pi / 360
                    toDeg = 1 / toRad

                    # Zeit:
                    local_time = time.localtime()
                    dt = datetime.datetime.now()

                    euler = np.array(data['euler'])
                    print(f"euler: x: {euler[0]}, y: {euler[1]}, z: {euler[2]}")
                    #print(line)

                    #orient_vec = vector(euler[0], euler[1], euler[2])
                    #fhObj_2.axis = orient_vec

                    #acc_data = np.array([euler[3], euler[4], euler[5]])
                    #gyro_data = np.array([euler[6], euler[7], euler[8]])

                    #print(acc_data)
                    #print(gyro_data)

                    # orientation_angles = np.zeros(3)
                    # for i in range(3):
                    #     orientation_angles[i] = orientation_angles[i] + gyro_data[i] * dt.microsecond / 100
                    # # Convert the accelerometer data to angles
                    # acceleration_angles = np.zeros(3)
                    # acceleration_angles[0] = np.arctan2(acc_data[1], acc_data[2])
                    # acceleration_angles[1] = np.arctan2(-acc_data[0], np.sqrt(
                    #     acc_data[1] ** 2 + acc_data[2] ** 2))
                    #
                    # # Combine the orientation angles and accelerometer angles using a complementary filter
                    # # Der Wert von alpha bestimmt das Verhältnis zwischen den Integrationsergebnissen
                    # # und den Beschleunigungsmesswerten. Ein höherer Wert von alpha bedeutet, dass die Integrationsergebnisse stärker gewichtet werden.
                    # alpha = 0.98
                    # for i in range(3):
                    #     orientation_angles[i] = alpha * orientation_angles[i] + (1.0 - alpha) * acceleration_angles[i]

                    rate(50)
                    #print("mydata:")
                    #print(orientation_angles)

                    #orient_vec = vector(orientation_angles[0], orientation_angles[1], orientation_angles[2])
                    #fhObj_2.axis = orient_vec

                    # orient_vec = vector(orientation_angles[0], orientation_angles[1], orientation_angles[2])
                    # fhObj_2.axis = orient_vec



                    # x_angle_rad = euler[0] * (3.14159 / 180.0)
                    # y_angle_rad = euler[1] * (3.14159 / 180.0)
                    # z_angle_rad = euler[2] * (3.14159 / 180.0)
                    #
                    # fhObj_2.rotate(angle=x_angle_rad, axis=vector(1, 0 , 0))
                    # fhObj_2.rotate(angle=y_angle_rad, axis=vector(0, 1 , 0))
                    # fhObj_2.rotate(angle=z_angle_rad, axis=vector(0, 0 , 1))


                    roll = euler[1]*toRad
                    pitch = euler[0]*toRad
                    yaw = euler[2]*toRad+np.pi + 180
                    k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
                    y = vector(0, 1, 0)
                    s = cross(k, y)
                    v = cross(s, k)
                    vrot = v * cos(roll) + cross(k, v) * sin(roll)

                    fhObj_2.axis = k
                    fhObj_2.up = vrot



                elif 'quaternions' in data:
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

                    #calc of the angles
                    # pitch = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
                    # roll = math.asin(2 * (qw * qy - qz * qx))
                    # yaw = -math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) - np.pi / 2


                    rate(50)
                    k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
                    y = vector(0, 1, 0)
                    s = cross(k, y)
                    v = cross(s, k)
                    vrot = v * cos(roll) + cross(k, v) * sin(roll)

                    # Die Winkeln an das Objekt anpassen
                    fhObj.axis = k
                    fhObj.up = vrot


            except json.JSONDecodeError as e:
                print(f"Fehler beim Dekodieren der JSON-Daten: {e}")

    except KeyboardInterrupt:
        # Das Programm beenden, wenn Strg+C gedrückt wird
        ser.close()
        print("Programm beendet.")


# Erstellen Sie sechs Subplots
fig, axs = plt.subplots(3, 3, figsize=(10, 6), sharex=True, sharey=True)

# Erstellen Sie eine Liste von Farben für die Plots
colors = ['red', 'red', 'red', 'green', 'green', 'green','blue', 'blue', 'blue']

# Füllen Sie jeden Subplot mit Dummy-Daten
for i, ax in enumerate(axs.flat):
    x = [1, 2, 3, 4, 5]
    y = [i * j for j in x]
    ax.plot(x, y, color=colors[i])
    ax.set_title(f'Plot {i+1}')

#ax.set_title(f'Accelerometer x')

# Fügen Sie einen gemeinsamen Titel hinzu
fig.suptitle('2D-Koordinatensysteme', fontsize=16)


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

# Fügen Sie die Plots in das Tkinter-Frame ein
canvas = FigureCanvasTkAgg(fig, master=frame_2d_systems)
canvas.draw()
canvas.get_tk_widget().place(relx=0.5, rely=0.5, anchor='center')
#canvas.get_tk_widget().pack()

# Button-Frame im Hauptframe
button_frame = tk.Frame(main_frame, bg='white')
button_frame.place(relx=0.5, rely=0.5, anchor='center')

helv36 = tkFont.Font(family='Helvetica', size=24, weight='bold')

button_3d = tk.Button(button_frame, text="3D-Visualisierung", command=lambda: open_scene(), bg='white', fg='black', height= 10, width=40, font=helv36)
button_3d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

button_2d = tk.Button(button_frame, text="2D-Koordinatensystem", command=lambda: show_frame(frame_2d), bg='white', fg='black', height= 10, width=40, font=helv36)
button_2d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

# Fügen Sie hier den Inhalt für jeden Frame hinzu
#label_3d = tk.Label(frame_3d, text="3D-Visualisierung")
#label_3d.pack()

#label_2d = tk.Label(frame_2d, text="2D-Koordinatensystem")
#label_2d.pack()

# Zurück-Buttons für die Frames frame_3d und frame_2d
back_button_3d = tk.Button(frame_3d, text="Zurück", command=lambda: show_frame(main_frame), bg='white', fg='black')
back_button_3d.place(x=0, y=0)  # Positioniert den Button in der linken oberen Ecke

back_button_2d = tk.Button(frame_2d, text="Zurück", command=lambda: show_frame(main_frame), bg='white', fg='black')
back_button_2d.place(x=0, y=0) # Positioniert den Button in der linken oberen Ecke

# Combobox für den Zeitraum der Datenauswertung
label_time = tk.Label(frame_2d, text="Zeitraum der Datenauswertung", font=helv36)
label_time.pack(pady=10)

time_options = ["24 Stunden", "Eine Stunde", "30 Minuten", "48 Stunden"]
time_combobox = ttk.Combobox(frame_2d, values=time_options)
time_combobox.pack(pady=10)

# Auswahl-Button, der erscheint, wenn eine Option in der Combobox ausgewählt wurde
button_choice = tk.Button(frame_2d, text="Auswahl", command=lambda: show_frame(frame_2d_systems), bg='white', fg='black')
def on_combobox_change(event):
    button_choice.pack(pady=10)

time_combobox.bind("<<ComboboxSelected>>", on_combobox_change)

# Zurück-Button für den Frame frame_2d_systems
back_button_2d_systems = tk.Button(frame_2d_systems, text="Zurück", command=lambda: show_frame(frame_2d), bg='white', fg='black')
back_button_2d_systems.place(x=0, y=0)  # Positioniert den Button in der linken oberen Ecke

# Zeigen Sie den Hauptframe zuerst an
show_frame(main_frame)

root.mainloop()

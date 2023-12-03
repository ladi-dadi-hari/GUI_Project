import tkinter as tk
from tkinter import ttk

def show_frame(frame):
    frame.tkraise()

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

# Button-Frame im Hauptframe
button_frame = tk.Frame(main_frame, bg='white')
button_frame.place(relx=0.5, rely=0.5, anchor='center')

button_3d = tk.Button(button_frame, text="3D-Visualisierung", command=lambda: show_frame(frame_3d), bg='white', fg='black')
button_3d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

button_2d = tk.Button(button_frame, text="2D-Koordinatensystem", command=lambda: show_frame(frame_2d), bg='white', fg='black')
button_2d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

# Fügen Sie hier den Inhalt für jeden Frame hinzu
#label_3d = tk.Label(frame_3d, text="3D-Visualisierung")
#label_3d.pack()

#label_2d = tk.Label(frame_2d, text="2D-Koordinatensystem")
#label_2d.pack()

# Zurück-Buttons für die Frames frame_3d und frame_2d
back_button_3d = tk.Button(frame_3d, text="Zurück", command=lambda: show_frame(main_frame), bg='white', fg='black')
back_button_3d.pack(anchor='nw')  # Positioniert den Button in der linken oberen Ecke

back_button_2d = tk.Button(frame_2d, text="Zurück", command=lambda: show_frame(main_frame), bg='white', fg='black')
back_button_2d.pack(anchor='nw')  # Positioniert den Button in der linken oberen Ecke

# Combobox für den Zeitraum der Datenauswertung
label_time = tk.Label(frame_2d, text="Zeitraum der Datenauswertung")
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
back_button_2d_systems.pack(anchor='nw')  # Positioniert den Button in der linken oberen Ecke

# Zeigen Sie den Hauptframe zuerst an
show_frame(main_frame)

root.mainloop()

import tkinter as tk
import time

def show_frame(frame):
    frame.tkraise()

root = tk.Tk()
root.geometry('1200x900')  # Setzt die Größe des Fensters auf 1200x900
root.configure(bg='white')  # Setzt die Hintergrundfarbe auf Weiß für den hellen Modus

# Willkommensbildschirm
welcome_frame = tk.Frame(root, bg='white')
welcome_frame.grid(row=0, column=0, sticky='news')
welcome_label = tk.Label(welcome_frame, text="3D-Visualisierung", bg='white')
welcome_label.pack(pady=450)  # Zentriert den Text vertikal

# Hauptframe
main_frame = tk.Frame(root, bg='white')
main_frame.grid(row=0, column=0, sticky='news')
button_3d = tk.Button(main_frame, text="3D-Visualisierung", command=lambda: None, bg='white')  # Fügen Sie Ihre Funktion hier ein
button_3d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu
button_2d = tk.Button(main_frame, text="2D-Koordinatensystem", command=lambda: None, bg='white')  # Fügen Sie Ihre Funktion hier ein
button_2d.pack(pady=10)  # Zentriert den Button vertikal und fügt einen Abstand zwischen den Buttons hinzu

# Zeige zuerst den Willkommensbildschirm
show_frame(welcome_frame)

# Wechsle nach 3 Sekunden zum Hauptframe
root.after(3000, show_frame, main_frame)

root.mainloop()

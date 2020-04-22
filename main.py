from tkinter import *
import threading

program_mode = 0
lights_mode = 0


class App:
    def __init__(self, tk_root):
        self.root = tk_root
        self.program_mode_int_var = None
        self.lights_mode_int_var = None

    def update_variables(self):
        global program_mode, lights_mode
        program_mode = self.program_mode_int_var.get()
        lights_mode = self.lights_mode_int_var.get()

    def create_window(self):
        self.root.title('Robot car')
        self.root.geometry("400x300+10+10")
        self.program_mode_int_var = IntVar()
        self.lights_mode_int_var = IntVar()
        self.program_mode_int_var.set(0)
        self.lights_mode_int_var.set(0)

        r1 = Radiobutton(self.root, text="Line follower",
                         variable=self.program_mode_int_var, value=0,
                         command=self.update_variables())
        r2 = Radiobutton(self.root, text="Remote control",
                         variable=self.program_mode_int_var, value=1,
                         command=self.update_variables())
        r1.place(x=80, y=50)
        r2.place(x=200, y=50)

        c1 = Checkbutton(self.root, text="Turn on lights",
                         variable=self.lights_mode_int_var,
                         command=self.update_variables())
        c1.place(x=120, y=70)


if __name__ == "__main__":
    window = Tk()
    application = App(window)
    application.create_window()
    while True:
        try:
            window.update()
        except Exception as ex:
            break
        application.update_variables()
        print(program_mode, lights_mode)

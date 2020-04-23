from tkinter import *
import serial
import sys
import time

program_mode = 0
lights_mode = 0
port = "/dev/tty.HC-06-DevB"


class App:
    def __init__(self, tk_root):
        """
        Creates robot control desktop app.
        :param tk_root: Tkinter object.
        """
        self.root = tk_root
        self.program_mode_int_var = None
        self.lights_mode_int_var = None

    def update_variables(self):
        """
        Update global variables by user inputs.
        """
        global program_mode, lights_mode
        program_mode = self.program_mode_int_var.get()
        lights_mode = self.lights_mode_int_var.get()

    def create_window(self):
        """
        Creates window with controls.
        """
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

        assert self.root is not None, "App is not created."


class Bluetooth:
    def __init__(self):
        """
        Bluetooth pipeline with robot.
        """
        self.blue = None

    def connect(self):
        """
        Connect app to device via bluetooth port, baud rate 9600.
        """
        try:
            self.blue = serial.Serial(port, 9600)
        except OSError:
            print("Unable to connect bluetooth device.")
            self.blue = None
        assert self.blue is not None, "App cannot be connected."
        self.blue.flushInput()  # Flush input buffer.


    def send_msg(self, msg):
        """
        Send msg via bluetooth, encode it to bytes.
        :param msg: string message to be sent.
        """
        assert len(msg) != 0
        self.blue.write(b"" + str.encode(str(program_mode)+str(lights_mode)))
        time.sleep(0.2)  # A pause between bursts

    def read_msg(self):
        """
        Read message from bluetooth device.
        :return: string message.
        """
        input_msg = self.blue.readline()
        assert input_msg is not None
        return input_msg.decode()

    def kill(self):
        """
        Close bluetooth connection.
        """
        self.blue.close()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]

    window = Tk()
    application = App(window)
    print("App created.")
    application.create_window()
    bluetooth = Bluetooth()
    bluetooth.connect()
    print("Bluetooth connected.")

    while True:
        try:
            window.update()
        except Exception as ex:
            break
        application.update_variables()
        print(program_mode, lights_mode)
        bluetooth.send_msg(str(program_mode)+str(lights_mode))

    if bluetooth.blue:
        bluetooth.kill()
    print("Closing.")

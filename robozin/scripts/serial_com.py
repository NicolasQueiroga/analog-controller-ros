from time import sleep
import serial
import logging


class SerialControllerInterface:
    def __init__(self):
        logging.basicConfig(level=print)
        self.ser = serial.Serial("/dev/rfcomm0", 9600)
        self.incoming = "0"
        self.head = b""
        self.connected = False
        self.analog_val = {b"X": -1, b"Y": -1}
        self.SPD_UP = False
        self.SPD_DOWN = False
        self.TURN_ARROUND = False

    def set_spd_up(self, value):
        self.SPD_UP = value

    def set_spd_down(self, value):
        self.SPD_DOWN = value

    def set_turn_around(self, value):
        self.TURN_ARROUND = value

    def get_turn_around(self):
        return self.TURN_ARROUND

    def get_connected(self):
        return self.connected

    def connecting(self):
        while self.incoming != b"Q":
            self.incoming = self.ser.read()
            print("Received INCOMING: {}".format(self.incoming))

        self.head = self.ser.read()
        D1 = self.ser.read()
        D0 = self.ser.read()

        print("Received INCOMING to connect: {0}, {1}, {2}".format(self.head, D1, D0))

        if self.head == b"C" and D0 == b"1":
            self.ser.write(b"C")
            sleep(0.1)
            self.ser.write(b"0")
            sleep(0.1)
            self.ser.write(b"1")
            sleep(0.1)
            self.ser.write(b"Q")
            sleep(0.1)
            self.connected = True
            print('CONECTED')

        self.incoming = self.ser.read()

    def get_com_data(self):
        while self.incoming != b"Q":
            self.incoming = self.ser.read()
            print("Received INCOMING: {}".format(self.incoming))

        self.head = self.ser.read()

        if self.head == b"C":
            D0 = self.ser.read()
            D1 = self.ser.read()
            self.connected = False
            return

        if self.head == b"X" or self.head == b"Y":
            D0 = self.ser.read()
            D1 = self.ser.read()
            analog_info = int.from_bytes(D0 + D1, "big")
            self.analog_val[self.head] = analog_info
        else:
            D1 = self.ser.read()
            D0 = self.ser.read()
            if D0 == b"\x01":
                if self.head == b"U":
                    self.SPD_UP = True
                if self.head == b"D":
                    self.SPD_DOWN = True
                if self.head == b"R":
                    self.TURN_ARROUND = True

                logging.info(f"KEYDOWN {self.head}")
            elif D0 == b"\x00":
                logging.info(f"KEYUP {self.head}")

        self.incoming = self.ser.read()

    def send_lidar_data(self, head, data):
        D0 = int(data).to_bytes(1, "big")

        self.ser.write(head)
        sleep(0.1)
        self.ser.write(b"0")
        sleep(0.1)
        self.ser.write(D0)
        sleep(0.1)
        self.ser.write(b"Q")
        sleep(0.1)

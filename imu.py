# IMU functions
# import itertools
# import time
# import winreg
import serial
import glob
import struct
import sys

class IMU:
    def __init__(self, port, address):
        self.serial_port = port
        self.address = address

    def calibrate(self):
        msg = ">" + str(self.address) + ",165\n".encode()
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = readData(self.serial_port)
                return dados
                # teste 3
            else:
                return 0
        except ValueError:
            return 0

    def setEulerToYXZ(self):
        msg = ">" + str(self.address) + ",16,1\n".encode()
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 0
        except ValueError:
            return 0

    def tare(self):
        msg = ">" + str(self.address) + ",96\n".encode()
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 0
        except ValueError:
            return 0

    def sync_timestamp(self):
        msg = ">182\n"
        try:
            if self.serial_port is not None:
                self.serial_port.flush()
                self.serial_port.write(msg)
                return 0
            else:
                return "Port error"
        except:
            return "Error"

    def reset_timestamp(self):
        msg = ">" + str(self.address) + ",95,0\n"
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
            else:
                return "Port error"
        except:
            return "Error"


    def checkButtons(self):

        try:
            if self.serial_port is not None:
                self.serial_port.write(
                    (">" + str(self.address) + ",250\n".encode()))  # Get button state) # e escreve na porta
                dados = readData(self.serial_port)
                botao = dados.split(",")
                if len(botao) == 4:
                    botao = botao[3]
                    if (int(botao) == 1):
                        return 1
                    elif (int(botao) == 2):
                        return 2
                    else:
                        return 0

        except ValueError:
            return 'Error'

    def getEulerAngles(self):
        msg = ">" + str(self.address) + ",1\n".encode()
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 'Port error'
        except ValueError:
            return 'Error'

    def getUntaredQuaternions(self):
        msg = ">" + str(self.address) + ",6\n".encode()
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 'Port error'
        except ValueError:
            return 'Error'

    def getUntaredQuaternionsAsBytes(self):
        msg = "\xF8\x02\x06\x08"
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = bytearray(self.serial_port.read(19))

                while len(dados) != 19:
                    self.serial_port.write(msg)  # e escreve na porta
                    dados = bytearray(self.serial_port.read(19))

                a = [dados[3], dados[4], dados[5], dados[6]]
                b = ''.join(chr(i) for i in a)
                x = struct.unpack('>f', b)
                a = [dados[7], dados[8], dados[9], dados[10]]
                b = ''.join(chr(i) for i in a)
                y = struct.unpack('>f', b)
                a = [dados[11], dados[12], dados[13], dados[14]]
                b = ''.join(chr(i) for i in a)
                z = struct.unpack('>f', b)
                a = [dados[15], dados[16], dados[17], dados[18]]
                b = ''.join(chr(i) for i in a)
                w = struct.unpack('>f', b)
                return [x[0], y[0], z[0], w[0]]
            else:
                return 'Port error'
        except ValueError:
            return 'Error'

    def startStreaming(self):
        msg_config_timing = ">" + str(self.address) + ",82,1000,0,0\n".encode()
        msg_config_slots = ">" + str(self.address) + ",80,0,255,255,255,255,255,255,255\n".encode()

        # config header: timestamp and logical ID
        msg_config_header = bytearray()
        msg_config_header.append(0xf7)
        msg_config_header.append(0xdb)
        msg_config_header.append(0x00)
        msg_config_header.append(0x00)
        msg_config_header.append(0x00)
        msg_config_header.append(0x12)
        msg_config_header.append(0xed)

        msg = bytearray()
        msg.append(0xfa)  # initial byte
        msg.append(self.address)  # sensor address
        msg.append(0x55)  # start stream command
        msg.append(sum(msg[1:]))  # checksum
        # msg = "\xF8\x02\x55\x57"  # start streaming
        try:
            if self.serial_port is not None:
                self.serial_port.flush()
                self.serial_port.write(msg_config_timing)  # configura o streaming
                self.serial_port.write(msg_config_slots)  # configura o streaming
                self.serial_port.write(msg_config_header) # configura o header
                # print readData(self.serial_port)
                self.serial_port.write(msg)  # start stream
                return 0
            else:
                return 'Port error'
        except ValueError:
            return 'Error'

    def stop_streaming(self):
        # stop
        msg_stop = bytearray()
        msg_stop.append(0xfa)
        msg_stop.append(self.address)
        msg_stop.append(0x56)
        msg_stop.append(sum(msg_stop[1:]))
        self.serial_port.write(msg_stop)  # stop streaming

    def getGyroData(self):
        msg = ">" + str(self.address) + ",33\n".encode()
        try:
            if self.serial_port is not None:
                self.serial_port.write(msg)  # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 'Port error'
        except ValueError:
            return 'Error'

    def singleCommand(self, command):
        try:
            if self.serial_port is not None:
                self.serial_port.write(">" + str(self.address) + "," + command + "\n")  # e escreve na porta
                dados = readData(self.serial_port)
                dados = dados.split(",")
                if int(dados[0]) == 0:
                    return dados
                else:
                    return "No answer"
            else:
                return 'Port error'

        except ValueError:
            return 'Error'

    def listen_streaming(self):
        # listening:
        # print 'listening'
        # self.serial_port.flush()
        for i in range(0, 1):
            cont = 0
            while self.serial_port.inWaiting() == 0:
                # print 'waiting'
                cont = cont + 1
                if cont > 1000:
                    break
                # print cont
            dados = bytearray(self.serial_port.read(self.serial_port.inWaiting()))

            length = len(dados)
            # print length
            response = ""
            for i in range(0,len(dados)):
                response = response + str(dados[i]) + ' '
            # print response
            # print length
            if length < 21:
                if length == 0:
                    return 'no answer'
                else:
                    print length
                    print response
                    return 'short data'
            while length >= 21:
                # print 'entrou'
                # if dados[0] == 0:
                # print timestamp
                # size = dados[2]
                #teste
                address = dados[4]
                t = ''.join(chr(i) for i in dados[0:4])
                timestamp = struct.unpack('>I',t)[0]
                timestamp = str(timestamp)
                x = bytesToFloat(dados[5:9])
                y = bytesToFloat(dados[9:13])
                z = bytesToFloat(dados[13:17])
                w = bytesToFloat(dados[17:21])
                # print str(address) + ": " + str(time.time()) + ',' + str(x) + ',' + str(y) + ',' + str(z) + ',' + str(w)
                # The next two lines will be useful if there's more data in the package
                dados = dados[21:]
                length = len(dados)

                return [address, x, y, z, w, timestamp]

def readData(port):
    # dados = ''
    # data = ''
    # i = 1
    # while dados == "":
    #     port.flush()
    #     data = port.read(port.inWaiting()) # le da porta bytearray
    #     dados = data.decode()  # transforma bytearray em string
    #     i += 1
    #     if i > 700:
    #         dados = 'No answer'
    #         break
    i = 1
    while port.inWaiting() == 0:
        # print "Waiting for response"
        i = i + 1
        if i > 10000:
            return "No response"
            # pass

    data = port.read(port.inWaiting())  # le da porta bytearray
    # print data
    return data
    # return dados

def bytesToFloat(args):
    b = ''.join(chr(i) for i in args)
    result = struct.unpack('>f', b)
    return result[0]

def get_port():
    """Get the serial port where the device is connected. Only available on Windows and OSX
    :raises EnvironmentError:
        On unsupported or unknown platforms
    :return:
        The serial port where the device is connected
    """
    port = 0
    if sys.platform.startswith('darwin'):
        port = glob.glob('/dev/tty.usbmodem*')[0]
    elif sys.platform.startswith('win'):
        # path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        # try:
        #     key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        # except winreg.WindowsError:
        #     raise winreg.IterationError
        #
        # for i in itertools.count():
        #     try:
        #         val = winreg.EnumValue(key, i)
        #         yield (str(val[1]), str(val[0]))
        #     except EnvironmentError:
        #         break
        ports = ['COM%s' % (i + 1) for i in range(32)]
        for p in ports:
            try:
                s = serial.Serial(p)
                s.close()
                port = p
            except (OSError, serial.SerialException):
                pass
    return port
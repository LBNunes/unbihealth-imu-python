#IMU functions
#import serial

class IMU:
    def __init__(self, port, address):     
        self.serial_port = port
        self.address = address

########################################
# Calibration
########################################
        
    def calibrate(self):
        msg = ">" + str(self.address) + ",165\n".encode()
        try:                
            if self.serial_port is not None:
                self.serial_port.write(msg) # e escreve na porta
                dados = readData(self.serial_port)
                return dados
                #teste 3
            else:
                return 0              
        except ValueError:
            return 0
            
            
########################################
# Set euler to YXZ
########################################
        
    def setEulerToYXZ(self):
        msg = ">" + str(self.address) + ",16,1\n".encode()
        try:                
            if self.serial_port is not None:
                self.serial_port.write(msg) # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 0               
        except ValueError:
            return 0
            
            
########################################
# Tare with current orientation
########################################
        
    def tare(self):
        msg = ">" + str(self.address) + ",96\n".encode()
        try:                
            if self.serial_port is not None:
                self.serial_port.write(msg) # e escreve na porta
                dados = readData(self.serial_port)
                return dados
            else:
                return 0                
        except ValueError:
            return 0




########################################
# Check Buttons
########################################

    def checkButtons(self):
        
        try:  
            if self.serial_port is not None:
                self.serial_port.write((">" + str(self.address) + ",250\n".encode())) #Get button state) # e escreve na porta
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



########################################
# Get Euler Angles
########################################

    def getEulerAngles(self):
        msg = ">" + str(self.address) + ",1\n".encode()        
        try:                
            if self.serial_port is not None:
                self.serial_port.write(msg) # e escreve na porta
                dados = readData(self.serial_port)
                return dados            
            else:
                return 'Port error'               
        except ValueError:
            return 'Error'
            
########################################
# Get Gyro Data
########################################

    def getGyroData(self):
        msg = ">" + str(self.address) + ",33\n".encode()        
        try:                
            if self.serial_port is not None:
                self.serial_port.write(msg) # e escreve na porta
                dados = readData(self.serial_port)
                return dados            
            else:
                return 'Port error'               
        except ValueError:
            return 'Error'
            
            
########################################
# Single Command
########################################
            
    def singleCommand(self, command):
        try:    
            if self.serial_port is not None:
                self.serial_port.write(">" + str(self.address) + "," + command + "\n") # e escreve na porta
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
        return dados
        
def readData(port):
    #dados = ''
    #data = ''
    #i = 1
    # while dados == "":
    #     port.flush()
    #     data = port.read(port.inWaiting()) # le da porta bytearray
    #     dados = data.decode()  # transforma bytearray em string
    #     i += 1
    #     if i > 700:
    #         dados = 'No answer'
    #         break
    while port.inWaiting() == 0:
        #print "Waiting for response"
        pass

    data = port.read(port.inWaiting()) # le da porta bytearray
    # print data
    return data
    #return dados

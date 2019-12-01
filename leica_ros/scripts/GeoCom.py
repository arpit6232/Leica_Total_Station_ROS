from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object
import serial
import time
import pdb

#ser = serial.Serial(
#    port='/dev/ttyUSB0',
#    baudrate=115200,
#    parity=serial.PARITY_NONE,
#    stopbits=serial.STOPBITS_ONE,
#    bytesize=serial.EIGHTBITS
#)
ser = 0

Debug_Level = 0

class ResponseClass(object):
    RC_COM = 0
    TrId = 0
    RC = 0
    parameters = []
    
    def setResponse(self, response):

        # remove the ' from the string, remove the end-line character and split it up
        words = response.replace('\'','').strip().split(',')
        # print words
        if(len(words)>1) :
            self.RC_COM = int(words[1])
            words2 = words[2].split(':')
            self.TrId = int(words2[0])
            self.RC = int(words2[1])

            self.parameters = words[3:len(words)]

            if(self.RC==0 and Debug_Level==1) :
                print('Problem occurred, Error code: ', self.RC)


def SerialRequest(request, length = 0, t_timeout = 3):
    if(Debug_Level==2) :
        print(request)
        
    response = ResponseClass()
    global ser
        
    try : # try method for the case that TS is not connected
        ser.read(ser.inWaiting())

        lf_req = request + '\r\n'
        ser.write(lf_req.encode())
        
        t_start = time.time()
        # do as long as:
        # 1: buffer has specific length
        # 2: if specific length not defined (=0), then until buffer > 0
        # 3: timeout not reached
    
        while((ser.inWaiting()<length or (length == 0 and ser.inWaiting()==0)) and time.time()-t_start<t_timeout) :
            time.sleep(0.001)
    
        if(time.time()-t_start>=t_timeout) :
            response.RC = 3077
            return response

        time.sleep(0.025)	# Short break to make sure serial port is not read while stuff is written

        serial_output = ser.read(ser.inWaiting())
        output_str = "".join( chr(x) for x in bytearray(serial_output))
        response.setResponse(output_str)
        
        if(Debug_Level==2) :
            print(serial_output)


    except :
        print("Leica TS communication error - not connected?")
        response.RC = 1
        
    return response

def HexToDec(hex_in):

    dec_out = int(hex_in, 16)
    
    return dec_out


def CreateRequest(cmd, args):

    request = '%R1Q,'
    request = request + str(cmd)
    request = request + ':'
    
    if(len(args)>0) :
        for i in range(0,len(args)-1) :
            request = request + str(args[i])
            request = request + ','
    
        request = request + str(args[-1])
    
    return request


def COM_OpenConnection(ePort, eRate, nRetries=5):
    
    global ser
    try :
        ser = serial.Serial(
            port=ePort,
            baudrate=eRate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
    
        while(not ser.isOpen()) :
            ser.open()
            if(not ser.isOpen()) :
                ser.close()

        if(not ser.isOpen() and Debug_Level==1) :
            print('Problem opening port')
        
        # 0 = everything ok
        return [not ser.isOpen(),ser,0]
    
    except :
        print("Connection Error - Leica TS not connected?")
        return [1,0,[]]


def COM_CloseConnection():
    
    global ser
    ser.close()
    
    
    if(not ser.isOpen() and Debug_Level==1) :
        print('Problem closing port')
        
    return [ser.isOpen(),0,[]]


def COM_SwitchOnTPS(eOnMode=2) :
    
    request = CreateRequest('111',[eOnMode])
    
    response = SerialRequest(request)

    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Turn on TPS') 
        
    elif(response.RC==5) :
        error = 0
        if(Debug_Level==1) :
            print('TPS already turned on')

    else :
        error = 1
        if(Debug_Level==1) :
            print('Problem turning TPS on')
        
    return [error,response.RC,[]]


def COM_SwitchOffTPS(eOffMode=0) :
    
    request = CreateRequest('112',[eOffMode])
    
    response = SerialRequest(request)
    
    
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Shut down TPS')
    
    else :
        error = 1
        if(Debug_Level==1) :
            print('Error shutting down TPS')

    return [error,response.RC,[]]

def CSV_GetDateTime():
    DateTime = []
    
    response = SerialRequest('%R1Q,5008:')

    error = 1
    if(response.RC==0) :
        error = 0
        
        DateTime = [int(response.parameters[0])]
        for i in range(1,len(response.parameters)) :
            DateTime.append(HexToDec(response.parameters[i]))
        
        if(Debug_Level==1) :
            print('Date and Time: ', DateTime)

    return [error,response.RC,DateTime]

def CSV_GetInstrumentNo():
    response = SerialRequest(CreateRequest('5003',[]))
    error = 1
    serial_num = '0'
    if response.RC == 0:
        error = 0
        serial_num = response.parameters[0]
        
        if (Debug_Level == 1):
            print('Serial number:', serial_num)
    return [error, response.RC, serial_num]

def CSV_GetInstrumentName():
    response = SerialRequest(CreateRequest('5004',[]))
    error = 1
    inst_name = ''
    if response.RC == 0:
        error = 0
        inst_name = response.parameters[0]
        
        if (Debug_Level == 1):
            print('Instrument name:', inst_name)
            
    return [error, response.RC, inst_name]

CSV_POWER_SOURCE = {1: 'CSV_POWER_EXTERNAL',
                    2: 'CSV_POWER_INTERNAL'}

def CSV_CheckPower():
    response = SerialRequest(CreateRequest('5039',[]))
    error = 1
    bat_cap = -1
    power_src = -1

    #Power_src enum: 1 == external, 2 == internal
    
    if (response.RC == 0) or (response.RC == 16) or (response.RC == 18):
        error = 0
        bat_cap = response.parameters[0]
        power_src = response.parameters[1]
        
        if (Debug_Level == 1):
            print('Battery capacity:', bat_cap)
            print('Power source:', power_src)
            
    return [error, response.RC, [bat_cap, power_src]]

def AUS_SetUserLockState(on = 0):
    """ [GeoCOM manual **p42**] """

    request = CreateRequest('18007',[on])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Lock activated or deactivated')
    return [error, response.RC, []]

def AUS_SetUserAtrState(on = 0):
    """ [GeoCOM manual **p40**] """

    request = CreateRequest('18005',[on])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('ATR activated or deactivated')


    return [error, response.RC, []]

def AUT_MakePositioning(Hz, V, POSMode=0, ATRMode=0, bDummy=0):

    request = CreateRequest('9027',[Hz,V,POSMode, ATRMode, bDummy])

    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Going to new position: ', Hz, ',', V)

            
    return [error, response.RC, []]

def AUT_Search(Hz_Area, V_Area, bDummy = 0, t_timeout = 3):
    
    request = CreateRequest('9029',[Hz_Area, V_Area, bDummy])
    
    response = SerialRequest(request, t_timeout = t_timeout)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Target search successful')
    else :
        if(Debug_Level==1) :
            if(response.RC==8710) :
                print('No target found')
            
    return [error, response.RC, []]


# Does not work - connection time out...
# seems to be not needed, when the Leica is directed to the prism
def AUT_FineAdjust(dSrchHz=0.1, dSrchV=0.1):

    request = CreateRequest('9037',[dSrchHz, dSrchV, 0])

    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
            
    return [error, response.RC, []]


def AUT_LockIn() :
    
    request = CreateRequest('9013',[])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Lock successful')

            
    return [error, response.RC, []]


def BAP_SetPrismType2(PrismType = 1):

    request = CreateRequest('17008',[PrismType])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0

    return [error, response.RC, []]

def BAP_SetPrismType(PrismType = 7):

    request = CreateRequest('17008',[PrismType])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0

    return [error, response.RC, []]

def TMC_SetOrientation():

    request = CreateRequest('2113',[0])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0

    return [error, response.RC, []]


def EDM_Laserpointer(eOn = 0):

    request = CreateRequest('1004',[eOn])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Laserpointer turned on/off')
        
    
    return [error, response.RC, []]

EDM_INTENSITY = {'EDM_EGLINTEN_OFF': 0,
                 'EDM_EGLINTEN_LOW': 1,
                 'EDM_EGLINTEN_MID': 2,
                 'EDM_EGLINTEN_HIGH' : 3}

def EDM_SetEGLIntensity(cmd):

    request = CreateRequest('1059',[cmd])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('EGL intensity set to ', cmd)
        
    
    return [error, response.RC, []]

    
def TMC_DoMeasure(cmd=1, mode=1) :
    
    request = CreateRequest('2008',[cmd,mode])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Measuring successful')
        
            
    return [error, response.RC, []]


def TMC_SetEdmMode(mode=6) :

    request = CreateRequest('2020',[mode])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('EDM Mode set successfully')
            
    return [error, response.RC, []]


def TMC_GetCoordinate(WaitTime=100,mode=1) :
    coord = []
    
    request = CreateRequest('2082',[WaitTime,mode])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        
        if(len(response.parameters)==8) :
            coord = [float(response.parameters[0]),float(response.parameters[1]),float(response.parameters[2])]
            
            if(Debug_Level==1) :
                print('Coordinates read successfully: ', coord)
                
        else :
            error = 1
    
            
    return [error, response.RC, coord]

        
        
def TMC_GetSimpleMea(WaitTime=100, mode = 1) :
    coord = []
    request = CreateRequest('2108',[WaitTime,mode])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        coord = [response.parameters[0],response.parameters[1],response.parameters[2]]
        if(Debug_Level==1) :
            print('Coordinates read successfully: ', coord)

            
    return [error, response.RC, coord]


def TMC_GetAngle(mode = 1) :
    coord = []
    request = CreateRequest('2107',[mode])

    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        coord = [response.parameters[0],response.parameters[1]]

    return [error, response.RC, coord]


#def TMC_GetEdmMode(): 
#    
#    EDM_MODE = {0 : 'EDM_MODE_NOT_USED',
#                1 : 'EDM_SINGLE_TAPE', 
#                2 : 'EDM_SINGLE_STANDARD',
#                3 : 'EDM_SINGLE_FAST',
#                4 : 'EDM_SINGLE_LRANGE',
#                5 : 'BAP_CONT_REF_FAST',
#                6 : 'BAP_CONT_RLESS_VISIBLE',
#                7 : 'BAP_AVG_REF_STANDARD',rosrun leica_ros leica_node.py
#                8 : 'BAP_AVG_REF_VISIBLE',
#                9 : 'BAP_AVG_RLESS_VISIBLE',
#                10 :'BAP_CONT_REF_SYNCHRO',
#                11 :'BAP_SINGLE_REF_PRECISE'}
##    EDM_MODE :
##        EDM_MODE_NOT_USED       0, // Init value
##                 1, // IR Standard Reflector Tape
##             2, // IR Standard
##                 3, // IR Fast
##               4, // LO Standard
##        EDM_SINGLE_SRANGE       5, // RL Standard
##        EDM_CONT_STANDARD       6, // Standard repeated measurement
##        EDM_CONT_DYNAMIC        7, // IR Tacking
##        EDM_CONT_REFLESS        8, // RL Tracking
##        EDM_CONT_FAST           9, // Fast repeated measurement
##        EDM_AVERAGE_IR          10,// IR Average
##        EDM_AVERAGE_SR          11,// RL Average
##        EDM_AVERAGE_LR          12,// LO Average
##        EDM_PRECISE_IR          13,// IR Precise (TS30, TM30)
##        EDM_PRECISE_TAPE        14,// IR Precise Reflector Tape (TS30, TM30)
#    
#    request = CreateRequest('2021',[])
#    
#    response = SerialRequest(request)
#    
#    error = 1
#    if(response.RC==0) :
#        error = 0
#        if(Debug_Level==1) :
#            print 'EDM Mode read successfully: '
#            
#    return [error, response.RC, []]
#
#
#def TMC_SetEdmMode(mode) :
#    
##    EDM_MODE :
##        EDM_MODE_NOT_USED       0, // Init value
##        EDM_SINGLE_TAPE         1, // IR Standard Reflector Tape
##        EDM_SINGLE_STANDARD     2, // IR Standard
##        EDM_SINGLE_FAST         3, // IR Fast
##        EDM_SINGLE_LRANGE       4, // LO Standard
##        EDM_SINGLE_SRANGE       5, // RL Standard
##        EDM_CONT_STANDARD       6, // Standard repeated measurement
##        EDM_CONT_DYNAMIC        7, // IR Tacking
##        EDM_CONT_REFLESS        8, // RL Tracking
##        EDM_CONT_FAST           9, // Fast repeated measurement
##        EDM_AVERAGE_IR          10,// IR Average
##        EDM_AVERAGE_SR          11,// RL Average
##        EDM_AVERAGE_LR          12,// LO Average
##        EDM_PRECISE_IR          13,// IR Precise (TS30, TM30)
##        EDM_PRECISE_TAPE        14,// IR Precise Reflector Tape (TS30, TM30)
#
#    request = CreateRequest('2020',[mode])
#    
#    response = SerialRequest(request)
#
#    error = 1
#    if(response.RC==0) :
#        error = 0
#        if(Debug_Level==1) :
#            print 'EDM Mode set successfully'
#            
#    return [error, response.RC, []]
    
MOT_CONTROLLER_MODE = {'MOT_POSIT' : 0,
                       'MOT_OCONST' : 1,
                       'MOT_MANUPOS' : 2,
                       'MOT_LOCK' : 3,
                       'MOT_BRAKE' : 4,
                       'MOT_TERM' : 5}

def MOT_StartController(ControlMode=2):
    request = CreateRequest('6001',[ControlMode])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Motor controller started')
   
    return [error, response.RC, []]

MOT_STOP_MODE = {'MOT_NORMAL' : 0,
                 'MOT_SHUTDOWN'  : 1}

def MOT_StopController(Mode=0):
    
    request = CreateRequest('6000',[Mode])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Motor controller stopped')

    
    return [error, response.RC, []]
    
def MOT_SetVelocity(Hz_speed,v_speed) :
    
    request = CreateRequest('6004',[Hz_speed,v_speed])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Velocity set')
            
            
    return [error, response.RC, []]

MOT_LOCK_STATUS = {0: 'MOT_LOCKED_OUT',
                   1: 'MOT_LOCKED_IN',
                   2: 'MOT_PREDICTION'}

def MOT_ReadLockStatus() :
    
    request = CreateRequest('6021',[])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Lock status:', response.parameters[0])
            
            
    return [error, response.RC, response.parameters[0]]
    
    
BAP_TARGET_TYPE = { 0 : 'BAP_REFL_USE', # with reflector
                    1 : 'BAP_REFL_LESS'} # without reflector

def BAP_GetTargetType() :



    parameter = []
    request = CreateRequest('17022',[])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        parameter = response.parameters[0]
        
        if(Debug_Level==1) :
            print('Target type: ', BAP_TARGET_TYPE[int(response.parameters[0])][1])

            
    return [error, response.RC, parameter]

    
    
def BAP_SetTargetType(eTargetType = 0) :
    
    request = CreateRequest('17021',[eTargetType])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Target set successfully ')
            
            
    return [error, response.RC, []]
    
    
BAP_PRISMTYPE = {0 : ['BAP_PRISM_ROUND', 'Leica Circular Prism'],
                 1 : ['BAP_PRISM_MINI', 'Leica Mini Prism'], 
                 2 : ['BAP_PRISM_TAPE', 'Leica Reflector Tape'],
                 3 : ['BAP_PRISM_360', 'Leica 360 Prism'],
                 4 : ['BAP_PRISM_USER1', 'not supported'],
                 5 : ['BAP_PRISM_USER2', 'not supported'],
                 6 : ['BAP_PRISM_USER3', 'not supported'],
                 7 : ['BAP_PRISM_360_MINI', 'Leica Mini 360 Prism'],
                 8 : ['BAP_PRISM_MINI_ZERO', 'Leica Mini Zero Prism'], 
                 9 : ['BAP_PRISM_USE', 'User Defined Prism'], 
                 10 :['BAP_PRISM_NDS_TAPE','Leica HDS Target'],
                 11 :['BAP_PRISM_GRZ121_ROUND', 'GRZ121 360 Prism for Machine Guidance'], 
                 12 :['BAP_PRISM_MA_MP3122', 'MPR122 360 Prism for Machine Guidance'] }
    

def BAP_GetPrismType() :
    parameter = []
    
    request = CreateRequest('17009',[]) 
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        parameter = [response.parameters[0], BAP_PRISMTYPE[int(response.parameters[0])][1]]
        if(Debug_Level==1) :
            print('Prism type: ', BAP_PRISMTYPE[int(response.parameters[0])][1])

            
    return [error, response.RC, parameter]

# def BAP_GetPrismType2() :
#     parameter = []
    
#     request = CreateRequest('17031',[]) 
    
#     response = SerialRequest(request)
    
#     error = 1
#     if(response.RC==0) :
#         error = 0
#         parameter = [response.parameters[0], BAP_PRISMTYPE[int(response.parameters[0])][1]]
#         if(Debug_Level==1) :
#             print('Prism type: ', BAP_PRISMTYPE[int(response.parameters[0])][1])

            
#     return [error, response.RC, parameter]



def BAP_SetPrismType(ePrismType) :
    
    request = CreateRequest('17008',[ePrismType])
    
    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Prism type set')
            
    return [error, response.RC, []]
    
def BAP_SetPrismType2(ePrismType) :
    
    request = CreateRequest('17008',[ePrismType])
    
    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Prism type set')
            
    return [error, response.RC, []]
     
    
BAP_USER_MEASPRG = {0 : ['BAP_SINGLE_REF_STANDARD','Reflector, Standard'],
                    1 : ['BAP_SINGLE_REF_FAST', 'Reflector, Fast'], 
                    2 : ['BAP_SINGLE_REF_VISIBLE', 'Long Range, Standard'], 
                    3 : ['BAP_SINGLE_RLESS_VISIBLE', 'No Reflector, Standard'], 
                    4 : ['BAP_CONT_REF_STANDARD', 'Reflector, Tracking'], 
                    5 : ['BAP_CONT_REF_FAST', 'not available'], 
                    6 : ['BAP_CONT_RLESS_VISIBLE', 'No Reflector, Fast Tracking'], 
                    7 : ['BAP_AVG_REF_STANDARD', 'Reflector, Average'], 
                    8 : ['BAP_AVG_REF_VISIBLE', 'Long Range, Average'], 
                    9 : ['BAP_AVG_RLESS_VISIBLE', 'No Reflector, Average'], 
                    10 :['BAP_CONT_REF_SYNCHRO', 'Reflector, Synchro Tracking'],
                    11 :['BAP_SINGLE_REF_PRECISE','not available']}
    
def BAP_GetMeasPrg() :
    
    parameter = []
    

    request = CreateRequest('17018',[])
    
    response = SerialRequest(request)

    error = 1
    if(response.RC==0) :
        error = 0
        parameter = [response.parameters[0],BAP_USER_MEASPRG[int(response.parameters[0])][1]]
        if(Debug_Level==1) :
            print('Measurement program: ', BAP_USER_MEASPRG[int(response.parameters[0])][1])

            
    return [error, response.RC, parameter]
    
def BAP_SetMeasPrg(eMeasPrg) :
    
    request = CreateRequest('17019',[eMeasPrg])
    
    response = SerialRequest(request)
    
    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Measurement program set')

            
    return [error, response.RC, []]
    
def BAP_SearchTarget(bDummy = 0) :
    
    request = CreateRequest('17020',[bDummy])
        
    response = SerialRequest(request,0,10)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Prism found!')
            
    else :
        if(Debug_Level==1) :

            if(response.RC == 8710) :
                print('No prism found!')

            elif(response.RC == 8711) :
                print('Multiple prism found!')

            
    return [error, response.RC, []]
    
    
def CAM_TakeImage(CamID = 0):
    
    request = CreateRequest('23623',[CamID])
        
    response = SerialRequest(request,0,3)

    error = 1
    if(response.RC==0) :
        error = 0
        if(Debug_Level==1) :
            print('Prism found!')
            
            
    return [error, response.RC, []]
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    



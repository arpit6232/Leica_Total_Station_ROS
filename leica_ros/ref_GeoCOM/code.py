""" 
.. module :: Geocom
"""

import serial 
import time 

ser = 0
Debug_Level = 0
GTrId = 0

class ResponseClass:
    """
    Manage the response from the station (transaction ID and paramteres returned)
    and the error codes returned. 
    : RC_COM : Communication return Code
    : Trid : transaction ID
    : RC = Request return code
    : parameters : list of returned parameters 

    """
    RC_COM = 0
    TrID = 0
    RC = 0
    paramters = []

    def setResponse(self, response):
        """
        Create and Initiate a ResponseClass from an ASCII response. 
        : response : ASCII response from the station
        : type response : ResponseClass 
        """
        if(Debug_Level==2):
            print'response =',response 
            #removing the end line and splitting 
            words = response.replace('\'','').strip().split(',') #Stripping and Splitting 

            if(len(words)>1):
                self.RC_COM = int(words[1])
                words2 = words[2].split(':')
                self.TrID = int(words2[0])
                self.RC = int(words2[1])
                self.parameters = words[3:len(words)]
                if(self.RC!=0 and Debug_Level==1):
                    print 'Problem, Error code:', self.RC

class SerialRequestError(Exception):
    """
    Create and Initiate a SerialRequestError from an Exception.
    : Exception: Exception error has ocured 
    : type Exception: Exception
    """
    def __init__(self,value):
        """
        Initialize the error 
        :value: the message from the exception
        : type value: str
        """
        self.value = value

    def __str__(self):
        #Return the message from the error as a string
        return repr(self.value)

    def getTrid(request):
        """
        Getting the transaction ID from the ASCII request.
        :request: an ASCII request
        : type request: str
        : returns: parsed transaction ID
        : rtype: int
        """
        words = request.replace('\'').strip().split(',')[2].split(':')
        return int(words[0])

    def SerialRequest(request, length = 0, t_timeout=3):
        """
        Seding the request to the server 
        


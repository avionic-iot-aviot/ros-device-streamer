from configparser import ConfigParser
config = ConfigParser()
config.read('./config.ini')


class Packets:

    def __init__(self,NetId,Length,Destination,Source,Type,TTL,NextHop,Payload):
        self.NetId = NetId
        self.Length = Length
        self.Destination = Destination
        self.Source = Source
        self.Type = Type
        self.TTL = TTL
        self.NextHop = NextHop
        self.Payload = Payload
        self.LengthCalcolator()

    def printFullPacket(self):
        return f"\nNetId:[{self.NetId}] Length:[{self.Length}] Destination:[{self.Destination}] Source:[{self.Source}] Type:[{self.Type}] TTL:[{self.TTL}] NextHop:[{self.NextHop}] Payload:[{self.Payload}]\n"
    
    def printLitePacket(self):
        return f"\n[{self.NetId}]  [{self.Length}]  [{self.Destination}]  [{self.Source}]  [{self.Type}]  [{self.TTL}]  [{self.NextHop}]  [{self.Payload}]\n"

    @classmethod
    def getPacketFromBytes(cls,frame):

        range1 = int(config['PACKET']['LenNetId'])
        range2 = range1 + int(config['PACKET']['LenLength'])
        range3 = range2 + int(config['PACKET']['LenDestination'])
        range4 = range3 + int(config['PACKET']['LenSource'])
        range5 = range4 + int(config['PACKET']['LenType'])
        range6 = range5 + int(config['PACKET']['LenTTL'])
        range7 = range6 + int(config['PACKET']['LenNextHop'])

        frame1 = frame[:range1]
        frame2 = frame[range1:range2]
        frame3 = frame[range2:range3]
        frame4 = frame[range3:range4]
        frame5 = frame[range4:range5]
        frame6 = frame[range5:range6]
        frame7 = frame[range6:range7]
        frame8 = frame[range7:]

        while frame1.decode("utf-8")[0] == "-":
            frame1 = frame1[1:]

        while frame2.decode("utf-8")[0] == "-":
            frame2 = frame2[1:]
        
        while frame3.decode("utf-8")[0] == "-":
            frame3 = frame3[1:]

        while frame4.decode("utf-8")[0] == "-":
            frame4 = frame4[1:]

        while frame5.decode("utf-8")[0] == "-":
            frame5 = frame5[1:]

        while frame6.decode("utf-8")[0] == "-":
            frame6 = frame6[1:]

        while frame7.decode("utf-8")[0] == "-":
            frame7 = frame7[1:]

        if isinstance (frame8, str):
            return cls(frame1.decode("utf-8"), frame2.decode("utf-8"), frame3.decode("utf-8"), frame4.decode("utf-8"), frame5.decode("utf-8"), frame6.decode("utf-8"), frame7.decode("utf-8"), frame8.decode("utf-8") )
        else:
            return cls(frame1.decode("utf-8"), frame2.decode("utf-8"), frame3.decode("utf-8"), frame4.decode("utf-8"), frame5.decode("utf-8"), frame6.decode("utf-8"), frame7.decode("utf-8"), frame8)
       
        
    def getBytesFromPackets(self):
        self.fixTheLen()
        
        if ( (len(self.NetId) == int(config['PACKET']['LenNetId']))  and (len(self.Length) == int(config['PACKET']['LenLength'])) and (len(self.Destination) == int(config['PACKET']['LenDestination'])) and (len(self.Source) == int(config['PACKET']['LenSource'])) and (len(self.Type) == int(config['PACKET']['LenType'])) and (len(self.TTL) == int(config['PACKET']['LenTTL'])) and (len(self.NextHop) == int(config['PACKET']['LenNextHop'])) and (len(self.Payload) <= int(config['PACKET']['LenPayload'])) ) :
            frame1 = bytearray(self.NetId,'utf-8')
            frame2 = bytearray(self.Length,'utf-8')
            frame3 = bytearray(self.Destination,'utf-8')
            frame4 = bytearray(self.Source,'utf-8')
            frame5 = bytearray(self.Type,'utf-8')
            frame6 = bytearray(self.TTL,'utf-8')
            frame7 = bytearray(self.NextHop,'utf-8')
            
            if isinstance (self.Payload, str):
                frame8 = bytearray(self.Payload,'utf-8')
            else:
                frame8=bytearray()
                frame8.extend(self.Payload)

            


          
            frame = frame1 + frame2 + frame3 + frame4 + frame5 + frame6 + frame7 + frame8
            return frame
        else:
            print("Error Packet Size")
                
    def fixTheLen(self):
        fix1 = int(config['PACKET']['LenNetId']) - len(self.NetId)
        if (fix1 != 0):
            while fix1 > 0:
                self.NetId = "-" + self.NetId
                fix1=fix1 - 1
                fix1 = int(config['PACKET']['LenNetId']) - len(self.NetId)

        fix1 = int(config['PACKET']['LenLength']) - len(self.Length)
        if (fix1 != 0):
            while fix1 > 0:
                self.Length = "-" + self.Length
                fix1=fix1 - 1

        fix1 = int(config['PACKET']['LenDestination']) - len(self.Destination)
        if (fix1 != 0):
            while fix1 > 0:
                self.Destination = "-" + self.Destination
                fix1=fix1 - 1

        fix1 = int(config['PACKET']['LenSource']) - len(self.Source)
        if (fix1 != 0):
            while fix1 > 0:
                self.Source = "-" + self.Source
                fix1=fix1 - 1

        fix1 = int(config['PACKET']['LenType']) - len(self.Type)
        if (fix1 != 0):
            while fix1 > 0:
                self.Type = "-" + self.Type
                fix1=fix1 - 1

        fix1 = int(config['PACKET']['LenTTL']) - len(self.TTL)
        if (fix1 != 0):
            while fix1 > 0:
                self.TTL = "-" + self.TTL
                fix1=fix1 - 1

        fix1 = int(config['PACKET']['LenNextHop']) - len(self.NextHop)
        if (fix1 != 0):
            while fix1 > 0:
                self.NextHop = "-" + self.NextHop
                fix1=fix1 - 1

        # fix1 = int(config['PACKET']['LenPayload']) - len(self.Payload)
        # if (fix1 != 0):
        #     while fix1 > 0:
        #         self.Payload = "0" + self.Payload
        #         fix1=fix1 - 1

    def LengthCalcolator(self):
        temp = len(self.NetId) + len(self.Destination) + len(self.Source) +len(self.Type) + len(self.TTL) +len(self.NextHop) +len(self.Payload)
        #print(temp)
        self.Length = str(temp)
    
    def DecreaseTTL(self):
        temp = int(self.TTL) - 1
        self.TTL = str(temp)
    
    def ChangeDst(self,NewDest):
        self.Destination = NewDest

class BeaconPacket(Packets):
    
    def __init__(self,NetId,Destination,Source,TTL,NextHop,Payload):
        #super().__init__(NetId,"",Destination,Source,"0",TTL,NextHop,"Payload BEACON")
        super().__init__(NetId,"",Destination,Source,"0",TTL,NextHop,Payload)  #0 beacon
        super().LengthCalcolator()

class ReportPacket(Packets):
    
    def __init__(self,NetId,Destination,Source,TTL,NextHop,Payload):
        super().__init__(NetId,"",Destination,Source,"1",TTL,NextHop,Payload)  #1 report
        super().LengthCalcolator()

class DataPacket(Packets):
    
    def __init__(self,NetId,Destination,Source,TTL,NextHop,Payload):
        #super().__init__(NetId,"",Destination,Source,"2",TTL,NextHop,"Payload DATA")  #2 data
        super().__init__(NetId,"",Destination,Source,"2",TTL,NextHop,Payload)  #2 data
        super().LengthCalcolator()


class FunctionPacket(Packets):    
    def __init__(self,NetId,Destination,Source,TTL,NextHop,Payload):
        #super().__init__(NetId,"",Destination,Source,"2",TTL,NextHop,"Payload DATA")  #2 data
        super().__init__(NetId,"",Destination,Source,"3",TTL,NextHop,Payload)  #2 data
        super().LengthCalcolator()


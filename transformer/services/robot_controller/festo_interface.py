import time
import socket

class FestoManifold:
    main_pressure_input = 0
    schunk_pressure_input = 1
    schunkclamp = 4 #schunk clamp / unclamp coil / valve
    grclamp = 2 #gressell riser clamp / clamp coil / valve
    RENISHAW = 7
    clamp = False
    unclamp = True
    RENON = False
    RENOFF = True
    def __init__(self,FestoIPAddress):
        # Festo ethernet IP uses port 991
        # default subnet is 192.168.1.n, last quad must be set using dipswitches
        # Subnet must be set using festo maintenance software tool
        self.HostAddressPort = (FestoIPAddress, 991)
        self.UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    def __del__(self): 
        # close the UDP socket connection on exit
        self.UDPSocket.close()
    
    # Use Ethernet/IP protocol to get the pressue value from Festo pressure sensors
    # Pressure value is returned as an integer in mbar
    def get_analog_input_values(self, analogchan):
        try:
            # Command to retrieve pressure readings: DE1.n where n = the pressure channel (0..3) 
            analogsendmessage = "DE1." + str(analogchan)

            #Encode to byte array and send message to Festo 
            self.UDPSocket.sendto(analogsendmessage.encode("utf-8"), self.HostAddressPort)

            # This can hang for a very long time if there's a connection issue, so
            # Turn off blocking and set timeout to 1 second
            self.UDPSocket.setblocking(0)
            self.UDPSocket.settimeout(1)

            # Message is returned as a tuple containing the response and who sent it (=value, (ipaddress,port)) 
            msgFromServer, hostaddressport = self.UDPSocket.recvfrom(1500)

            # Split message to get string value
            null, value = msgFromServer.decode("utf-8").split("=")

            # convert to an int (mbar)
            value = int(value)
        except Exception as error:
            print("Error - get_analog_input_valves:" + str(error))
            value = 0
        return value

    # Use Ethernet/IP protocol to get the coil / valve state
    # coil / valve state is returned as a boolean
    def get_valves_state(self, Valve):
        try:
            # Command to retrieve coil / valve state: FA2.n
            # where n = the coil / valve number (0..31)
            valvestatemessage = "FA2." + str(Valve)

            # Encode to byte array and send message
            self.UDPSocket.sendto(valvestatemessage.encode("utf-8"), self.HostAddressPort)

            # This can hang for a very long time if there's a connection issue, so
            # Turn off blocking and set timeout to 1 second
            self.UDPSocket.setblocking(0)
            self.UDPSocket.settimeout(1)

            # Message is returned as a tuple containing the response and who sent it (=value, (ipaddress,port)) 
            msgFromServer, hostaddressport = self.UDPSocket.recvfrom(1500)

            # Split message to get string value
            null, value = msgFromServer.decode("utf-8").split("=")

            # Valve state can be disabled or 0 = False, or 1 = True
            if ((value == 'd') or (value == '0')):
                valvestate = False
            else:
                valvestate = True
      
        except Exception as error:
            print("Error - get_valves_state:" + str(error))
            valvestate = False
        return valvestate
    
    # Use Ethernet/IP protocol to set the coil / valve state
    # Returns True if coil / valve = new_state, False otherwise
    def change_valve_state(self, valve_id, new_state):
        valvestate = False
        try:
            # Command to set coil/valve state: FA2.n = x 
            # were n = valve number, x = state (d=disabled, 0=off, 1=on)
            if ((str(new_state) == "1") or (str(new_state) == "True") or (str(new_state) == "true")):
                New_Valve_State = 1
            else:
                New_Valve_State = 0

            # Build coil / valve state change string
            # First enable valves: FE=1;
            # Then add valve state change FA2.n=x
            valvesetmessage = "FE=1;FA2." + str(valve_id) + "=" + str(New_Valve_State)

            # Encode to byte array and send message
            self.UDPSocket.sendto(valvesetmessage.encode("utf-8"), self.HostAddressPort)

            # This can hang for a very long time if there's a connection issue, so
            # Turn off blocking and set timeout to 1 second
            self.UDPSocket.setblocking(0)
            self.UDPSocket.settimeout(1)

            # Message is returned as a tuple containing the response and who sent it (=value, (ipaddress,port)) 
            msgFromServer, hostaddressport = self.UDPSocket.recvfrom(1500)

            # Should receive "OK" from festo
            if (msgFromServer.decode("utf-8") == "OK"):
                # get the current state of the valve
                newstate = self.get_valves_state(valve_id) 
                # Return True if the valve state matches what we wanted
                if ((str(new_state) == "1") or (str(new_state) == "True") or (str(new_state) == "true")):
                    # we wanted a 1
                    if (newstate == True):
                        valvestate = True
                    else:
                        valvestate = False
                else:
                    # we wanted a 0
                    if (newstate == False):
                        valvestate = True
                    else:
                        valvestate = False
                # did not return "OK"
            else:
                valvestate = False

        except Exception as error:
            print("Error - set_valves_state:" + str(error))
            valvestate = False

        return bool(valvestate)



# create festo manifold class and connect to DVF4 festo @ 192.168.1.114
# festotest = FestoManifold("192.168.1.114")

# # Capture main air pressure and convert from mbar to psi
# MainPressureInPSI = (festotest.get_analog_input_values(0)/1000) * 14.5038

# # Capture schunk clamp station pressure and convert from mbar to psi
# # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
# SchunkClampPressurePSI = (festotest.get_analog_input_values(1)/1000) * 14.5038

# # Capture Gressell Riser clamp station pressure and convert from mbar to psi
# # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
# GRClampPressurePSI = (festotest.get_analog_input_values(1)/1000) * 14.5038

# # Get all the pressures and display them before fliping valves on/off
# # Capture main air pressure and convert from mbar to psi
# MainPressureInPSI = (festotest.get_analog_input_values(0)/1000) * 14.5038

# # Capture schunk clamp station pressure and convert from mbar to psi
# # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
# SchunkClampPressurePSI = (festotest.get_analog_input_values(1)/1000) * 14.5038

# # Capture Gressell Riser clamp station pressure and convert from mbar to psi
# # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
# GRClampPressurePSI = (festotest.get_analog_input_values(2)/1000) * 14.5038

# print("Main Pressure:", str(round(MainPressureInPSI, 2)), " PSI")
# print("Schunk Clamp pressure:", str(round(SchunkClampPressurePSI, 2)), "PSI")
# print("GR Clamp pressure:", str(round(GRClampPressurePSI, 2)))
# print("")

# while(1):
#     print("Unclamping Schunk")
#     # unclamp the schunk station
#     festotest.change_valve_state(festotest.schunkclamp, festotest.unclamp)

#     # wate 1 second to check pressure
#     time.sleep(1)

#     # Capture main air pressure and convert from mbar to psi
#     MainPressureInPSI = (festotest.get_analog_input_values(0)/1000) * 14.5038

#     # Capture schunk clamp station pressure and convert from mbar to psi
#     # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
#     SchunkClampPressurePSI = (festotest.get_analog_input_values(1)/1000) * 14.5038

#     # Capture Gressell Riser clamp station pressure and convert from mbar to psi
#     # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
#     GRClampPressurePSI = (festotest.get_analog_input_values(2)/1000) * 14.5038

#     print("Main Pressure:", str(round(MainPressureInPSI, 2)), " PSI")
#     print("Schunk Clamp pressure:", str(round(SchunkClampPressurePSI, 2)), "PSI")
#     print("GR Clamp pressure:", str(round(GRClampPressurePSI, 2)))
#     print("")

#     # wate 10 seconds before clamping
#     time.sleep(5)

#     # unclamp the schunk station
#     festotest.change_valve_state(festotest.schunkclamp, festotest.clamp)

#     # wate 1 second to check pressure
#     time.sleep(1)

#     # Capture main air pressure and convert from mbar to psi
#     MainPressureInPSI = (festotest.get_analog_input_values(0)/1000) * 14.5038

#     # Capture schunk clamp station pressure and convert from mbar to psi
#     # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
#     SchunkClampPressurePSI = (festotest.get_analog_input_values(1)/1000) * 14.5038

#     # Capture Gressell Riser clamp station pressure and convert from mbar to psi
#     # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
#     GRClampPressurePSI = (festotest.get_analog_input_values(2)/1000) * 14.5038

#     print("Main Pressure:", str(round(MainPressureInPSI, 2)), " PSI")
#     print("Schunk Clamp pressure:", str(round(SchunkClampPressurePSI, 2)), "PSI")
#     print("GR Clamp pressure:", str(round(GRClampPressurePSI, 2)))
#     print("")
#     time.sleep(5)

   
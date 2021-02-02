#lslidar C32 scan packets

Header           header         #standard ros message header
uint64           basetime       #time to the top of hour
LslidarC32Packet[] packets        #vector of raw packets

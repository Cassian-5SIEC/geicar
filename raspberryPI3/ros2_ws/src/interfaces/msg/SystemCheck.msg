#Message status
bool request false  #If true, the message is a request to check communications
bool response false #If true, the message is a response to the previous request
bool report false   #If true, the message is a report published by the system_check_node
bool print false    #If true, the message is a request to print the last report in the terminal

#responses
bool jetson false
bool l476 false
bool f103 false

#report
string comm_jetson  # "OK" or "Failed"
string comm_l476    # "OK" or "Failed"
string comm_f103    # "OK" or "Failed"

string battery      # "OK", "Low Voltage", "High Voltage", "No data"

string[6] ultrasonics  # [Front left, Front center, Front right, Rear left, Rear center, Rear right], possible values : "OK", "Out of range", "No data"

string gps # "No Fix", "Autonomous GNSS fix", "Differential GNSS fix", "RTK fixed", "RTK float", "Estimated/dead reckoning fix", "No data"

string imu # "OK", "No data"

string lidar # "OK", "No data"

string camera # "OK", "No data"
from DobotController import DobotController

ip = "192.168.5.1"

with DobotController(ip) as dc:
    print(dc.clear_error())
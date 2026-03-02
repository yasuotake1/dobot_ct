import sys
from time import sleep

from dobot.dobot_api import *

ip = "192.168.5.1"
dashboardPort = 29999

pt_pkg = [-90.0000,0.0000,-140.0000,-40.0000,0.0000,0.0000,1]
pt_org = [0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,1]
pt_1 = [0.0000,0.0000,0.0000,0.0000,45.0000,0.0000,1]
pt_pallet1 = [162.2955,-98.4775,283.3075,179.6192,-0.5929,-134.9657,0]
relZ_up = 60.0
relX_col = 0.5
relY_col = 24.5
relX_raw = 25.0
relY_raw = -0.5
pt_gonio = [-207.6416,-263.9685,268.0075,179.6192,-0.5929,178.6143,0]
pt_gonioUp = pt_gonio[:2] + [pt_gonio[2] + relZ_up] + pt_gonio[3:]
pt_midway = [123.0251,-162.1742,335.6075,179.6192,-0.5929,-134.9657,0]

db = None

def main():
    global db
    try:
        db = DobotApiDashboard(ip, dashboardPort)
    except Exception as e:
        print(f"Connection Error:{e}")
        sys.exit()
    
    db.VelJ(20)
    db.VelL(80)
    movj_and_wait(pt_midway)
    sleep(1)
    db.DO(2,1)
    sleep(1)
    db.DO(2,0)
    sleep(1)
    db.DO(1,1)
    sleep(1)
    db.DO(1,0)
    sleep(1)
    db.DO(1,1)
    sleep(1)

    for i in range(10):
        pallet_to_gonio(i)
        sleep(1)
        gonio_to_pallette(i)
    
    movl_and_wait(pt_midway)

def pallet_to_gonio(idx: int):
    movl_and_wait(posPallete(idx, True))
    movl_and_wait(posPallete(idx, False))
    db.DO(1,0)
    sleep(1)
    movl_and_wait(posPallete(idx, True))
    movl_and_wait(pt_midway)
    movl_and_wait(pt_gonioUp)
    movl_and_wait(pt_gonio)
    sleep(1)
    db.DO(1,1)
    sleep(1)
    movl_and_wait(pt_gonioUp)
    movl_and_wait(pt_midway)

def gonio_to_pallette(idx: int):
    movl_and_wait(pt_gonioUp)
    movl_and_wait(pt_gonio)
    db.DO(1,0)
    sleep(1)
    movl_and_wait(pt_gonioUp)
    movl_and_wait(pt_midway)
    movl_and_wait(posPallete(idx, True))
    movl_and_wait(posPallete(idx, False))
    sleep(1)
    db.DO(1,1)
    sleep(1)
    movl_and_wait(posPallete(idx, True))

def posPallete(idx: int, up: bool):
    relX = relX_col * (idx % 5) + relX_raw * (idx // 5)
    relY = relY_col * (idx % 5) + relY_raw * (idx // 5)
    relZ = relZ_up if up else 0.0
    return [pt_pallet1[0] + relX] + [pt_pallet1[1] + relY] + [pt_pallet1[2] + relZ] + pt_pallet1[3:]

def movj_and_wait(dst: list[float]):
    recv = db.MovJ(*dst)
    cmdId = int(getValue(recv))
    sleep(0.1)
    while True:
        if int(getValue(db.RobotMode())) == 5 and int(getValue(db.GetCurrentCommandID())) == cmdId :
            break
        sleep(0.1)

def movl_and_wait(dst: list[float]):
    recv = db.MovL(*dst)
    cmdId = int(getValue(recv))
    sleep(0.1)
    while True:
        if int(getValue(db.RobotMode())) == 5 and int(getValue(db.GetCurrentCommandID())) == cmdId :
            break
        sleep(0.1)

def getValue(str):
    return str.split("{")[1].split("}")[0] 

if __name__ == "__main__":
    main()
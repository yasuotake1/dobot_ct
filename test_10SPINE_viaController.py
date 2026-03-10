from __future__ import annotations

from time import sleep

from DobotController import DobotController

ip = "192.168.5.1"

pt_pkg = [-90.0000, 0.0000, -140.0000, -40.0000, 0.0000, 0.0000, 1]
pt_org = [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1]
pt_1 = [0.0000, 0.0000, 0.0000, 0.0000, 45.0000, 0.0000, 1]
pt_pallet1 = [167.5849, 56.7808, 266.5972, 179.6192, -0.5929, -134.9657, 0]
relZ_up = 60.0
relX_col = 0.5
relY_col = 25.0
relX_raw = 25.0
relY_raw = 0.5
pt_stage = [-207.3151, -266.7192, 267.8760, 179.6192, -0.5929, 178.6143, 0]
pt_stageUp = pt_stage[:2] + [pt_stage[2] + relZ_up] + pt_stage[3:]
pt_midway = [123.0251, -162.1742, 335.6075, 179.6192, -0.5929, -134.9657, 0]


def posPallete(idx: int, up: bool):
    relX = relX_col * (idx % 5) + relX_raw * (idx // 5)
    relY = relY_col * (idx % 5) + relY_raw * (idx // 5)
    relZ = relZ_up if up else 0.0
    return [pt_pallet1[0] + relX] + [pt_pallet1[1] + relY] + [pt_pallet1[2] + relZ] + pt_pallet1[3:]


def pallet_to_stage(dc: DobotController, idx: int):
    pose_tcp_up = posPallete(idx, True)
    pose_tcp_dn = posPallete(idx, False)

    dc.mov_l_wait(pose_tcp_up)
    dc.mov_l_wait(pose_tcp_dn)
    dc.do(1, 0)
    sleep(1)
    dc.mov_l_wait(pose_tcp_up)
    dc.mov_l_wait(pt_midway)
    dc.mov_l_wait(pt_stageUp)
    dc.mov_l_wait(pt_stage)
    sleep(1)
    dc.do(1, 1)
    sleep(1)
    dc.mov_l_wait(pt_stageUp)
    dc.mov_l_wait(pt_midway)


def stage_to_pallette(dc: DobotController, idx: int):
    dc.mov_l_wait(pt_stageUp)
    dc.mov_l_wait(pt_stage)
    dc.do(1, 0)
    sleep(1)
    dc.mov_l_wait(pt_stageUp)
    dc.mov_l_wait(pt_midway)

    pose_tcp_up = posPallete(idx, True)
    pose_tcp_dn = posPallete(idx, False)

    dc.mov_l_wait(pose_tcp_up)
    dc.mov_l_wait(pose_tcp_dn)
    sleep(1)
    dc.do(1, 1)
    sleep(1)
    dc.mov_l_wait(pose_tcp_up)


def main():
    with DobotController(ip) as dc:
        dc.enable_robot()
        sleep(5)

        dc.vel_j(20)
        dc.vel_l(60)

        dc.mov_j_wait(pt_midway)
        sleep(1)
        dc.do(2, 1)
        sleep(1)
        dc.do(2, 0)
        sleep(1)
        dc.do(1, 1)
        sleep(1)
        dc.do(1, 0)
        sleep(1)
        dc.do(1, 1)
        sleep(1)

        for i in range(10):
            print(f"Moving Sample #{i + 1} from palette to rotation stage.")
            pallet_to_stage(dc, i)
            sleep(1)
            print(f"Moving Sample #{i + 1} from rotation stage to pallete.")
            stage_to_pallette(dc, i)

        dc.mov_l_wait(pt_midway)

        dc.disable_robot()


if __name__ == "__main__":
    main()

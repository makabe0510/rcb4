from rcb4.armh7interface import ARMH7Interface
import numpy as np
import time
from typing import Tuple

def read_voltage_test(interface):
    val = interface.read_imu_data()
    print(val)
    return val

def free_gripper(interface):
    interface.angle_vector([0, 0], servo_ids=[5, 7])

def open_gripper_init():
    print("open gripper init")
    interface.hold()
    interface.angle_vector([0], servo_ids=[9])
    interface.angle_vector([-90, -90], servo_ids=[2, 3])
    # interface.angle_vector([-30, -30], servo_ids=[0, 1])
    init_gripper(interface, servo_a_id = 4, servo_b_id = 6, threshold = 3.0, shrink_cmd = 40)
    loose_gripper(interface, servo_a_id = 4, servo_b_id = 6, extend_cmd = 40)
    
    init_gripper(interface, servo_a_id = 5, servo_b_id = 7, threshold = 3.0, shrink_cmd = 40)
    loose_gripper(interface, servo_a_id = 5, servo_b_id = 7, extend_cmd = 40)

def hold_vial():
    print("hold vial")
    interface.angle_vector([10, 10], servo_ids=[0, 1])
    time.sleep(5)
    interface.angle_vector([0, 0], servo_ids=[0, 1])

def loosen_stopper():
    print("loosen stopper")
    interface.angle_vector([-90], servo_ids=[3])
    time.sleep(3)
    interface.angle_vector([-15, -15], servo_ids=[5, 7])
    time.sleep(1)
    interface.angle_vector([0, 3], servo_ids=[5, 7])
    time.sleep(0.5)
    interface.angle_vector([-10, -10], servo_ids=[5, 7])
    time.sleep(8)
    interface.angle_vector([0, -2], servo_ids=[5, 7])
    time.sleep(3)
    # interface.angle_vector([13, 10], servo_ids=[5, 7])
    # time.sleep(8)
    # # todo 
    # interface.angle_vector([0, -3], servo_ids=[5, 7])
    # time.sleep(1)
    # # # interface.angle_vector([0, 0], servo_ids=[5, 7])
    # interface.angle_vector([0], servo_ids=[3])
    # interface.angle_vector([0, 0], servo_ids=[5, 7])
    # time.sleep(3)

def insert_stopper():
    print("insert stopper")
    interface.angle_vector([-90], servo_ids=[3])
    time.sleep(3)
    interface.angle_vector([0, -4], servo_ids=[5, 7])
    time.sleep(1)
    interface.angle_vector([-10, -10], servo_ids=[5, 7])
    time.sleep(11)
    interface.angle_vector([0, 2], servo_ids=[5, 7])
    time.sleep(1)
    interface.angle_vector([10, 10], servo_ids=[5, 7])
    time.sleep(11)
    interface.angle_vector([0, 0], servo_ids=[5, 7])
    interface.angle_vector([120], servo_ids=[3])

def loop_task():
    loosen_stopper()
    insert_stopper()


def command_diff_drive_2dof(
    interface,
    close_cmd: float,
    extend_cmd: float,
    servo_a_id: int = 5,
    servo_b_id: int = 7,
) -> Tuple[float, float]:
    """
    差動2自由度（開閉・直動）の指令をモータA/Bへ送信する。

    定義（ユーザ仕様）:
      - 開閉自由度: (A_cmd - B_cmd) に対応
          * 正 -> 閉じる
          * 負 -> 開く
      - 直動自由度: (A_cmd + B_cmd) に対応
          * 負 -> 伸びる
          * 正 -> 縮む

    引数:
      close_cmd  : 閉じる方向の指令値（正で閉、負で開）
      extend_cmd : 伸びる方向の指令値（正で伸び、負で縮み）
      servo_a_id : モータAのID（デフォルト 5）
      servo_b_id : モータBのID（デフォルト 7）

    戻り値:
      (A_cmd, B_cmd) ※実際に送った値
    """
    d = float(close_cmd)        # diff = A - B
    s = -float(extend_cmd)      # sum  = A + B（伸びる=負にしたいので符号反転）

    a_cmd = (s + d) / 2.0
    b_cmd = (s - d) / 2.0

    interface.angle_vector([a_cmd, b_cmd], servo_ids=[servo_a_id, servo_b_id])
    return a_cmd, b_cmd

def _stop_two_servos(interface, servo_a_id: int, servo_b_id: int) -> None:
    interface.angle_vector([0.0, 0.0], servo_ids=[servo_a_id, servo_b_id])


def _read_sum_current(interface, servo_a_id: int, servo_b_id: int, print_flag = False) -> float:
    currents = interface.read_servo_current()

    ia = float(currents[servo_a_id])
    ib = float(currents[servo_b_id])
    isum = ia + ib

    if print_flag:
        print(
            f"[current] "
            f"A(ID{servo_a_id})={ia:+.3f}, "
            f"B(ID{servo_b_id})={ib:+.3f}, "
            f"sum={isum:+.3f}"
        )
    return isum

def open_then_shrink_until_current_threshold_step(
    interface,
    open_close_cmd: float = 2.0,
    shrink_cmd: float = 40.0,
    threshold: float = 0.5,
    wait_s: float = 1.0,
    servo_a_id: int = 5,
    servo_b_id: int = 7,
    reset: bool = False,
) -> bool:
    """
    ノンブロッキング手順（1回呼ぶごとに1ステップ）:
      1) 開閉を「開く」方向へ指令
      2) wait_s 秒待つ
      3) 直動を「縮む」方向へ指令
      4) abs(Ia + Ib) > threshold で停止して終了

    返り値:
      True  -> 完了（停止含む）
      False -> 継続（もう一度呼ぶ）
    """
    key = (servo_a_id, servo_b_id)

    if reset or not hasattr(open_then_shrink_until_current_threshold_step, "_states"):
        open_then_shrink_until_current_threshold_step._states = {}  # type: ignore

    states = open_then_shrink_until_current_threshold_step._states  # type: ignore
    if reset or key not in states:
        states[key] = {"state": "INIT", "t_open": None}

    st = states[key]["state"]

    if st == "INIT":
        # 開く方向: close_cmd を負（強さは open_close_cmd の絶対値）
        command_diff_drive_2dof(
            interface,
            close_cmd=-abs(float(open_close_cmd)),
            extend_cmd=0.0,
            servo_a_id=servo_a_id,
            servo_b_id=servo_b_id,
        )
        states[key]["t_open"] = time.time()
        states[key]["state"] = "WAIT_OPEN"
        return False

    if st == "WAIT_OPEN":
        if (time.time() - float(states[key]["t_open"])) >= float(wait_s):
            # 縮む方向: extend_cmd を負（extend正=伸び、負=縮み）
            command_diff_drive_2dof(
                interface,
                close_cmd=0.0,
                extend_cmd=-abs(float(shrink_cmd)),
                servo_a_id=servo_a_id,
                servo_b_id=servo_b_id,
            )
            states[key]["state"] = "MONITOR"
        return False

    if st == "MONITOR":
        isum = _read_sum_current(interface, servo_a_id, servo_b_id)
        if abs(isum) > float(threshold):
            _stop_two_servos(interface, servo_a_id, servo_b_id)
            states[key]["state"] = "DONE"
            return True
        return False

    return True  # DONE

def extend_until_current_threshold_step(
    interface,
    extend_cmd: float = 40.0,
    threshold: float = 0.5,      # 低電流判定（abs(Ia+Ib) < threshold）
    servo_a_id: int = 5,
    servo_b_id: int = 7,
    reset: bool = False,
    preclose_cmd: float = 2.0,
    preclose_wait_s: float = 2.0,
    low_hold_s: float = 0.5,     # 低電流が続く必要時間
    max_time: float | None = 10.0 # 暴走防止（Noneで無効）
) -> bool:
    key = (servo_a_id, servo_b_id)

    if reset or not hasattr(extend_until_current_threshold_step, "_states"):
        extend_until_current_threshold_step._states = {}  # type: ignore

    states = extend_until_current_threshold_step._states  # type: ignore
    if reset or key not in states:
        states[key] = {
            "state": "INIT",
            "t_preclose": None,
            "t_start": time.time(),
            "t_low_start": None,
        }

    st = states[key]["state"]

    # 暴走防止
    if max_time is not None and (time.time() - states[key]["t_start"]) > float(max_time):
        _stop_two_servos(interface, servo_a_id, servo_b_id)
        states[key]["state"] = "DONE"
        return True

    if st == "INIT":
        command_diff_drive_2dof(
            interface,
            close_cmd=abs(float(preclose_cmd)),
            extend_cmd=0.0,
            servo_a_id=servo_a_id,
            servo_b_id=servo_b_id,
        )
        states[key]["t_preclose"] = time.time()
        states[key]["state"] = "WAIT_PRECLOSE"
        return False

    if st == "WAIT_PRECLOSE":
        if (time.time() - float(states[key]["t_preclose"])) >= float(preclose_wait_s):
            command_diff_drive_2dof(
                interface,
                close_cmd=0.0,
                extend_cmd=abs(float(extend_cmd)),
                servo_a_id=servo_a_id,
                servo_b_id=servo_b_id,
            )
            states[key]["state"] = "MONITOR_LOW_STABLE"
        return False

    if st == "MONITOR_LOW_STABLE":
        isum = _read_sum_current(interface, servo_a_id, servo_b_id)
        a = abs(isum)

        if a < float(threshold):
            if states[key]["t_low_start"] is None:
                states[key]["t_low_start"] = time.time()

            if (time.time() - float(states[key]["t_low_start"])) >= float(low_hold_s):
                _stop_two_servos(interface, servo_a_id, servo_b_id)
                states[key]["state"] = "DONE"
                return True
        else:
            states[key]["t_low_start"] = None

        return False

    return True

print("open_then_shrink_until_current_threshold_step(interface, open_close_cmd=2.0, shrink_cmd=20.0, reset=True)")

def init_gripper(
    interface,
    servo_a_id: int = 5,
    servo_b_id: int = 7,
    threshold: float = 3.0,
    open_close_cmd: float = 2.0,
    shrink_cmd: float = 40.0,
):
    """
    グリッパ初期化:
      - 開く → 縮む → 電流しきい値で停止
    """
    # 初期化ステップ
    open_then_shrink_until_current_threshold_step(
        interface,
        open_close_cmd=open_close_cmd,
        shrink_cmd=shrink_cmd,
        servo_a_id=servo_a_id,
        servo_b_id=servo_b_id,
        threshold=threshold,
        reset=True,
    )

    done = False
    while not done:
        done = open_then_shrink_until_current_threshold_step(
            interface,
            open_close_cmd=open_close_cmd,
            shrink_cmd=shrink_cmd,
            servo_a_id=servo_a_id,
            servo_b_id=servo_b_id,
            threshold=threshold,
        )
print("left")
print("init_gripper(interface, servo_a_id = 4, servo_b_id = 6, threshold = 2.0, shrink_cmd = 40)")

print("right")
print("init_gripper(interface, servo_a_id = 5, servo_b_id = 7, threshold = 2.0, shrink_cmd = 40)")

def loose_gripper(
    interface,
    servo_a_id: int = 5,
    servo_b_id: int = 7,
    threshold: float = 3.0,
    extend_cmd: float = 40.0,
):
    """
    グリッパを緩める（直動を伸ばす → 電流しきい値で停止）
    """
    # 初期化ステップ
    extend_until_current_threshold_step(
        interface,
        extend_cmd=extend_cmd,
        servo_a_id=servo_a_id,
        servo_b_id=servo_b_id,
        threshold=threshold,
        reset=True,
    )

    done = False
    while not done:
        done = extend_until_current_threshold_step(
            interface,
            extend_cmd=extend_cmd,
            servo_a_id=servo_a_id,
            servo_b_id=servo_b_id,
            threshold=threshold,
        )
print("left")
print("loose_gripper(interface, servo_a_id = 4, servo_b_id = 6, extend_cmd = 40)")

print("right")
print("loose_gripper(interface, servo_a_id = 5, servo_b_id = 7, extend_cmd = 40)")

if __name__ == "__main__":
    interface = ARMH7Interface()
    try:
        print(interface.auto_open())
        interface.switch_reading_servo_current(True)
        open_gripper_init()
        # loosen_stopper()
        # insert_stopper()
    except Exception as e:
        if "LIBUSB_ERROR_ACCESS" in str(e):
            # Error already handled and printed by the class method
            pass
        else:
            print(f"Error: {e}")


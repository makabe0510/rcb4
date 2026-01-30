from rcb4.armh7interface import ARMH7Interface
import numpy as np
import time
from typing import Tuple
from dataclasses import dataclass
from typing import List, Dict, Sequence, Literal

GripperStage = Literal["init", "loose"]

@dataclass(frozen=True)
class GripperConfig:
    name: str
    servo_a_id: int
    servo_b_id: int

    # --- init 用（open_then_shrink...） ---
    init_threshold: float = 0.5
    init_open_close_cmd: float = 2.0
    init_shrink_cmd: float = 20.0
    init_wait_s: float = 1.0

    # --- loose 用（extend_until...） ---
    loose_threshold: float = 0.5
    loose_extend_cmd: float = 40.0

    # extend_until_current_threshold_step が前動作/低電流安定版のとき用
    loose_preclose_cmd: float = 2.0
    loose_preclose_wait_s: float = 0.2
    loose_low_hold_s: float = 0.3
    loose_max_time: float | None = 5.0
    
def read_voltage_test(interface):
    val = interface.read_imu_data()
    print(val)
    return val

def free_gripper(interface):
    interface.angle_vector([0, 0], servo_ids=[5, 7])

def demo():
    loosen_left_stopper()    
    # loosen_right_stopper()
    interface.angle_vector([-90], servo_ids=[9])
    time.sleep(5)
    interface.angle_vector([90], servo_ids=[9])
    time.sleep(5)
    insert_left_stopper()
    # insert_right_stopper()

def hold_vial():
    print("hold vial")
    command_single_dof_for_duration(interface, cmd = 20, duration_s=4.0, servo_id = 1)
    # command_two_dof_for_duration(interface, cmd_a = 20, cmd_b = 20, duration_s=4.0, servo_a_id = 0, servo_b_id = 1)

def release_vial():
    print("release vial")
    command_two_dof_for_duration(interface, cmd_a = -20, cmd_b = -20, duration_s=3.0, servo_a_id = 0, servo_b_id = 1)
    command_two_dof_for_duration(interface, cmd_a = 30, cmd_b = 30, duration_s=0.5, servo_a_id = 0, servo_b_id = 1)

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

def command_diff_drive_for_duration(
    interface,
    close_cmd: float,
    extend_cmd: float,
    duration_s: float,
    servo_a_id: int = 5,
    servo_b_id: int = 7,
    stop_after: bool = True,
) -> Tuple[float, float]:
    """
    差動2自由度（開閉・直動）の指令を一定時間だけ与え、その後（任意で）停止する。

    Args:
        interface: ARMH7Interface 等（angle_vectorを持つ）
        close_cmd: 開閉自由度指令（正=閉/負=開）
        extend_cmd: 直動自由度指令（正=伸び/負=縮み）
        duration_s: 指令を与える時間 [s]
        servo_a_id: モータA ID（デフォルト 5）
        servo_b_id: モータB ID（デフォルト 7）
        stop_after: Trueなら duration_s 後に 0,0 を送って停止

    Returns:
        (a_cmd, b_cmd): 実際に送ったモータA/B指令値
    """
    a_cmd, b_cmd = command_diff_drive_2dof(
        interface,
        close_cmd=close_cmd,
        extend_cmd=extend_cmd,
        servo_a_id=servo_a_id,
        servo_b_id=servo_b_id,
    )

    time.sleep(float(duration_s))

    if stop_after:
        _stop_two_servos(interface, servo_a_id, servo_b_id)

    return a_cmd, b_cmd

def command_single_dof_for_duration(
    interface,
    cmd: float,
    duration_s: float,
    servo_id: int,
    stop_after: bool = True,
) -> float:
    """
    1自由度（単一サーボ）に指令を一定時間だけ与え、その後停止する。

    Args:
        interface : ARMH7Interface 等
        cmd       : サーボ指令値（正 → 閉まる方向、負 → 開く方向）
        duration_s: 指令を与える時間 [s]
        servo_id  : 対象サーボID
        stop_after: Trueなら duration_s 後に 0 指令を送る

    Returns:
        cmd : 実際に送った指令値
    """
    # 指令送信
    interface.angle_vector([float(cmd)], servo_ids=[servo_id])

    # 指定時間待つ（ブロッキング）
    time.sleep(float(duration_s))

    # 停止
    if stop_after:
        interface.angle_vector([0.0], servo_ids=[servo_id])

    return float(cmd)

def command_two_dof_for_duration(
    interface,
    cmd_a: float,
    cmd_b: float,
    duration_s: float,
    servo_a_id: int,
    servo_b_id: int,
    stop_after: bool = True,
) -> Tuple[float, float]:
    """
    差動ではない2自由度（2サーボ独立）で、
    指令を一定時間だけ与え、その後停止する。

    Args:
        interface  : ARMH7Interface 等
        cmd_a      : サーボA指令値（正 → 閉まる方向）
        cmd_b      : サーボB指令値（正 → 閉まる方向）
        duration_s : 指令を与える時間 [s]
        servo_a_id : サーボA ID
        servo_b_id : サーボB ID
        stop_after : Trueなら duration_s 後に 0 指令を送る

    Returns:
        (cmd_a, cmd_b): 実際に送った指令値
    """
    # 2サーボに同時指令
    interface.angle_vector(
        [float(cmd_a), float(cmd_b)],
        servo_ids=[servo_a_id, servo_b_id],
    )

    # 指定時間待つ（ブロッキング）
    time.sleep(float(duration_s))

    # 停止
    if stop_after:
        interface.angle_vector(
            [0.0, 0.0],
            servo_ids=[servo_a_id, servo_b_id],
        )

    return float(cmd_a), float(cmd_b)

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
    max_time: float | None = 15.0 # 暴走防止（Noneで無効）
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

def run_grippers_sequence_parallel(
    interface,
    grippers: List[GripperConfig],
    stages: Sequence[GripperStage] = ("init", "loose"),
) -> Dict[str, Dict[str, bool]]:
    """
    複数グリッパーに対して stages を順番に実行する（各stage内は同時進行）。
    例: stages=("init","loose") なら
        - 全グリッパーを同時に init 完了
        - 次に全グリッパーを同時に loose 完了

    Returns:
      {
        "init":  {"left": True, "right": True, ...},
        "loose": {"left": True, "right": True, ...},
      }
    """
    results: Dict[str, Dict[str, bool]] = {}

    def _sleep_yield():
        # CPU占有を避ける最小sleep（I/Oが即返る想定でも入れるのが無難）
        time.sleep(0.001)

    try:
        for stage in stages:
            done_map: Dict[str, bool] = {g.name: False for g in grippers}

            # stage開始（全グリッパー reset=True）
            if stage == "init":
                for g in grippers:
                    open_then_shrink_until_current_threshold_step(
                        interface,
                        open_close_cmd=g.init_open_close_cmd,
                        shrink_cmd=g.init_shrink_cmd,
                        threshold=g.init_threshold,
                        wait_s=g.init_wait_s,
                        servo_a_id=g.servo_a_id,
                        servo_b_id=g.servo_b_id,
                        reset=True,
                    )

                while not all(done_map.values()):
                    for g in grippers:
                        if done_map[g.name]:
                            continue
                        done_map[g.name] = bool(
                            open_then_shrink_until_current_threshold_step(
                                interface,
                                open_close_cmd=g.init_open_close_cmd,
                                shrink_cmd=g.init_shrink_cmd,
                                threshold=g.init_threshold,
                                wait_s=g.init_wait_s,
                                servo_a_id=g.servo_a_id,
                                servo_b_id=g.servo_b_id,
                                reset=False,
                            )
                        )
                    _sleep_yield()

            elif stage == "loose":
                for g in grippers:
                    extend_until_current_threshold_step(
                        interface,
                        extend_cmd=g.loose_extend_cmd,
                        threshold=g.loose_threshold,
                        servo_a_id=g.servo_a_id,
                        servo_b_id=g.servo_b_id,
                        reset=True,
                        # ↓ これら引数を extend_until_current_threshold_step が持つ場合のみ有効
                        preclose_cmd=g.loose_preclose_cmd,
                        preclose_wait_s=g.loose_preclose_wait_s,
                        low_hold_s=g.loose_low_hold_s,
                        max_time=g.loose_max_time,
                    )

                while not all(done_map.values()):
                    for g in grippers:
                        if done_map[g.name]:
                            continue
                        done_map[g.name] = bool(
                            extend_until_current_threshold_step(
                                interface,
                                extend_cmd=g.loose_extend_cmd,
                                threshold=g.loose_threshold,
                                servo_a_id=g.servo_a_id,
                                servo_b_id=g.servo_b_id,
                                reset=False,
                                preclose_cmd=g.loose_preclose_cmd,
                                preclose_wait_s=g.loose_preclose_wait_s,
                                low_hold_s=g.loose_low_hold_s,
                                max_time=g.loose_max_time,
                            )
                        )
                    _sleep_yield()

            else:
                raise ValueError(f"Unknown stage: {stage}")

            results[stage] = done_map

    except KeyboardInterrupt:
        # 途中停止時は関与しているサーボを止める
        for g in grippers:
            _stop_two_servos(interface, g.servo_a_id, g.servo_b_id)
        raise

    return results

def init_both_gripper():
    interface.angle_vector([-60, -60, 90], servo_ids=[2, 3, 9])
    time.sleep(3)
    res = run_grippers_sequence_parallel(interface, [left], stages=("init", "loose"))
    print(res)
        
def init_left_gripper():
    # command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)
    interface.angle_vector([-60], servo_ids=[3])
    time.sleep(3)
    # init_gripper(interface, servo_a_id = 5, servo_b_id = 7, shrink_cmd = 40)
    # loose_gripper(interface, servo_a_id = 5, servo_b_id = 7, extend_cmd = 40)

def init_right_gripper():
    # command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 4, servo_b_id = 6)
    interface.angle_vector([-60], servo_ids=[2])
    time.sleep(3)    
    init_gripper(interface, servo_a_id = 4, servo_b_id = 6, shrink_cmd = 40)
    loose_gripper(interface, servo_a_id = 4, servo_b_id = 6, extend_cmd = 40)

def loosen_left_stopper():
    print("loosen stopper")
    # init_gripper(interface, servo_a_id = 5, servo_b_id = 7, shrink_cmd = 40)
    # loose_gripper(interface, servo_a_id = 5, servo_b_id = 7, extend_cmd = 40)
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)
    interface.angle_vector([-90], servo_ids=[3])
    time.sleep(3)
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=20.0, duration_s=7.0, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=15.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=15.0, extend_cmd=-20.0, duration_s=7.0, servo_a_id = 5, servo_b_id = 7)
    interface.angle_vector([120], servo_ids=[3])
    time.sleep(5)

def loosen_left_stopper2():
    print("loosen stopper")
    # init_gripper(interface, servo_a_id = 5, servo_b_id = 7, shrink_cmd = 40)
    # loose_gripper(interface, servo_a_id = 5, servo_b_id = 7, extend_cmd = 40)
    res = run_grippers_sequence_parallel(interface, [left], stages=("init", "loose"))
    print(res)
    
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)
    interface.angle_vector([-90], servo_ids=[3])
    time.sleep(2)
    command_diff_drive_for_duration(interface, close_cmd=-4.0, extend_cmd=20.0, duration_s=3.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)

    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=-20.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=7.0, extend_cmd=0.0, duration_s=0.5, servo_a_id = 5, servo_b_id = 7)

    interface.angle_vector([120], servo_ids=[3])
    time.sleep(2)

def insert_left_stopper():
    print("insert stopper")
    interface.angle_vector([-90], servo_ids=[3])
    time.sleep(3)
    command_diff_drive_for_duration(interface, close_cmd=0.0, extend_cmd=20.0, duration_s=7, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=-4.0, extend_cmd=-20.0, duration_s=7, servo_a_id = 5, servo_b_id = 7)
    interface.angle_vector([120], servo_ids=[3])
    time.sleep(3)

def insert_left_stopper2():
    print("insert stopper")
    interface.angle_vector([-90], servo_ids=[3])
    time.sleep(2)
    command_diff_drive_for_duration(interface, close_cmd=3.0, extend_cmd=20.0, duration_s=3.5, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 5, servo_b_id = 7)
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=-20.0, duration_s=3.5, servo_a_id = 5, servo_b_id = 7)
    interface.angle_vector([120], servo_ids=[3])
    time.sleep(2)

def loosen_right_stopper():
    print("loosen stopper")
    command_diff_drive_for_duration(interface, close_cmd=-5.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 4, servo_b_id = 6)
    interface.angle_vector([-90], servo_ids=[2])
    time.sleep(3)
    command_diff_drive_for_duration(interface, close_cmd=-5.0, extend_cmd=20.0, duration_s=7.0, servo_a_id = 4, servo_b_id = 6)
    command_diff_drive_for_duration(interface, close_cmd=15.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 4, servo_b_id = 6)
    command_diff_drive_for_duration(interface, close_cmd=15.0, extend_cmd=-20.0, duration_s=7.0, servo_a_id = 4, servo_b_id = 6)
    interface.angle_vector([120], servo_ids=[2])
    time.sleep(5)

def insert_right_stopper():
    print("insert stopper")
    interface.angle_vector([-90], servo_ids=[2])
    time.sleep(3)
    command_diff_drive_for_duration(interface, close_cmd=0.0, extend_cmd=20.0, duration_s=7.0, servo_a_id = 4, servo_b_id = 6)
    command_diff_drive_for_duration(interface, close_cmd=-5.0, extend_cmd=0.0, duration_s=2.0, servo_a_id = 4, servo_b_id = 6)
    command_diff_drive_for_duration(interface, close_cmd=-2.0, extend_cmd=-20.0, duration_s=7.0, servo_a_id = 4, servo_b_id = 6)
    interface.angle_vector([120], servo_ids=[2])
    time.sleep(5)

left = GripperConfig(
    name="left",
    servo_a_id=5,
    servo_b_id=7,
    init_threshold=2.5,
    init_shrink_cmd=20.0,
    loose_threshold=0.5,
    loose_extend_cmd=40.0,
)

right = GripperConfig(
    name="right",
    servo_a_id=4,
    servo_b_id=6,
    init_threshold=2.5,   # グリッパーごとに条件を変えられる
    init_shrink_cmd=20.0,
    loose_threshold=0.5,
    loose_extend_cmd=40.0,
)

def open_gripper_init():
    print("open gripper init")
    interface.hold()
    # res = run_grippers_sequence_parallel(interface, [left], stages=("init", "loose"))
    # # res = run_grippers_sequence_parallel(interface, [left, right], stages=("init", "loose"))
    # print(res)
    # release_vial()
    init_both_gripper()

# res = run_grippers_sequence_parallel(interface, [left, right], stages=("init", "loose"))
# print(res)

def left_loop_test():
    try:
        while True:
            loosen_left_stopper2()
            insert_left_stopper2()
    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    interface = ARMH7Interface()
    try:
        print(interface.auto_open())
        interface.switch_reading_servo_current(True)
        open_gripper_init()
        left_loop_test()
        # loosen_stopper()
        # insert_stopper()
    except Exception as e:
        if "LIBUSB_ERROR_ACCESS" in str(e):
            # Error already handled and printed by the class method
            pass
        else:
            print(f"Error: {e}")


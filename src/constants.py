import enum


class CommandType(enum.Enum):
    """モーターへ送信するコマンドの種類"""

    GET_DEVICE_ID = 0x00
    ENABLE = 0x03
    DISABLE = 0x04
    READ_PARAM = 0x11
    WRITE_PARAM = 0x12


class MotorStatus(enum.Enum):
    """モーターからの応答に含まれる状態"""

    RESET = 0
    CALIBRATION = 1
    RUN = 2
    UNKNOWN = 99


class RunMode(enum.Enum):
    """モーターの運転モード"""

    OPERATION = 0
    POSITION_PP = 1
    VELOCITY = 2
    CURRENT = 3
    POSITION_CSP = 5


class ParameterIndex(enum.Enum):
    """モーターパラメータのインデックス"""

    # --- 共通 ---
    RUN_MODE = 0x7005
    LOC_REF = 0x7016

    # --- PPモード用 ---
    VEL_MAX = 0x7024
    ACC_SET = 0x7025

    # --- 電流モード用 ---
    IQ_REF = 0x7006

    # --- 速度モード用 ---
    SPD_REF = 0x700A
    LIMIT_CUR = 0x7018
    ACC_RAD = 0x7022

    # --- CSPモード用 ---
    LIMIT_SPD = 0x7017

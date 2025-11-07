from time import sleep
import numpy as np
from enum import IntEnum
from struct import unpack
from struct import pack


class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        """
        Define Motor object
        :param MotorType: Motor type
        :param SlaveID: CAN ID (Motor ID)
        :param MasterID: Master ID (recommended not to set to 0)
        """
        self.Pd = float(0)
        self.Vd = float(0)
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)
        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isEnable = False
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}

    def recv_data(self, q: float, dq: float, tau: float):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau

    def getPosition(self):
        """
        Get the position of the motor
        :return: The position of the motor
        """
        return self.state_q

    def getVelocity(self):
        """
        Get the velocity of the motor
        :return: The velocity of the motor
        """
        return self.state_dq

    def getTorque(self):
        """
        Get the torque of the motor
        :return: The torque of the motor
        """
        return self.state_tau

    def getParam(self, RID):
        """
        Get the parameter of the motor (needs to be read in advance)
        :param RID: DM_variable motor parameter
        :return: The parameter of the motor
        """
        if RID in self.temp_param_dict:
            return self.temp_param_dict[RID]
        else:
            return None


class MotorControl:
    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)
    #                4310           4310_48        4340           4340_48
    Limit_Param = [[12.5, 30, 10], [12.5, 50, 10], [12.5, 8, 28], [12.5, 10, 28],
                   # 6006           8006           8009            10010L         10010
                   [12.5, 45, 20], [12.5, 45, 40], [12.5, 45, 54], [12.5, 25, 200], [12.5, 20, 200],
                   # H3510            DMG6215      DMH6220
                   [12.5 , 280 , 1],[12.5 , 45 , 10],[12.5 , 45 , 10]]

    def __init__(self, serial_device):
        """
        Define MotorControl object
        :param serial_device: Serial object
        """
        self.serial_ = serial_device
        self.motors_map = dict()
        self.data_save = bytes()  # Save data
        if self.serial_.is_open:  # Open the serial port
            print("Serial port is open")
            serial_device.close()
        self.serial_.open()

    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        MIT Control Mode Function
        :param DM_Motor: Motor object
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param q: Desired position
        :param dq: Desired velocity
        :param tau: Desired torque
        :return: None
        """
        if DM_Motor.SlaveID not in self.motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.__send_data(DM_Motor.SlaveID, data_buf)
        self.recv()  # Receive the data from serial port

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        """
        MIT Control Mode Function with delay
        :param DM_Motor: Motor object
        :param kp: Proportional gain
        :param kd: Derivative gain
        :param q: Desired position
        :param dq: Desired velocity
        :param tau: Desired torque
        :param delay: Delay time in seconds
        """
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        """
        Control the motor in position and velocity control mode
        :param Motor: Motor object
        :param P_desired: Desired position
        :param V_desired: Desired velocity
        :return: None
        """
        if Motor.SlaveID not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(P_desired)
        V_desired_uint8s = float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        # time.sleep(0.001)
        self.recv()  # Receive the data from serial port

    def control_Vel(self, Motor, Vel_desired):
        """
        Control the motor in velocity control mode
        :param Motor: Motor object
        :param Vel_desired: Desired velocity
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(Vel_desired)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # Receive the data from serial port

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
        """
        Control the motor in EMIT control mode (position-torque hybrid mode)
        :param Pos_des: Desired position in rad
        :param Vel_des: Desired velocity in rad/s (scaled by 100)
        :param i_des: Desired current range 0-10000 (per-unit current scaled by 10000)
        Per-unit current: actual current value divided by maximum current value, see power-on printout for max current
        """
        if Motor.SlaveID not in self.motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(Pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()  # Receive the data from serial port

    def enable(self, Motor):
        """
        Enable motor
        It's recommended to enable motor a few seconds after power-on
        :param Motor: Motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()  # Receive the data from serial port

    def enable_old(self, Motor ,ControlMode):
        """
        Enable motor for old firmware (compatibility for older motor firmware)
        Old firmware requires an offset for enabling
        It's recommended to enable motor a few seconds after power-on
        :param Motor: Motor object
        """
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc], np.uint8)
        enable_id = ((int(ControlMode)-1) << 2) + Motor.SlaveID
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()  # Receive the data from serial port

    def disable(self, Motor):
        """
        Disable motor
        :param Motor: Motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.01)

    def set_zero_position(self, Motor):
        """
        Set the zero position of the motor
        :param Motor: Motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()  # Receive the data from serial port

    def recv(self):
        # Include remaining unparsed data from last time
        data_recv = b''.join([self.data_save, self.serial_.read_all()])
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)

    def recv_set_param_data(self):
        data_recv = self.serial_.read_all()
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_set_param_packet(data, CANID, CMD)

    def __process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID != 0x00:
                if CANID in self.motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                    MotorType_recv = self.motors_map[CANID].MotorType
                    Q_MAX = self.Limit_Param[MotorType_recv][0]
                    DQ_MAX = self.Limit_Param[MotorType_recv][1]
                    TAU_MAX = self.Limit_Param[MotorType_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)
            else:
                MasterID=data[0] & 0x0f
                if MasterID in self.motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                    MotorType_recv = self.motors_map[MasterID].MotorType
                    Q_MAX = self.Limit_Param[MotorType_recv][0]
                    DQ_MAX = self.Limit_Param[MotorType_recv][1]
                    TAU_MAX = self.Limit_Param[MotorType_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    self.motors_map[MasterID].recv_data(recv_q, recv_dq, recv_tau)


    def __process_set_param_packet(self, data, CANID, CMD):
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
            masterid=CANID
            slaveId = ((data[1] << 8) | data[0])
            if CANID==0x00:  # Prevent issues when MasterID is set to 0
                masterid=slaveId

            if masterid not in self.motors_map:
                if slaveId not in self.motors_map:
                    return
                else:
                    masterid=slaveId

            RID = data[3]
            # Data obtained from reading parameters
            if is_in_ranges(RID):
                # uint32 type
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num

            else:
                # float type
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[masterid].temp_param_dict[RID] = num


    def addMotor(self, Motor):
        """
        Add motor to the motor control object
        :param Motor: Motor object
        """
        self.motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.motors_map[Motor.MasterID] = Motor
        return True

    def __control_cmd(self, Motor, cmd: np.uint8):
        data_buf = np.array([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)

    def __send_data(self, motor_id, data):
        """
        Send data to the motor
        :param motor_id:
        :param data:
        :return:
        """
        self.send_data_frame[13] = motor_id & 0xff
        self.send_data_frame[14] = (motor_id >> 8)& 0xff  # ID high 8 bits
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def __read_RID_param(self, Motor, RID):
        can_id_l = Motor.SlaveID & 0xff # ID low 8 bits
        can_id_h = (Motor.SlaveID >> 8)& 0xff  # ID high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x33, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, Motor, RID, data):
        can_id_l = Motor.SlaveID & 0xff # ID low 8 bits
        can_id_h = (Motor.SlaveID >> 8)& 0xff  # ID high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x55, np.uint8(RID), 0x00, 0x00, 0x00, 0x00], np.uint8)
        if not is_in_ranges(RID):
            # Data is float
            data_buf[4:8] = float_to_uint8s(data)
        else:
            # Data is int
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        """
        Switch the control mode of the motor
        :param Motor: Motor object
        :param ControlMode: Control_Type motor control mode (example: MIT mode: Control_Type.MIT)
        """
        max_retries = 10
        retry_interval = 0.05  # Retry interval
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    if self.motors_map[Motor.SlaveID].temp_param_dict[RID] == ControlMode:
                        return True
                    else:
                        return False
        return False

    def save_motor_param(self, Motor):
        """
        Save all parameters to flash
        :param Motor: Motor object
        :return:
        """
        can_id_l = Motor.SlaveID & 0xff # ID low 8 bits
        can_id_h = (Motor.SlaveID >> 8)& 0xff  # ID high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.disable(Motor)  # Disable the motor before saving
        self.__send_data(0x7FF, data_buf)
        sleep(0.001)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        Change the PMAX VMAX TMAX of the motor
        :param Motor_Type:
        :param PMAX: Motor's PMAX
        :param VMAX: Motor's VMAX
        :param TMAX: Motor's TMAX
        :return:
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self,Motor):
        """
        Get the motor status
        """
        can_id_l = Motor.SlaveID & 0xff # ID low 8 bits
        can_id_h = (Motor.SlaveID >> 8) & 0xff  # ID high 8 bits
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        self.__send_data(0x7FF, data_buf)
        self.recv()  # Receive the data from serial port

    def change_motor_param(self, Motor, RID, data):
        """
        Change the RID parameter of the motor
        :param Motor: Motor object
        :param RID: DM_variable motor parameter
        :param data: Value of the motor parameter
        :return: True or False, True means success, False means fail
        """
        max_retries = 20
        retry_interval = 0.05  # Retry interval

        self.__write_motor_param(Motor, RID, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map and RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                if abs(self.motors_map[Motor.SlaveID].temp_param_dict[RID] - data) < 0.1:
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def read_motor_param(self, Motor, RID):
        """
        Read the RID parameter of the motor (e.g., version number, etc.)
        :param Motor: Motor object
        :param RID: DM_variable motor parameter
        :return: Value of the motor parameter
        """
        max_retries = 20
        retry_interval = 0.05  # Retry interval
        self.__read_RID_param(Motor, RID)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.motors_map:
                if RID in self.motors_map[Motor.SlaveID].temp_param_dict:
                    return self.motors_map[Motor.SlaveID].temp_param_dict[RID]
                else:
                    return None
        return None

    # -------------------------------------------------
    # Extract packets from the serial data
    def __extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames


def LIMIT_MIN_MAX(x, min, max):
    if x <= min:
        x = min
    elif x > max:
        x = max


def float_to_uint(x: float, x_min: float, x_max: float, bits):
    LIMIT_MIN_MAX(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, min: float, max: float, bits):
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


def float_to_uint8s(value):
    # Pack the float into 4 bytes
    packed = pack('f', value)
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def data_to_uint8s(value):
    # Check if the value is within the range of uint32
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # Pack the uint32 into 4 bytes
        packed = pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")

    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def is_in_ranges(number):
    """
    Check if the number is in the range of uint32
    :param number:
    :return:
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


def uint8s_to_uint32(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single uint32 value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a uint32 value
    return unpack('<I', packed)[0]


def uint8s_to_float(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single float value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a float value
    return unpack('<f', packed)[0]


def print_hex(data):
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))


def get_enum_by_index(index, enum_class):
    try:
        return enum_class(index)
    except ValueError:
        return None


class DM_Motor_Type(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM4340_48V = 3
    DM6006 = 4
    DM8006 = 5
    DM8009 = 6
    DM10010L = 7
    DM10010 = 8
    DMH3510 = 9
    DMH6215 = 10
    DMG6220 = 11


class DM_variable(IntEnum):
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36
    u_off = 50
    v_off = 51
    k1 = 52
    k2 = 53
    m_off = 54
    dir = 55
    p_m = 80
    xout = 81


class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    Torque_Pos = 4

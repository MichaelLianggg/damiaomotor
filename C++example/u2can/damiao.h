#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include <cmath>

#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300
#define max_retries 20
#define retry_interval 50000
namespace damiao
{
#pragma pack(1)
#define Motor_id uint32_t

    /*!
     * @brief Motor Type
     */
    enum DM_Motor_Type
    {
        DM4310,
        DM4310_48V,
        DM4340,
        DM4340_48V,
        DM6006,
        DM8006,
        DM8009,
        DM10010L,
        DM10010,
        DMH3510,
        DMH6215,
        DMG6220,
        Num_Of_Motor
    };

    /*
     * @brief Motor control mode
     */
    enum Control_Mode
    {
        MIT_MODE=1,
        POS_VEL_MODE=2,
        VEL_MODE=3,
        POS_FORCE_MODE=4,
    };

    /*
     * @brief Register list, refer to DaMiao manual for details
     */
    enum DM_REG
    {
        UV_Value = 0,
        KT_Value = 1,
        OT_Value = 2,
        OC_Value = 3,
        ACC = 4,
        DEC = 5,
        MAX_SPD = 6,
        MST_ID = 7,
        ESC_ID = 8,
        TIMEOUT = 9,
        CTRL_MODE = 10,
        Damp = 11,
        Inertia = 12,
        hw_ver = 13,
        sw_ver = 14,
        SN = 15,
        NPP = 16,
        Rs = 17,
        LS = 18,
        Flux = 19,
        Gr = 20,
        PMAX = 21,
        VMAX = 22,
        TMAX = 23,
        I_BW = 24,
        KP_ASR = 25,
        KI_ASR = 26,
        KP_APR = 27,
        KI_APR = 28,
        OV_Value = 29,
        GREF = 30,
        Deta = 31,
        V_BW = 32,
        IQ_c1 = 33,
        VL_c1 = 34,
        can_br = 35,
        sub_ver = 36,
        u_off = 50,
        v_off = 51,
        k1 = 52,
        k2 = 53,
        m_off = 54,
        dir = 55,
        p_m = 80,
        xout = 81,
    };

    typedef struct
    {
        uint8_t FrameHeader;
        uint8_t CMD;// Command 0x00: heartbeat
        //     0x01: receive fail 0x11: receive success
        //     0x02: send fail 0x12: send success
        //     0x03: set baudrate fail 0x13: set baudrate success
        //     0xEE: communication error, format field contains error code at this time
        //     8: overvoltage 9: undervoltage A: overcurrent B: MOS overtemperature C: motor coil overtemperature D: communication loss E: overload
        uint8_t canDataLen: 6; // Data length
        uint8_t canIde: 1; // 0: standard frame 1: extended frame
        uint8_t canRtr: 1; // 0: data frame 1: remote frame
        uint32_t canId; // Motor feedback ID
        uint8_t canData[8];
        uint8_t frameEnd; // Frame end
    } CAN_Receive_Frame;

    typedef struct can_send_frame
    {
        uint8_t FrameHeader[2] = {0x55, 0xAA}; // Frame header
        uint8_t FrameLen = 0x1e; // Frame length
        uint8_t CMD = 0x03; // Command 1: forward CAN data frame 2: handshake between PC and device, device responds OK 3: non-feedback CAN forwarding, no send status feedback
        uint32_t sendTimes = 1; // Send times
        uint32_t timeInterval = 10; // Time interval
        uint8_t IDType = 0; // ID type 0: standard frame 1: extended frame
        uint32_t canId=0x01; // CAN ID, use motor ID as CAN ID
        uint8_t frameType = 0; // Frame type 0: data frame 1: remote frame
        uint8_t len = 0x08; // len
        uint8_t idAcc=0;
        uint8_t dataAcc=0;
        uint8_t data[8]={0};
        uint8_t crc=0; // Not parsed, any value

        void modify(const Motor_id id, const uint8_t* send_data)
        {
            canId = id;
            std::copy(send_data, send_data+8, data);
        }

    } can_send_frame;

#pragma pack()

    typedef struct
    {
        float Q_MAX;
        float DQ_MAX;
        float TAU_MAX;
    }Limit_param;

    // Motor PMAX DQMAX TAUMAX parameters
    Limit_param limit_param[Num_Of_Motor]=
            {
                    {12.5, 30, 10 }, // DM4310
                    {12.5, 50, 10 }, // DM4310_48V
                    {12.5, 8, 28 },  // DM4340
                    {12.5, 10, 28 }, // DM4340_48V
                    {12.5, 45, 20 }, // DM6006
                    {12.5, 45, 40 }, // DM8006
                    {12.5, 45, 54 }, // DM8009
                    {12.5,25,  200}, // DM10010L
                    {12.5,20, 200},  // DM10010
                    {12.5,280,1},    // DMH3510
                    {12.5,45,10},    // DMH6215
                    {12.5,45,10}     // DMG6220
            };

    class Motor
    {
    private:
        /* data */
        Motor_id Master_id;
        Motor_id Slave_id;
        float state_q=0;
        float state_dq=0;
        float state_tau=0;
        Limit_param limit_param{};
        DM_Motor_Type Motor_Type;

        union ValueUnion {
            float floatValue;
            uint32_t uint32Value;
        };

        struct ValueType {
            ValueUnion value;
            bool isFloat;
        };

        std::unordered_map<uint32_t , ValueType> param_map;
    public:
        /**
         * @brief Construct a new Motor object
         *
         * @param Motor_Type Motor type
         * @param Slave_id CAN ID, slave ID (motor ID)
         * @param Master_id Master ID, it is recommended not to set all master IDs to 0x00
         *
         */
        Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
                : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type) {
            this->limit_param = damiao::limit_param[Motor_Type];
        }

        Motor() : Master_id(0x01), Slave_id(0x11), Motor_Type(DM4310) {
            this->limit_param = damiao::limit_param[DM4310];
        }

        void receive_data(float q, float dq, float tau)
        {
            this->state_q = q;
            this->state_dq = dq;
            this->state_tau = tau;
        }

        DM_Motor_Type GetMotorType() const { return this->Motor_Type; }

        /*
         * @brief get master id
         * @return MasterID
         */
        Motor_id GetMasterId() const { return this->Master_id; }

        /*
         * @brief get motor slave id (CAN ID)
         * @return SlaveID
         */
        Motor_id GetSlaveId() const { return this->Slave_id; }

        /*
         * @brief get motor position
         * @return motor position
         */
        float Get_Position() const { return this->state_q; }

        /*
         * @brief get motor velocity
         * @return motor velocity
         */
        float Get_Velocity() const { return this->state_dq; }

        /*
         * @brief get torque of the motor
         * @return motor torque (actual output torque)
         */
        float Get_tau() const { return this->state_tau; }

        /*
         * @brief get limit param
         * @return limit_param (motor limit parameters)
         */
        Limit_param get_limit_param() { return limit_param; }

        void set_param(int key, float value)
        {
            ValueType v{};
            v.value.floatValue = value;
            v.isFloat = true;
            param_map[key] = v;
        }

        void set_param(int key, uint32_t value)
        {
            ValueType v{};
            v.value.uint32Value = value;
            v.isFloat = false;
            param_map[key] = v;
        }

        float get_param_as_float(int key) const
        {
            auto it = param_map.find(key);
            if (it != param_map.end())
            {
                if (it->second.isFloat)
                {
                    return it->second.value.floatValue;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        uint32_t get_param_as_uint32(int key) const {
            auto it = param_map.find(key);
            if (it != param_map.end()) {
                if (!it->second.isFloat) {
                    return it->second.value.uint32Value;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        bool is_have_param(int key) const
        {
            return param_map.find(key) != param_map.end();
        }

    };


/**
 * @brief motor control class
 *
 * Uses USB-to-CAN for communication, with Linux as a virtual serial port
 */
    class Motor_Control
    {
    public:

        /*
        * @brief Define motor control object
        * @param serial Serial port object
        * Default serial port is /dev/ttyACM0
        */
        Motor_Control(SerialPort::SharedPtr serial = nullptr): serial_(std::move(serial))
        {
            if (serial_ == nullptr) {
                //default serial port
                serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
            }
        }

        ~Motor_Control()
        = default;

        /*
        * @brief enable the motor
        * @param motor Motor object
        */
        void enable(const Motor& motor)
        {
            control_cmd(motor.GetSlaveId(), 0xFC);
            usleep(100000);//100ms
            this->receive();
        }

        /*
         * @brief enable motor with old version firmware, it is recommended to upgrade to new version as soon as possible
         * @param motor Motor object
         * @param mode Control mode: damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
         */
        void enable_old(const Motor& motor, Control_Mode mode)
        {
            uint32_t id = ((mode -1) << 2) + motor.GetSlaveId();
            control_cmd(id, 0xFC);
            usleep(100000);
            this->receive();
        }

        /*
         * @brief refresh motor status
         * @param motor Motor object
         */
        void refresh_motor_status(const Motor& motor)
        {
            uint32_t id = 0x7FF;
            uint8_t can_low = motor.GetSlaveId() & 0xff; // id low 8 bit
            uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; //id high 8 bit
            std::array<uint8_t, 8> data_buf = {can_low,can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
            this->receive();
        }
        /*
        * @brief  disable the motor
        * @param  motor Motor object
        */
        void disable(const Motor& motor) {
            control_cmd(motor.GetSlaveId(), 0xFD);
            usleep(100000);
            this->receive();
        }

        /*
        * @brief set the current position as zero point
        * @param motor Motor object
        */
        void set_zero_position(const Motor& motor)
        {
            control_cmd(motor.GetSlaveId(), 0xFE);
            usleep(100000);
            this->receive();
        }

        /* @description: MIT Control Mode, refer to DaMiao manual for detailed description
         *@param kp: Proportional coefficient
         *@param kd: Differential coefficient
         *@param q: position
         *@param dq: velocity
         *@param tau: torque
        */
        void control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
        {
            // Position, velocity and torque use linear mapping to convert floating point data to signed fixed-point data
            static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
                float span = xmax - xmin;
                float data_norm = (x - xmin) / span;
                uint16_t data_uint = data_norm * ((1 << bits) - 1);
                return data_uint;
            };
            Motor_id id = DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            auto& m = motors[id];
            uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
            uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
            Limit_param limit_param_cmd = m->get_limit_param();
            uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
            uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
            uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = (q_uint >> 8) & 0xff;
            data_buf[1] = q_uint & 0xff;
            data_buf[2] = dq_uint >> 4;
            data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
            data_buf[4] = kp_uint & 0xff;
            data_buf[5] = kd_uint >> 4;
            data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
            data_buf[7] = tau_uint & 0xff;

            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description: Position Control Mode with velocity
         * @param pos: position
         * @param vel: velocity
         * @param DM_Motor: Motor object
         */
        void control_pos_vel(Motor &DM_Motor,float pos,float vel)
        {
            Motor_id id = DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
            }
            std::array<uint8_t, 8> data_buf{};
            memcpy(data_buf.data(), &pos, sizeof(float));
            memcpy(data_buf.data() + 4, &vel, sizeof(float));
            id += POS_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send(reinterpret_cast<uint8_t*>(&send_data), sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description: velocity control mode
         * @param DM_Motor: Motor object
         * @param vel: velocity
         */
        void control_vel(Motor &DM_Motor,float vel)
        {
            Motor_id id =DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("VEL ERROR : id not found");
            }
            std::array<uint8_t, 8> data_buf = {0};
            memcpy(data_buf.data(), &vel, sizeof(float));
            id=id+SPEED_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description: Position control mode with torque (force-position hybrid control mode)
         * @param DM_Motor: Motor object
         * @param pos: position
         * @param vel: velocity, range 0-10000, refer to DaMiao manual for details
         * @param i: current, range 0-10000, refer to DaMiao manual for details
         */
        void control_pos_force(Motor &DM_Motor,float pos, uint16_t vel, uint16_t i)
        {
            Motor_id id =DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("pos_force ERROR : Motor_Control id not found");
            }
            std::array<uint8_t, 8> data_buf{};
            memcpy(data_buf.data(), &pos, sizeof(float));
            memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t));
            memcpy(data_buf.data() + 6, &i, sizeof(uint16_t));
            id=id+POSI_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description Internal function call to parse motor parameters returned by CAN
         */
        void receive()
        {
            serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

            if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
            {
                static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                    float span = xmax - xmin;
                    float data_norm = float(x) / ((1 << bits) - 1);
                    float data = data_norm * span + xmin;
                    return data;
                };

                auto & data = receive_data.canData;

                uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
                uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
                uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];
                if(receive_data.canId != 0x00)   //make sure the motor id is not 0x00
                {
                    if(motors.find(receive_data.canId) == motors.end())
                    {
                        return;
                    }

                    auto m = motors[receive_data.canId];
                    Limit_param limit_param_receive = m->get_limit_param();
                    float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                    float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                    m->receive_data(receive_q, receive_dq, receive_tau);
                }
                else //why the user set the masterid as 0x00 ???
                {
                    uint32_t slaveID = data[0] & 0x0f;
                    if(motors.find(slaveID) == motors.end())
                    {
                        return;
                    }
                    auto m = motors[slaveID];
                    Limit_param limit_param_receive = m->get_limit_param();
                    float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                    float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                    m->receive_data(receive_q, receive_dq, receive_tau);
                }
                return;
            }
            else if (receive_data.CMD == 0x01) // receive fail
            {
                /* code */
            }
            else if (receive_data.CMD == 0x02) // send fail
            {
                /* code */
            }
            else if (receive_data.CMD == 0x03) // send success
            {
                /* code */
            }
            else if (receive_data.CMD == 0xEE) // communication error
            {
                /* code */
            }
        }

        void receive_param()
        {
            serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

            if(receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
            {
                auto & data = receive_data.canData;
                if(data[2]==0x33 or data[2]==0x55)
                {
                    uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
                    uint8_t RID = data[3];
                    if (motors.find(slaveID) == motors.end())
                    {
                        //can not found motor id
                        return;
                    }
                    if(is_in_ranges(RID))
                    {
                        uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                        motors[slaveID]->set_param(RID, data_uint32);
                    }
                    else
                    {
                        float data_float = uint8_to_float(data + 4);
                        motors[slaveID]->set_param(RID, data_float);
                    }
                }
                return ;
            }
        }

        /**
         * @brief add motor to class
         * @param DM_Motor : Motor object
         */
        void addMotor(Motor *DM_Motor)
        {
            motors.insert({DM_Motor->GetSlaveId(), DM_Motor});
            if (DM_Motor->GetMasterId() != 0)
            {
                motors.insert({DM_Motor->GetMasterId(), DM_Motor});
            }
        }

        /*
         * @description: read motor register param, refer to DaMiao manual for detailed register list
         * @param DM_Motor: Motor object
         * @param RID: register id, example: damiao::UV_Value
         * @return: motor param, returns 0 if not queried successfully
         */
        float read_motor_param(Motor &DM_Motor,uint8_t RID)
        {
            uint32_t id = DM_Motor.GetSlaveId();
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
            for(uint8_t i =0;i<max_retries;i++)
            {
                usleep(retry_interval);
                receive_param();
                if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return float(motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID));
                    }
                    else
                    {
                        return motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID);
                    }
                }
            }

            return 0;
        }


        /*
         * @description: switch control mode
         * @param DM_Motor: Motor object
         * @param mode: control mode like: damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
         */
        bool switchControlMode(Motor &DM_Motor,Control_Mode mode)
        {
            uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
            uint8_t RID = 10;
            write_motor_param(DM_Motor,RID,write_data);
            if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            for(uint8_t i =0;i<max_retries;i++)
            {
                usleep(retry_interval);
                receive_param();
                if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
                }
            }
            return false;
        }

        /*
         * @description: change motor param, modify motor internal register parameters, refer to DaMiao manual for detailed register list
         * @param DM_Motor: Motor object
         * @param RID: register id
         * @param data: param data, most data is float type, but if it is uint32 type, you can directly input integer, the function handles it internally
         * @return: bool true or false, whether the modification was successful
         */
        bool change_motor_param(Motor &DM_Motor,uint8_t RID,float data)
        {
            if(is_in_ranges(RID)) {
                // It's actually an integer range, let's handle it
                uint32_t data_uint32 = float_to_uint32(data);
                uint8_t *data_uint8;
                data_uint8=(uint8_t*)&data_uint32;
                write_motor_param(DM_Motor,RID,data_uint8);
            }
            else
            {
                //is float
                uint8_t *data_uint8;
                data_uint8=(uint8_t*)&data;
                write_motor_param(DM_Motor,RID,data_uint8);
            }
            if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            for(uint8_t i =0;i<max_retries;i++)
            {
                usleep(retry_interval);
                receive_param();
                if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
                    }
                    else
                    {
                        return fabsf(motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID) - data)<0.1f;
                    }
                }
            }
            return false;
        }


        /*
         * @description: save all param to motor flash
         * @param DM_Motor: Motor object
         * By default, motor parameters are not written to flash and need to be written explicitly
         */
        void save_motor_param(Motor &DM_Motor)
        {
            disable(DM_Motor);
            uint32_t id = DM_Motor.GetSlaveId();
            uint8_t id_low = id & 0xff;
            uint8_t id_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
            usleep(100000);//100ms wait for save
        }

        /*
         * @description: change motor limit param, this does not modify the motor's internal register parameters, but the motor's limit parameters
         * @param DM_Motor: Motor object
         * @param P_MAX: position max
         * @param Q_MAX: velocity max
         * @param T_MAX: torque max
         */
        static void changeMotorLimit(Motor &DM_Motor,float P_MAX,float Q_MAX,float T_MAX)
        {
            limit_param[DM_Motor.GetMotorType()]={P_MAX,Q_MAX,T_MAX};
        }

        /*
         * @description: change motor PMAX
         * @param DM_Motor: Motor object
         * @param P_MAX: position max
         */
        static void changeMotorPMAX(Motor &DM_Motor,float P_MAX)
        {
            limit_param[DM_Motor.GetMotorType()].Q_MAX=P_MAX;
        }

    private:
        void control_cmd(Motor_id id , uint8_t cmd)
        {
            std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        }

        void write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
        {
            uint32_t id = DM_Motor.GetSlaveId();
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
            data_buf[4] = data[0];
            data_buf[5] = data[1];
            data_buf[6] = data[2];
            data_buf[7] = data[3];
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(can_send_frame));
        }

        static bool is_in_ranges(int number) {
            return (7 <= number && number <= 10) ||
                   (13 <= number && number <= 16) ||
                   (35 <= number && number <= 36);
        }

        static uint32_t float_to_uint32(float value) {
            return static_cast<uint32_t>(value);
        }

        static float uint32_to_float(uint32_t value) {
            return static_cast<float>(value);
        }

        static float uint8_to_float(const uint8_t data[4]) {
            uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                                (static_cast<uint32_t>(data[2]) << 16) |
                                (static_cast<uint32_t>(data[1]) << 8)  |
                                static_cast<uint32_t>(data[0]);
            float result;
            memcpy(&result, &combined, sizeof(result));
            return result;
        }

        std::unordered_map<Motor_id, Motor*> motors;
        SerialPort::SharedPtr serial_;  // serial port
        can_send_frame send_data; // send data frame
        CAN_Receive_Frame receive_data{};// receive data frame
    };

};

#endif
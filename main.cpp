#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstring>

#define START_FRAME 0xA5
#define CMD_ID_GAME_STATUS 0x0001
#define CMD_ID_GAME_RESULT 0x0002
#define CMD_ID_GAME_ROBOT_HP 0x0003
#define CMD_ID_EVENT_DATA 0x0101
#define CMD_ID_REFEREE_WARNING 0x0104
#define CMD_ID_DART_INFO 0x0105
#define CMD_ID_ROBOT_STATUS 0x0201
#define CMD_ID_POWER_HEAT_DATA 0x0202
#define CMD_ID_ROBOT_POS 0x0203
#define CMD_ID_BUFF 0x0204
#define CMD_ID_HURT_DATA 0x0206
#define CMD_ID_SHOOT_DATA 0x0207
#define CMD_ID_PROJECTILE_ALLOWANCE 0x0208
#define CMD_ID_RFID_STATUS 0x0209
#define CMD_ID_DART_CLIENT_CMD 0x020A
#define CMD_ID_GROUND_ROBOT_POSITION 0x020B
#define CMD_ID_RADAR_MARK_DATA 0x020C
#define CMD_ID_SENTRY_INFO 0x020D
#define CMD_ID_RADAR_INFO 0x020E
#define CMD_ID_ROBOT_INTERACTION_DATA 0x0301

const uint8_t CRC8_Table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35};

// CRC16 校验表
const uint16_t CRC16_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

// 计算 CRC8
uint8_t calculateCRC8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc = CRC8_Table[crc ^ data[i]];
    }
    return crc;
}

// 计算 CRC16
uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc = (crc >> 8) ^ CRC16_Table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/**
 * @brief 比赛状态数据，固定以1Hz频率发送
 * @param game_type 比赛类型，`1`RoboMaster 机甲大师超级对抗赛，`2`RoboMaster 机甲大师高校单项赛，`3`ICRA RoboMaster 高校人工智能挑战赛，`4`RoboMaster 机甲大师高校联盟赛 3V3 对抗，'5'RoboMaster 机甲大师高校联盟赛步兵对抗
 * @param game_progress 当前比赛阶段。`0`未开始比赛，`1`准备阶段，`2`十五秒裁判系统自检阶段，`3`五秒倒计时，`4`比赛中，`5`比赛结算中
 * @param stage_remain_time 当前阶段剩余时间，单位：秒
 * @param sync_time_stamp UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效秒
 */
struct game_status_t
{
    uint8_t game_type;
    uint8_t game_progress;
    union stage_remain_time_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } stage_remain_time;
    union sync_time_stamp_t
    {
        uint8_t raw_data[8];
        uint16_t value;
    } sync_time_stamp;
};

struct game_result_t
{
    uint8_t winner;
};

struct game_robot_HP_t
{
    union red_1_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_1_robot_HP;
    union red_2_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_2_robot_HP;
    union red_3_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_3_robot_HP;
    union red_4_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_4_robot_HP;
    union red_reserved_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_reserved;
    union red_7_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_7_robot_HP;
    union red_outpost_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_outpost_HP;
    union red_base_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } red_base_HP;
    union blue_1_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_1_robot_HP;
    union blue_2_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_2_robot_HP;
    union blue_3_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_3_robot_HP;
    union blue_4_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_4_robot_HP;
    union blue_reserved_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_reserved;
    union blue_7_robot_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_7_robot_HP;
    union blue_outpost_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_outpost_HP;
    union blue_base_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } blue_base_HP;
};

union event_data_t
{
    uint8_t raw_data[4];
    uint32_t value;
};

/**
 * @brief 裁判警告数据，己方判罚/判负时触发发送，其余时间以1Hz频率发送
 * @param level 己方最后一次受到判罚的等级, `1`双方黄牌，`2`黄牌，`3`红牌，`4`判负
 * @param offending_robot_id 己方最后一次受到判罚的违规机器人 ID。（如红1机器人ID为1，蓝1机器人ID为101）。判负和双方黄牌时，该值为0
 * @param count 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为0）
 */
struct referee_warning_t
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
};

struct dart_info_t
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
};

struct shoot_data_t
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    union initial_speed_t
    {
        uint8_t raw_data[4];
        float value;
    } initial_speed;
};

// robot_status_t 结构体
struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    union current_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } current_HP;
    union maximum_HP_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } maximum_HP;
    union shooter_barrel_cooling_value_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } shooter_barrel_cooling_value;
    union shooter_barrel_heat_limit_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } shooter_barrel_heat_limit;
    union chassis_power_limit_t
    {
        uint8_t raw_data[2];
        uint16_t value;
    } chassis_power_limit;
    union power_management_output_t
    {
        uint8_t raw_data;
        bool value[8];
    } power_management_output;
};

// 串口初始化
int initSerial(const char *port)
{
    struct termios newstate;
    bzero(&newstate, sizeof(newstate));
    int fd = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        std::cerr << "无法打开串口 " << port << std::endl;
        return -1;
    }

    cfsetospeed(&newstate, B115200);
    cfsetispeed(&newstate, B115200);

    newstate.c_cflag |= CLOCAL | CREAD;
    newstate.c_cflag &= ~CSIZE;
    newstate.c_cflag &= ~CSTOPB;
    newstate.c_cflag |= CS8;
    newstate.c_cflag &= ~PARENB;

    newstate.c_cc[VTIME] = 0;
    newstate.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newstate);
    return fd;
}

// 读取数据
void receiveData(int fd)
{
    std::vector<uint8_t> buffer;
    uint8_t temp;

    while (true)
    {
        if (read(fd, &temp, sizeof(temp)) > 0)
        {
            buffer.push_back(temp);

            // 检查帧头
            if (buffer[0] != START_FRAME)
            {
                buffer.clear();
                continue;
            }

            //  至少需要 5 字节才能解析帧头
            if (buffer.size() >= 5)
            {
                uint16_t data_length = buffer[1] | ((((uint16_t)buffer[2]) << 8) & 0xff00);
                uint8_t packet_seq = buffer[3];
                uint8_t header_crc = buffer[4];

                // 校验 CRC8
                if ((calculateCRC8(buffer.data(), 4)) != header_crc)
                {
                    std::cerr << "帧头 CRC8 校验失败 " << std::hex << (int)calculateCRC8(buffer.data(), 4) << std::endl;
                    std::cout << "length:" << std::hex << (int)buffer[0] << ' ' << (int)buffer[1] << ' ' << (int)buffer[2] << std::endl;
                    buffer.clear();
                    continue;
                }

                size_t total_length = 5 + 2 + data_length + 2;

                if (buffer.size() == total_length)
                {
                    uint16_t cmd_id = buffer[5] | ((((uint16_t)buffer[6]) << 8) & 0xff00);
                    uint16_t received_crc16 = buffer[total_length - 2] | ((((uint16_t)buffer[total_length - 1]) << 8) & 0xff00);
                    std::cout << "----------cmd_id:" << std::hex << cmd_id << "----------" << std::endl;
                    // 校验 CRC16
                    if (calculateCRC16(buffer.data(), total_length - 2) != received_crc16)
                    {
                        std::cerr << "整包 CRC16 校验失败" << std::endl;
                        buffer.clear();
                        continue;
                    }

                    switch (cmd_id)
                    {
                    case CMD_ID_GAME_STATUS:
                        game_status_t game_status;
                        game_status.game_type = (buffer[7] & 0x0F);
                        game_status.game_progress = ((buffer[7] & 0xF0) >> 4);
                        game_status.stage_remain_time.raw_data[0] = buffer[8];
                        game_status.stage_remain_time.raw_data[1] = buffer[9];
                        for (int i = 0; i < 8; i++)
                        {
                            game_status.sync_time_stamp.raw_data[i] = buffer[i + 10];
                        }
                        std::cout << "比赛类型: " << std::dec << (int)game_status.game_type << std::endl;
                        std::cout << "当前比赛阶段: " << (int)game_status.game_progress << std::endl;
                        std::cout << "当前阶段剩余时间: " << (int)game_status.stage_remain_time.value << std::endl;
                        std::cout << "UNIX 时间: " << (unsigned long long)game_status.sync_time_stamp.value << std::endl;
                        break;
                    case CMD_ID_GAME_RESULT:
                        game_result_t game_result;
                        game_result.winner = buffer[7];
                        std::cout << "比赛结果: " << std::dec << (int)game_result.winner << std::endl;
                        break;
                    case CMD_ID_GAME_ROBOT_HP:
                        game_robot_HP_t game_robot_HP;
                        game_robot_HP.red_1_robot_HP.raw_data[0] = buffer[7];
                        game_robot_HP.red_1_robot_HP.raw_data[1] = buffer[8];
                        game_robot_HP.red_2_robot_HP.raw_data[0] = buffer[9];
                        game_robot_HP.red_2_robot_HP.raw_data[1] = buffer[10];
                        game_robot_HP.red_3_robot_HP.raw_data[0] = buffer[11];
                        game_robot_HP.red_3_robot_HP.raw_data[1] = buffer[12];
                        game_robot_HP.red_4_robot_HP.raw_data[0] = buffer[13];
                        game_robot_HP.red_4_robot_HP.raw_data[1] = buffer[14];
                        game_robot_HP.red_reserved.raw_data[0] = buffer[15];
                        game_robot_HP.red_reserved.raw_data[1] = buffer[16];
                        game_robot_HP.red_7_robot_HP.raw_data[0] = buffer[17];
                        game_robot_HP.red_7_robot_HP.raw_data[1] = buffer[18];
                        game_robot_HP.red_outpost_HP.raw_data[0] = buffer[19];
                        game_robot_HP.red_outpost_HP.raw_data[1] = buffer[20];
                        game_robot_HP.red_base_HP.raw_data[0] = buffer[21];
                        game_robot_HP.red_base_HP.raw_data[1] = buffer[22];

                        game_robot_HP.blue_1_robot_HP.raw_data[0] = buffer[23];
                        game_robot_HP.blue_1_robot_HP.raw_data[1] = buffer[24];
                        game_robot_HP.blue_2_robot_HP.raw_data[0] = buffer[25];
                        game_robot_HP.blue_2_robot_HP.raw_data[1] = buffer[26];
                        game_robot_HP.blue_3_robot_HP.raw_data[0] = buffer[27];
                        game_robot_HP.blue_3_robot_HP.raw_data[1] = buffer[28];
                        game_robot_HP.blue_4_robot_HP.raw_data[0] = buffer[29];
                        game_robot_HP.blue_4_robot_HP.raw_data[1] = buffer[30];
                        game_robot_HP.blue_reserved.raw_data[0] = buffer[31];
                        game_robot_HP.blue_reserved.raw_data[1] = buffer[32];
                        game_robot_HP.blue_7_robot_HP.raw_data[0] = buffer[33];
                        game_robot_HP.blue_7_robot_HP.raw_data[1] = buffer[34];
                        game_robot_HP.blue_outpost_HP.raw_data[0] = buffer[35];
                        game_robot_HP.blue_outpost_HP.raw_data[1] = buffer[36];
                        game_robot_HP.blue_base_HP.raw_data[0] = buffer[37];
                        game_robot_HP.blue_base_HP.raw_data[1] = buffer[38];

                        std::cout << "红1英雄机器人血量: " << std::dec << (int)game_robot_HP.red_1_robot_HP.value << std::endl;
                        std::cout << "红2工程机器人血量: " << (int)game_robot_HP.red_2_robot_HP.value << std::endl;
                        std::cout << "红3步兵机器人血量: " << (int)game_robot_HP.red_3_robot_HP.value << std::endl;
                        std::cout << "红4步兵机器人血量: " << (int)game_robot_HP.red_4_robot_HP.value << std::endl;
                        std::cout << "保留位: " << (int)game_robot_HP.red_reserved.value << std::endl;
                        std::cout << "红7哨兵机器人血量: " << (int)game_robot_HP.red_7_robot_HP.value << std::endl;
                        std::cout << "红方前哨站血量: " << (int)game_robot_HP.red_outpost_HP.value << std::endl;
                        std::cout << "红方基地血量: " << (int)game_robot_HP.red_base_HP.value << std::endl;

                        std::cout << "蓝1英雄机器人血量: " << std::dec << (int)game_robot_HP.blue_1_robot_HP.value << std::endl;
                        std::cout << "蓝2工程机器人血量: " << (int)game_robot_HP.blue_2_robot_HP.value << std::endl;
                        std::cout << "蓝3步兵机器人血量: " << (int)game_robot_HP.blue_3_robot_HP.value << std::endl;
                        std::cout << "蓝4步兵机器人血量: " << (int)game_robot_HP.blue_4_robot_HP.value << std::endl;
                        std::cout << "保留位: " << (int)game_robot_HP.blue_reserved.value << std::endl;
                        std::cout << "蓝7哨兵机器人血量: " << (int)game_robot_HP.blue_7_robot_HP.value << std::endl;
                        std::cout << "蓝方前哨站血量: " << (int)game_robot_HP.blue_outpost_HP.value << std::endl;
                        std::cout << "蓝方基地血量: " << (int)game_robot_HP.blue_base_HP.value << std::endl;
                        break;
                    case CMD_ID_EVENT_DATA:
                        event_data_t event_data;
                        event_data.raw_data[0] = buffer[7];
                        event_data.raw_data[1] = buffer[8];
                        event_data.raw_data[2] = buffer[9];
                        event_data.raw_data[3] = buffer[10];

                        std::cout << "己方与兑换区不重叠的补给区占领状态: " << std::dec << (int)(event_data.value & 0b1) << std::endl;
                        std::cout << "己方与兑换区重叠的补给区占领状态: " << (int)((event_data.value & (0b1 << 0b1)) >> 1) << std::endl;
                        std::cout << "己方补给区的占领状态: " << (int)((event_data.value & (0b1 << 2)) >> 2) << std::endl;
                        std::cout << "己方小能量机关的激活状态: " << (int)((event_data.value & (0b1 << 3)) >> 3) << std::endl;
                        std::cout << "己方大能量机关的激活状态: " << (int)((event_data.value & (0b1 << 4)) >> 4) << std::endl;
                        std::cout << "己方中央高地的占领状态: " << (int)((event_data.value & (0b11 << 5)) >> 5) << std::endl;
                        std::cout << "己方梯形高地的占领状态: " << (int)((event_data.value & (0b11 << 7)) >> 7) << std::endl;
                        std::cout << "对方飞镖最后一次击中己方前哨站或基地的时间: " << (int)((event_data.value & (0b111111111 << 9)) >> 9) << std::endl;
                        std::cout << "对方飞镖最后一次击中己方前哨站或基地的具体目标: " << (int)((event_data.value & (0b111 << 18)) >> 18) << std::endl;
                        std::cout << "中心增益点的占领状态: " << (int)((event_data.value & (0b11 << 21)) >> 21) << std::endl;
                        std::cout << "保留位: " << (int)((event_data.value & (0b111111111 << 23)) >> 23) << std::endl;
                        break;
                    case CMD_ID_REFEREE_WARNING:
                        referee_warning_t referee_warning;
                        referee_warning.level = buffer[7];
                        referee_warning.offending_robot_id = buffer[8];
                        referee_warning.count = buffer[9];

                        std::cout << "己方最后一次受到判罚的等级: " << std::dec << (int)referee_warning.level << std::endl;
                        std::cout << "己方最后一次受到判罚的违规机器人ID: " << (int)referee_warning.offending_robot_id << std::endl;
                        std::cout << "己方最后一次受到判罚的违规机器人对应判罚等级的违规次数: " << (int)referee_warning.count << std::endl;
                        break;
                    case CMD_ID_ROBOT_STATUS:
                        robot_status_t robot_status;
                        robot_status.robot_id = buffer[7];
                        robot_status.robot_level = buffer[8];
                        robot_status.current_HP.raw_data[0] = buffer[9];
                        robot_status.current_HP.raw_data[1] = buffer[10];
                        robot_status.maximum_HP.raw_data[0] = buffer[11];
                        robot_status.maximum_HP.raw_data[1] = buffer[12];
                        robot_status.shooter_barrel_cooling_value.raw_data[0] = buffer[13];
                        robot_status.shooter_barrel_cooling_value.raw_data[1] = buffer[14];
                        robot_status.shooter_barrel_heat_limit.raw_data[0] = buffer[15];
                        robot_status.shooter_barrel_heat_limit.raw_data[1] = buffer[16];
                        robot_status.chassis_power_limit.raw_data[0] = buffer[17];
                        robot_status.chassis_power_limit.raw_data[1] = buffer[18];
                        robot_status.power_management_output.raw_data = buffer[19];
                        std::cout << "机器人ID: " << std::dec << (int)robot_status.robot_id << std::endl;
                        std::cout << "机器人等级: " << (int)robot_status.robot_level << std::endl;
                        std::cout << "当前HP: " << (int)robot_status.current_HP.value << std::endl;
                        std::cout << "最大HP: " << (int)robot_status.maximum_HP.value << std::endl;
                        std::cout << "枪管冷却值: " << (int)robot_status.shooter_barrel_cooling_value.value << std::endl;
                        std::cout << "枪管热量限制: " << (int)robot_status.shooter_barrel_heat_limit.value << std::endl;
                        std::cout << "底盘功率限制: " << (int)robot_status.chassis_power_limit.value << std::endl;
                        std::cout << "云台功率输出: " << (int)(robot_status.power_management_output.raw_data & 0x01) << std::endl;
                        std::cout << "底盘功率输出: " << (int)((robot_status.power_management_output.raw_data & 0x02) >> 1) << std::endl;
                        std::cout << "射击器功率输出: " << (int)((robot_status.power_management_output.raw_data & 0x04) >> 2) << std::endl;
                        break;
                    case CMD_ID_SHOOT_DATA:
                        shoot_data_t shoot_data;
                        shoot_data.bullet_type = buffer[7];
                        shoot_data.shooter_number = buffer[8];
                        shoot_data.launching_frequency = buffer[9];
                        shoot_data.initial_speed.raw_data[0] = buffer[10];
                        shoot_data.initial_speed.raw_data[1] = buffer[11];
                        shoot_data.initial_speed.raw_data[2] = buffer[12];
                        shoot_data.initial_speed.raw_data[3] = buffer[13];
                        std::cout << "弹丸类型: " << std::dec << (int)shoot_data.bullet_type << std::endl;
                        std::cout << "发射机构 ID: " << (int)shoot_data.shooter_number << std::endl;
                        std::cout << "弹丸射速: " << (int)shoot_data.launching_frequency << std::endl;
                        std::cout << "弹丸初速度: " << shoot_data.initial_speed.value << std::endl;
                        break;
                    default:
                        break;
                    }

                    buffer.clear();
                }
            }
        }
        // tcflush(fd, TCIFLUSH);
    }
}

int main()
{
    const char *serial_port = "/dev/ttyUSB0";
    int fd = initSerial(serial_port);
    if (fd == -1)
        return -1;

    receiveData(fd);
    close(fd);
    return 0;
}
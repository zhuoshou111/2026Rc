#include "colorreco.h"

uint8_t color_value[3] = {0, 0, 0}; // 存储颜色值
int color = 0;

void I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  // 启用 GPIOC 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  // 启用 GPIOD 时钟


    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;          // 开漏输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;        // 速度设置
    GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;          // 开漏输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;        // 速度设置
    GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}



void I2C_Delay(void)
{
    for (volatile int i = 0; i < 100; i++);
}

void I2C_Start(void)
{
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_Delay();
    I2C_SDA_LOW();  // SDA 拉低，开始信号
    I2C_Delay();
    I2C_SCL_LOW();  // SCL 拉低
}

void I2C_Stop(void)
{
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_Delay();
    I2C_SDA_HIGH();  // SDA 拉高，停止信号
    I2C_Delay();
}

void I2C_SendBit(uint8_t bit)
{
    if (bit)
        I2C_SDA_HIGH();
    else
        I2C_SDA_LOW();
    
    I2C_SCL_HIGH();
    I2C_Delay();
    I2C_SCL_LOW();
    I2C_Delay();
}

uint8_t I2C_ReadBit(void)
{
    uint8_t bit;
    I2C_SDA_HIGH();  // 释放 SDA 以便读取数据
    I2C_SCL_HIGH();
    I2C_Delay();
    bit = I2C_SDA_READ();
    I2C_SCL_LOW();
    I2C_Delay();
    return bit;
}

void I2C_SendByte(uint8_t byte)
{
    for (int i = 0; i < 8; i++)
    {
        I2C_SendBit((byte & 0x80) != 0);  // 发送最高位
        byte <<= 1;
    }
    // 接收应答位
    I2C_ReadBit();
}

uint8_t I2C_ReadByte(uint8_t ack)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        byte <<= 1;
        byte |= I2C_ReadBit();  // 读取每一位
    }
    // 发送应答位
    I2C_SendBit(ack ? 0 : 1);  // 发送 ACK 或 NACK
    return byte;
}



void I2C_Read_Sensor(uint8_t *recv_value)
{
    I2C_Start();  // 发送起始信号
                              //                                默认 跳线帽设置  读写位
    I2C_SendByte(0x4F << 1);  // 发送从机地址（写模式） 从机地址为 1001 111        X      0x48为 0100 1111
    I2C_SendByte(0xD1);       // 发送命令 0xD1,读取HSL数据的三个通道值，0xD0是读取RGB数据
    I2C_Stop();               // 停止信号

    I2C_Delay();  // 等待一小段时间

    I2C_Start();  // 重新启动，进入读模式
    I2C_SendByte((0x4F << 1) | 1);  // 发送从机地址（读模式）

    // 接收数据
    recv_value[0] = I2C_ReadByte(1);  // 读第1个字节，发送 ACK
    recv_value[1] = I2C_ReadByte(1);  // 读第2个字节，发送 ACK
    recv_value[2] = I2C_ReadByte(0);  // 读第3个字节，发送 NACK

    I2C_Stop();  // 停止信号
}

#include <stdint.h>
#include <math.h>

/**
 * @brief 根据输入的HSL值（0-240范围）返回最接近的颜色
 * @param hsl 输入的HSL数组，hsl[0]=H, hsl[1]=S, hsl[2]=L
 * @return 1（红）、2（绿）、3（蓝）、4（白）、5（黑）
 */
int getClosestColor(uint8_t hsl[3]) 
{

    float h = hsl[0] / 240.0f * 360.0f;  // H标准化到 [0, 360)
    float s = hsl[1] / 240.0f;           // S标准化到 [0, 1]
    float l = hsl[2] / 240.0f;           // L标准化到 [0, 1]

    // 1. 检查黑色（低亮度）
    if (l <= 0.1f) {
        return 5; // 黑色
    }

    // 2. 检查白色（高亮度 + 低饱和度）
    if (l >= 0.9f && s <= 0.1f) {
        return 4; // 白色
    }

    // 3. 计算当前H与红、绿、蓝的色相差
    float hue = h;
    if (hue < 0) hue += 360;
    if (hue >= 360) hue -= 360;

    // 标准颜色的H值
    float red_hue = 0.0f;    // 红色H=0°
    float green_hue = 120.0f; // 绿色H=120°
    float blue_hue = 240.0f;  // 蓝色H=240°

    // 计算当前H与红、绿、蓝的最小色相差
    float diff_red = fminf(fabsf(hue - red_hue), 360 - fabsf(hue - red_hue));
    float diff_green = fminf(fabsf(hue - green_hue), 360 - fabsf(hue - green_hue));
    float diff_blue = fminf(fabsf(hue - blue_hue), 360 - fabsf(hue - blue_hue));

    // 如果最接近的标准颜色偏差 ≤30°，则直接返回
    if (diff_red <= 30.0f) return 1;   // 红
    if (diff_green <= 30.0f) return 2; // 绿
    if (diff_blue <= 30.0f) return 3;  // 蓝

    // 4. 如果饱和度较高，选择最接近的RGB颜色
    if (s >= 0.5f) {
        if (diff_red <= diff_green && diff_red <= diff_blue) return 1; // 红
        if (diff_green <= diff_red && diff_green <= diff_blue) return 2; // 绿
        return 3; // 蓝
    }

    // 5. 低饱和度情况：根据亮度返回白或黑
    if (l >= 0.5f) {
        return 4; // 白
    } else {
        return 5; // 黑
    }
}

// /**
//  * @brief 根据输入的HSL值（0-240范围）返回最接近的预定义颜色
//  * @param hsl 输入的HSL数组，hsl[0]=H, hsl[1]=S, hsl[2]=L
//  * @return 1（红）、2（绿）、3（蓝）、4（白）、5（黑）
//  */
// int getClosestColor(uint8_t hsl[3]) {
//     // 预定义的模板颜色（需实际测量）
//     const uint8_t COLOR_TEMPLATES[5][3] = {
//         {0,   240, 120},  // 红
//         {80,  200, 100},  // 绿
//         {160, 200, 100},  // 蓝
//         {0,   0,   240},  // 白
//         {0,   0,   0}     // 黑
//     };

//     float min_diff = INFINITY;
//     int closest_color = 1; // 默认红色

//     // 遍历所有模板颜色，计算差异
//     for (int i = 0; i < 5; i++) {
//         // 计算色相差（注意环形处理）
//         float dh = fminf(
//             fabsf(hsl[0] - COLOR_TEMPLATES[i][0]),
//             240 - fabsf(hsl[0] - COLOR_TEMPLATES[i][0])
//         );
//         // 饱和度和亮度差
//         float ds = fabsf(hsl[1] - COLOR_TEMPLATES[i][1]);
//         float dl = fabsf(hsl[2] - COLOR_TEMPLATES[i][2]);

//         // 加权计算总差异（色相权重更高）
//         float diff = dh * 0.6f + ds * 0.2f + dl * 0.2f;

//         // 更新最接近的颜色
//         if (diff < min_diff) {
//             min_diff = diff;
//             closest_color = i + 1; // 1~5对应红~黑
//         }
//     }

//     return closest_color;
// }


void color_align(void)  //颜色校准函数，打印五种颜色对应的hsl值
{
    







}






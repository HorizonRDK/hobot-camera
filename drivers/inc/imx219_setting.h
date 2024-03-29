/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define rpi_config
#ifndef UTILITY_SENSOR_INC_IMX219_SETTING_H_
#define UTILITY_SENSOR_INC_IMX219_SETTING_H_

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef rpi_config
    static uint32_t imx219_2464p_init_setting[] = {
        0x30EB, 0x05,
        0x30EB, 0x0C,
        0x300A, 0xFF,
        0x300B, 0xFF,
        0x30EB, 0x05,
        0x30EB, 0x09,
        0x0114, 0x01,
        0x0128, 0x00,
        0x012A, 0x18,
        0x012B, 0x00,
        0x0160, 0x09,
        0x0161, 0xC8,
        0x0162, 0x0D,
        0x0163, 0x78,
        0x0164, 0x00,
        0x0165, 0x00,
        0x0166, 0x0C,
        0x0167, 0xCF,
        0x0168, 0x00,
        0x0169, 0x00,
        0x016A, 0x09,
        0x016B, 0x9F,
        0x016C, 0x0C,
        0x016D, 0xC0,
        0x016E, 0x09,
        0x016F, 0xA0,
        0x0170, 0x01,
        0x0171, 0x01,
        0x0174, 0x00,
        0x0175, 0x00,
        0x018C, 0x0A,
        0x018D, 0x0A,
        0x0301, 0x05,
        0x0303, 0x01,
        0x0304, 0x03,
        0x0305, 0x03,
        0x0306, 0x00,
        0x0307, 0x2B,
        0x0309, 0x0A,
        0x030B, 0x01,
        0x030C, 0x00,
        0x030D, 0x56,
        0x455E, 0x00,
        0x471E, 0x4B,
        0x4767, 0x0F,
        0x4750, 0x14,
        0x4540, 0x00,
        0x47B4, 0x14,
        0x4713, 0x30,
        0x478B, 0x10,
        0x478F, 0x10,
        0x4793, 0x10,
        0x4797, 0x0E,
        0x479B, 0x0E};

    // static uint32_t imx219_480p_init_setting[] = {
    //     0x0100, 0x00,
    //     0x30eb, 0x05,
    //     0x30eb, 0x0c,
    //     0x300a, 0xff,
    //     0x300b, 0xff,
    //     0x30eb, 0x05,
    //     0x30eb, 0x09,
    //     0x0114, 0x01,
    //     0x0128, 0x00,
    //     0x012a, 0x18,
    //     0x012b, 0x00,
    //     0x0162, 0x0d,
    //     0x0163, 0x78,
    //     0x0164, 0x03,
    //     0x0165, 0xe8,
    //     0x0166, 0x08,
    //     0x0167, 0xe7,
    //     0x0168, 0x02,
    //     0x0169, 0xf0,
    //     0x016a, 0x06,
    //     0x016b, 0xaf,
    //     0x016c, 0x02,
    //     0x016d, 0x80,
    //     0x016e, 0x01,
    //     0x016f, 0xe0,
    //     0x0170, 0x01,
    //     0x0171, 0x01,
    //     0x0174, 0x03,
    //     0x0175, 0x03,
    //     0x0301, 0x05,
    //     0x0303, 0x01,
    //     0x0304, 0x03,
    //     0x0305, 0x03,
    //     0x0306, 0x00,
    //     0x0307, 0x39,
    //     0x030b, 0x01,
    //     0x030c, 0x00,
    //     0x030d, 0x72,
    //     0x0624, 0x06,
    //     0x0625, 0x68,
    //     0x0626, 0x04,
    //     0x0627, 0xd0,
    //     0x455e, 0x00,
    //     0x471e, 0x4b,
    //     0x4767, 0x0f,
    //     0x4750, 0x14,
    //     0x4540, 0x00,
    //     0x47b4, 0x14,
    //     0x4713, 0x30,
    //     0x478b, 0x10,
    //     0x478f, 0x10,
    //     0x4793, 0x10,
    //     0x4797, 0x0e,
    //     0x479b, 0x0e

    // };
    static uint32_t imx219_480p_init_setting[] = {
        0x0103, 0x01,
        0x0100, 0x00,
        0x30EB, 0x05,
        0x30EB, 0x0C,
        0x300A, 0xFF,
        0x300B, 0xFF,
        0x30EB, 0x05,
        0x30EB, 0x09,
        0x0114, 0x01,
        0x0128, 0x00,
        0x012A, 0x18,
        0x012B, 0x00,
        0x0157, 0x00,
        0x0160, 0x06,
        0x0161, 0xA0,
        0x0162, 0x0D,
        0x0163, 0xE8,
        0x0164, 0x03,
        0x0165, 0xE8,
        0x0166, 0x08,
        0x0167, 0xE7,
        0x0168, 0x02,
        0x0169, 0xF0,
        0x016A, 0x06,
        0x016B, 0xAF,
        0x016C, 0x02,
        0x016D, 0x80,
        0x016E, 0x01,
        0x016F, 0xE0,
        0x0170, 0x01,
        0x0171, 0x01,
        0x0174, 0x03,
        0x0175, 0x03,
        0x018C, 0x0A,
        0x018D, 0x0A,
        0x0301, 0x05,
        0x0303, 0x01,
        0x0304, 0x03,
        0x0305, 0x03,
        0x0306, 0x00,
        0x0307, 0x39,
        0x0309, 0x0A,
        0x030B, 0x01,
        0x030C, 0x00,
        0x030D, 0x72,
        0x455E, 0x00,
        0x471E, 0x4B,
        0x4767, 0x0F,
        0x4750, 0x14,
        0x4540, 0x00,
        0x47B4, 0x14,
        0x4713, 0x30,
        0x478B, 0x10,
        0x478F, 0x10,
        0x4793, 0x10,
        0x4797, 0x0E,
        0x479B, 0x0E};
    static uint32_t imx219_1080_init_setting[] = {
        0x0100, 0x00,
        0x30eb, 0x05,
        0x30eb, 0x0c,
        0x300a, 0xff,
        0x300b, 0xff,
        0x30eb, 0x05,
        0x30eb, 0x09,
        0x0114, 0x01,
        0x0128, 0x00,
        0x012a, 0x18,
        0x012b, 0x00,
        0x0160, 0x06,
        0x0161, 0xCD, // frame lenth
        0x0162, 0x0D,
        0x0163, 0x78, // line lenth
        0x0164, 0x02,
        0x0165, 0xa8,
        0x0166, 0x0a,
        0x0167, 0x27,
        0x0168, 0x02,
        0x0169, 0xb4,
        0x016a, 0x06,
        0x016b, 0xeb,
        0x016c, 0x07,
        0x016d, 0x80,
        0x016e, 0x04,
        0x016f, 0x38,
        0x0170, 0x01,
        0x0171, 0x01,
        0x0174, 0x00,
        0x0175, 0x00,
        0x0301, 0x05,
        0x0303, 0x01,
        0x0304, 0x03,
        0x0305, 0x03,
        0x0306, 0x00,
        0x0307, 0x39,
        0x030b, 0x01,
        0x030c, 0x00,
        0x030d, 0x72,
        0x0624, 0x07,
        0x0625, 0x80,
        0x0626, 0x04,
        0x0627, 0x38,
        0x455e, 0x00,
        0x471e, 0x4b,
        0x4767, 0x0f,
        0x4750, 0x14,
        0x4540, 0x00,
        0x47b4, 0x14,
        0x4713, 0x30,
        0x478b, 0x10,
        0x478f, 0x10,
        0x4793, 0x10,
        0x4797, 0x0e,
        0x479b, 0x0e};
    static uint32_t imx219_1232p_init_setting[] = {
        0x30EB, 0x05,
        0x30EB, 0x0C,
        0x300A, 0xFF,
        0x300B, 0xFF,
        0x30EB, 0x05,
        0x30EB, 0x09,
        0x0114, 0x01,
        0x0128, 0x00,
        0x012A, 0x18,
        0x012B, 0x00,
        0x0160, 0x05,
        0x0161, 0x34,
        0x0162, 0x0D,
        0x0163, 0x78,
        0x0164, 0x00,
        0x0165, 0x00,
        0x0166, 0x0C,
        0x0167, 0xCF,
        0x0168, 0x00,
        0x0169, 0x00,
        0x016A, 0x09,
        0x016B, 0x9F,
        0x016C, 0x06,
        0x016D, 0x60,
        0x016E, 0x04,
        0x016F, 0xD0,
        0x0170, 0x01,
        0x0171, 0x01,
        0x0174, 0x01,
        0x0175, 0x01,
        0x018C, 0x0A,
        0x018D, 0x0A,
        0x0301, 0x05,
        0x0303, 0x01,
        0x0304, 0x03,
        0x0305, 0x03,
        0x0306, 0x00,
        0x0307, 0x2B,
        0x0309, 0x0A,
        0x030B, 0x01,
        0x030C, 0x00,
        0x030D, 0x56,
        0x455E, 0x00,
        0x471E, 0x4B,
        0x4767, 0x0F,
        0x4750, 0x14,
        0x4540, 0x00,
        0x47B4, 0x14,
        0x4713, 0x30,
        0x478B, 0x10,
        0x478F, 0x10,
        0x4793, 0x10,
        0x4797, 0x0E,
        0x479B, 0x0E};
#endif
    static uint32_t imx219_gain_lut[] = {
        0x0,
        0x5,
        0xB,
        0xF,
        0x15,
        0x1A,
        0x1F,
        0x24,
        0x28,
        0x2D,
        0x32,
        0x35,
        0x3A,
        0x3F,
        0x43,
        0x47,
        0x4B,
        0x4E,
        0x52,
        0x56,
        0x5A,
        0x5E,
        0x61,
        0x64,
        0x68,
        0x6B,
        0x6E,
        0x71,
        0x74,
        0x77,
        0x7A,
        0x7D,
        0x80,
        0x83,
        0x86,
        0x89,
        0x8B,
        0x8D,
        0x90,
        0x92,
        0x94,
        0x97,
        0x99,
        0x9B,
        0x9D,
        0x9F,
        0xA2,
        0xA4,
        0xA6,
        0xA7,
        0xA9,
        0xAB,
        0xAD,
        0xAF,
        0xB1,
        0xB2,
        0xB4,
        0xB6,
        0xB7,
        0xB9,
        0xBA,
        0xBC,
        0xBD,
        0xBF,
        0xC0,
        0xC1,
        0xC3,
        0xC4,
        0xC5,
        0xC6,
        0xC8,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
        0xD0,
        0xD0,
        0xD2,
        0xD3,
        0xD4,
        0xD5,
        0xD6,
        0xD7,
        0xD7,
        0xD8,
        0xD9,
        0xDA,
        0xDB,
        0xDC,
        0xDD,
        0xDD,
        0xDE,
        0xDF,
        0xDF,
        0xE0,
        0xE1,
        0xE1,
        0xE2,
        0xE3,
        0xE4,
        0xE4,
        0xE5,
        0xE5,
        0xE6,
        0xE6,
        0xE7,
        0xE7,
        0xE8};
    // test
    static uint32_t imx219_stream_on_setting[] = {0x0100, 0x01};
    // test
    static uint32_t imx219_stream_off_setting[] = {
        0x0100, 0x00};

#ifdef __cplusplus
}
#endif

#endif

#include <stdio.h>
#include <gpiod.h>
#include "../inc/gpio.h"
#define RS485_GPIO_CHIP "/dev/gpiochip2"
#define RS485_GPIO_LINE 13
#define BD_EN_CHIP "/dev/gpiochip2"
#define BD_EN_LINE 22
#define BD_POW_CHIP "/dev/gpiochip3"
#define BD_POW_LINE 17
#define BT_POW_CHIP "/dev/gpiochip0"
#define BT_POW_LINE 29
#define EG_POW_CHIP "/dev/gpiochip3"
#define EG_POW_LINE 19

struct gpiod_chip *chip_rs485 = NULL;
struct gpiod_line *line_rs485 = NULL;
struct gpiod_chip *chip_bd_en = NULL;
struct gpiod_line *line_bd_en = NULL;
// struct gpiod_chip *chip_bd_pow = NULL;
// struct gpiod_line *line_bd_pow = NULL;
struct gpiod_chip *chip_bt_pow = NULL;
struct gpiod_line *line_bt_pow = NULL;
struct gpiod_chip *chip_4g_pow = NULL;
struct gpiod_line *line_4g_pow = NULL;
int gpio_init()
{
    chip_rs485 = gpiod_chip_open(RS485_GPIO_CHIP);
    if (!chip_rs485)
    {
        perror("gpiod_chip_open485");
        return -1;
    }
    line_rs485 = gpiod_chip_get_line(chip_rs485, RS485_GPIO_LINE);
    if (!line_rs485)
    {
        perror("gpiod_chip_get_line 485");
        gpiod_chip_close(chip_rs485);
        return -1;
    }
    // 请求为输出，初始值为0（接收模式）
    if (gpiod_line_request_output(line_rs485, "rs485", 0) < 0)
    {
        perror("gpiod_line_request_output485");
        gpiod_chip_close(chip_rs485);
        return -1;
    }

    chip_bd_en = gpiod_chip_open(BD_EN_CHIP);
    if (!chip_bd_en)
    {
        perror("gpiod_chip_open bd_en");
        return -1;
    }
    line_bd_en = gpiod_chip_get_line(chip_bd_en, BD_EN_LINE);
    if (!line_bd_en)
    {
        perror("gpiod_chip_get_line bd_en");
        gpiod_chip_close(chip_bd_en);
        return -1;
    }
    // 请求为输出，初始值为0
    if (gpiod_line_request_output(line_bd_en, "bd_en", 0) < 0)
    {
        perror("gpiod_line_request_output bd_en");
        gpiod_chip_close(chip_bd_en);
        return -1;
    }

    // chip_bd_pow = gpiod_chip_open(BD_POW_CHIP);
    // line_bd_pow = gpiod_chip_get_line(chip_bd_pow, BD_POW_LINE);
    // // 请求为输出，初始值为0
    // if (gpiod_line_request_output(line_bd_pow, "bd_pow_en", 0) < 0)
    // {
    //     perror("gpiod_line_request_output bd_pow_en");
    //     gpiod_chip_close(chip_bd_pow);
    //     return -1;
    // }

    chip_bt_pow = gpiod_chip_open(BT_POW_CHIP);
    line_bt_pow = gpiod_chip_get_line(chip_bt_pow, BT_POW_LINE);
    // 请求为输出，初始值为1
    if (gpiod_line_request_output(line_bt_pow, "bt_pow_en", 1) < 0)
    {
        perror("gpiod_line_request_output bt_pow_en");
        gpiod_chip_close(chip_bt_pow);
        return -1;
    }

    chip_4g_pow = gpiod_chip_open(EG_POW_CHIP);
    line_4g_pow = gpiod_chip_get_line(chip_4g_pow, EG_POW_LINE);
    // 请求为输出，初始值为1
    if (gpiod_line_request_output(line_4g_pow, "4g_pow_en", 1) < 0)
    {
        perror("gpiod_line_request_output 4g_pow_en");
        gpiod_chip_close(chip_4g_pow);
        return -1;
    }
    return 0;
}

void gpio_set_value(int value, struct gpiod_line *line)
{
    if (line)
        gpiod_line_set_value(line, value);
}

int gpio_get_value(struct gpiod_line *line)
{
    if (line)
        return gpiod_line_get_value(line);
    return -1;
}

void gpio_cleanup(void)
{
    if (chip_rs485)
        gpiod_chip_close(chip_rs485);
    if (chip_bd_en)
        gpiod_chip_close(chip_bd_en);
}
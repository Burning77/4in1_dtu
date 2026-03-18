#ifndef GPIO_H
#define GPIO_H

// 初始化GPIO芯片和引脚，设置为输出，初始为低电平（接收模式）
// 参数：chip_path - GPIO芯片路径（如"/dev/gpiochip2"），line_num - 引脚编号
// 返回：0成功，-1失败
int gpio_init(void);

// 设置引脚输出电平（1=高，0=低）
void gpio_set_value(int value,struct gpiod_line *line);

// 读取引脚当前电平
int gpio_get_value(struct gpiod_line *line);

// 释放GPIO资源（关闭芯片和线路）
void gpio_cleanup(void);

#endif
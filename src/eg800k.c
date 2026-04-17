#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdatomic.h>
#include "../inc/universal.h"
#include "../inc/eg800k.h"
#include "../inc/usart.h"

static int eg_connect_id = 0; // Socket 连接 ID
static int eg_context_id = 1; // PDP 场景 ID
extern int eg_fd;
extern atomic_int stop_flag;

// 发送 AT 命令并等待指定响应（带超时，可被 stop_flag 中断）
int eg_send_cmd(const char *cmd, const char *expected_resp, int timeout_sec)
{
    char buf[512];
    data_send((unsigned char *)cmd, strlen(cmd), EG_DEV);

    fd_set fds;
    struct timeval tv;
    int ret;
    int elapsed = 0;

    // 使用 1 秒超时循环，检查 stop_flag
    while (elapsed < timeout_sec && !stop_flag)
    {
        FD_ZERO(&fds);
        FD_SET(eg_fd, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
        if (ret > 0)
        {
            int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
            if (n <= 0)
                return -1;
            buf[n] = '\0';

            if (expected_resp && strstr(buf, expected_resp) == NULL)
            {
                printf("[EG] Unexpected response: %s\n", buf);
                return -1;
            }
            return 0;
        }
        elapsed++;
    }

    if (stop_flag)
    {
        printf("[EG] Command interrupted by stop_flag\n");
        return -1;
    }

    printf("[EG] Timeout waiting for response to: %s\n", cmd);
    return -1;
}

// 初始化 4G 模块（仅执行一次）
int eg_init(void)
{
    printf("[EG] Starting initialization...\n");

    // 1. AT 同步
    if (eg_send_cmd("AT\r\n", "OK", 2) != 0)
        return -1;

    // 2. 查询 SIM 卡
    if (eg_send_cmd("AT+CPIN?\r\n", "READY", 5) != 0)
    {
        printf("[EG] SIM card not ready\n");
        return -1;
    }

    // 3. 查询信号质量（仅打印，不强制要求）
    eg_send_cmd("AT+CSQ\r\n", NULL, 2);

    // 4. 查询网络注册（LTE）
    if (eg_send_cmd("AT+CEREG?\r\n", "1", 10) != 0)
    {
        printf("[EG] Network registration failed\n");
        return -1;
    }

    // 5. 配置 APN（请根据 SIM 卡运营商修改）
    // 中国联通: "UNINET", 中国移动: "CMNET", 中国电信: "CTNET"
    if (eg_send_cmd("AT+QICSGP=1,1,\"UNINET\",\"\",\"\",1\r\n", "OK", 5) != 0)
    {
        printf("[EG] APN configuration failed\n");
        return -1;
    }

    // 6. 激活 PDP 场景（最长 150 秒）
    if (eg_send_cmd("AT+QIACT=1\r\n", "OK", 150) != 0)
    {
        printf("[EG] PDP activation failed\n");
        return -1;
    }

    // 7. 查询 IP 地址（可选）
    eg_send_cmd("AT+QIACT?\r\n", NULL, 2);

    printf("[EG] Initialization complete\n");
    return 0;
}
// 发送数据（通过已建立的 TCP 连接）
int eg_send_data(const unsigned char *data, int len)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QISEND=%d,%d\r\n", eg_connect_id, len);

    // 发送命令，等待 '>' 提示符
    data_send((unsigned char *)cmd, strlen(cmd), EG_DEV);

    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(eg_fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
    {
        printf("[EG] No '>' prompt for QISEND\n");
        return -1;
    }

    char buf[16];
    int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
    if (n <= 0 || strstr(buf, ">") == NULL)
    {
        printf("[EG] Expected '>' not received\n");
        return -1;
    }

    // 发送原始数据
    data_send((unsigned char *)data, len, EG_DEV);
    // 发送 Ctrl+Z (0x1A) 结束
    unsigned char ctrl_z = 0x1A;
    data_send(&ctrl_z, 1, EG_DEV);

    // 等待 SEND OK
    FD_ZERO(&fds);
    FD_SET(eg_fd, &fds);
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
    {
        printf("[EG] SEND OK timeout\n");
        return -1;
    }

    n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
    if (n <= 0 || strstr(buf, "SEND OK") == NULL)
    {
        printf("[EG] Send failed: %s\n", buf);
        return -1;
    }

    printf("[EG] Sent %d bytes\n", len);
    return 0;
}

// 检查 4G 网络是否可用（信号正常 + 已注册）
int eg_is_network_available(void)
{
    char buf[64];
    int rssi, stat;

    // 1. 查询信号质量
    data_send((unsigned char *)"AT+CSQ\r\n", strlen("AT+CSQ\r\n"), EG_DEV);
    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(eg_fd, &fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
        return 0;
    int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
    if (sscanf(buf, "+CSQ: %d,%d", &rssi, &stat) != 2)
        return 0;
    if (rssi == 99)
        return 0; // 未知信号

    // 2. 查询 LTE 注册状态
    data_send((unsigned char *)"AT+CEREG?\r\n", strlen("AT+CEREG?\r\n"), EG_DEV);
    FD_ZERO(&fds);
    FD_SET(eg_fd, &fds);
    tv.tv_sec = 2;
    ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
        return 0;
    n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
    if (n <= 0)
        return 0;
    buf[n] = '\0';
    int n_val, reg_stat;
    if (sscanf(buf, "+CEREG: %d,%d", &n_val, &reg_stat) == 2)
    {
        if (reg_stat == 1 || reg_stat == 5)
            return 1; // 已注册或漫游
    }
    return 0;
}
int eg_connect(void)
{
    const char *tcp_cmd = "AT+QIOPEN=1,0,\"TCP\",\"220.180.239.212\",8009,0,0\r\n";
    char buf[256];

    data_send((unsigned char *)tcp_cmd, strlen(tcp_cmd), EG_DEV);

    // 等待 +QIOPEN: 0,0 （最长 150 秒，可被 stop_flag 中断）
    fd_set fds;
    struct timeval tv;
    int elapsed = 0;
    const int timeout_sec = 150;

    while (elapsed < timeout_sec && !stop_flag)
    {
        FD_ZERO(&fds);
        FD_SET(eg_fd, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int ret = select(eg_fd + 1, &fds, NULL, NULL, &tv);
        if (ret > 0)
        {
            int n = data_recv(buf, sizeof(buf) - 1, EG_DEV);
            if (n <= 0)
                return -1;
            buf[n] = '\0';

            if (strstr(buf, "+QIOPEN: 0,0") != NULL)
            {
                printf("[EG] TCP connected\n");
                return 0;
            }
            else
            {
                printf("[EG] TCP connect failed: %s\n", buf);
                return -1;
            }
        }
        elapsed++;
    }

    if (stop_flag)
    {
        printf("[EG] TCP connect interrupted by stop_flag\n");
        return -1;
    }

    printf("[EG] TCP connect timeout\n");
    return -1;
}

int eg_reinit_pdp(void)
{
    printf("[EG] Re-initializing PDP...\n");
    // 先去激活（清理本地状态）
    eg_send_cmd("AT+QIDEACT=1\r\n", "OK", 40);
    // 重新激活 PDP
    if (eg_send_cmd("AT+QIACT=1\r\n", "OK", 150) != 0)
    {
        printf("[EG] PDP reactivation failed\n");
        return -1;
    }
    // 重新建立 TCP 连接
    if (eg_connect() != 0)
    {
        printf("[EG] Reconnect after PDP reactivation failed\n");
        return -1;
    }
    printf("[EG] PDP re-initialization successful\n");
    return 0;
}
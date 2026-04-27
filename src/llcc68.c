#include "llcc68.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <gpiod.h>
#include <pthread.h>
#define FREQ_STEP 0.953674
#define MAX_FRAME_SIZE 256
// 全局句柄
int spi_fd = -1;
static struct gpiod_chip *gpio_chip = NULL;
static struct gpiod_line *line_nss = NULL;
static struct gpiod_line *line_reset = NULL;
static struct gpiod_line *line_busy = NULL;
static struct gpiod_line *line_dio1 = NULL;
static uint8_t regModeParam = 0x01; // 0: LDO, 1: DC-DC
static loRa_Para_t *lora_para_pt;
static uint8_t last_frame[MAX_FRAME_SIZE];
static int last_len = 0;
static uint8_t tx_buf[260];
static uint8_t rx_buf[260];
static pthread_mutex_t g_lora_lock = PTHREAD_MUTEX_INITIALIZER;
int hw_init(void)
{
	// 1. 初始化 SPI
	spi_fd = open(SPI_DEV_PATH, O_RDWR);
	if (spi_fd < 0)
	{
		perror("Failed to open SPI");
		return -1;
	}

	// 配置 SPI 模式 0, 8位, 速度 (LLCC68 最高 16MHz，这里设 10MHz 安全)
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint32_t speed = 10000000;

	ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

	// 2. 初始化 GPIO
	gpio_chip = gpiod_chip_open_by_name(GPIO_CHIP_NAME);
	if (!gpio_chip)
	{
		perror("Failed to open GPIO chip");
		close(spi_fd);
		return -1;
	}

	// 获取 Lines
	line_reset = gpiod_chip_get_line(gpio_chip, PIN_RESET);
	line_busy = gpiod_chip_get_line(gpio_chip, PIN_BUSY);
	line_dio1 = gpiod_chip_get_line(gpio_chip, PIN_DIO1);

	// 配置方向
	gpiod_line_request_output(line_reset, "llcc68", 1);
	gpiod_line_request_input(line_busy, "llcc68");
	gpiod_line_request_input(line_dio1, "llcc68");

	return 0;
}
void spi_init()
{
	// 1. 打开SPI设备
	int fd = open(SPI_DEV_PATH, O_RDWR);
	if (fd < 0)
	{
		perror("无法打开SPI设备");
		return 1;
	}

	// 2. 配置SPI参数
	uint8_t mode = SPI_MODE_0; // CPOL=0, CPHA=0
	uint8_t bits = 8;		   // 8位数据
	uint32_t speed = 1000000;  // 1MHz（可根据需要调整）

	ioctl(fd, SPI_IOC_WR_MODE, &mode);
	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}
void reset_llcc68()
{
	gpiod_line_set_value(line_reset, 0); // 拉低复位
	usleep(2 * 1000);
	gpiod_line_set_value(line_reset, 1);
	usleep(20 * 1000);
}
// 核心 SPI 传输函数 (对应 Arduino 的 SPI.transfer)
void spi_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx_buf,
		.rx_buf = (unsigned long)rx_buf,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = 10000000,
		.bits_per_word = 8,
	};
	ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}
void SetStandby(uint8_t StdbyConfig)
{
	uint8_t Opcode;

	CheckBusy();
	Opcode = SET_STANDBY; // 0x80

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&StdbyConfig, NULL, 1);
}
void CheckBusy(void)
{
	uint8_t busy_timeout_cnt;

	busy_timeout_cnt = 0;
	while (gpiod_line_get_value(line_busy)) // 忙信号为高电平
	{
		usleep(1 * 1000);
		busy_timeout_cnt++;

		if (busy_timeout_cnt > 2) // TODO
		{
			SetStandby(0);	// 0:STDBY_RC; 1:STDBY_XOSC
			reset_llcc68(); // reset RF
			LLCC68_Config();
			break;
		}
	}
}
void SetRegulatorMode(void)
{
	uint8_t Opcode;

	CheckBusy();
	Opcode = 0x96;

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&regModeParam, NULL, 1); // regModeParam
}

void SetPaConfig(void)
{
	uint8_t Opcode;
	uint8_t paDutyCycle = 0x04; // paDutyCycle
	uint8_t hpMax = 0x07;		// hpMax:0x00~0x07; 7:22dbm
	uint8_t deviceSel = 0x00;	// deviceSel:0x00~0x01; 0:PA_BOOST; 1:RFO

	CheckBusy();
	Opcode = 0x95;

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&paDutyCycle, NULL, 1); // paDutyCycle
	spi_transfer(&hpMax, NULL, 1);		 // hpMax:0x00~0x07; 7:22dbm
	spi_transfer(&deviceSel, NULL, 1);	 // deviceSel
	spi_transfer(&regModeParam, NULL, 1);
}

void SetPacketType(uint8_t PacketType)
{
	uint8_t Opcode;

	CheckBusy();
	Opcode = SET_PACKET_TYPE; // 0x8A

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&PacketType, NULL, 1);
}
void SetRfFrequency(uint32_t frequency)
{
	uint8_t Opcode;
	uint8_t Rf_Freq[4];
	uint32_t RfFreq = 0;

	RfFreq = (uint32_t)((double)frequency / (double)FREQ_STEP);

	CheckBusy();

	Opcode = SET_RF_FREQUENCY; // 0x86

	Rf_Freq[0] = (RfFreq >> 24) & 0xFF; // MSB
	Rf_Freq[1] = (RfFreq >> 16) & 0xFF;
	Rf_Freq[2] = (RfFreq >> 8) & 0xFF;
	Rf_Freq[3] = RfFreq & 0xFF; // LSB

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&Rf_Freq, NULL, 4);
}
void SetTxParams(int8_t power, uint8_t RampTime)
{
	uint8_t Opcode;

	CheckBusy();
	Opcode = SET_TX_PARAMS; // 0x8E

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&power, NULL, 1);
	spi_transfer(&RampTime, NULL, 1);
}

void SetModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
	uint8_t Opcode;
	uint8_t reserve_bytes = 0xFF;
	CheckBusy();
	Opcode = 0x8B;

	spi_transfer(&Opcode, NULL, 1);

	spi_transfer(&sf, NULL, 1);	  // SF=5~12
	spi_transfer(&bw, NULL, 1);	  // BW
	spi_transfer(&cr, NULL, 1);	  // CR
	spi_transfer(&ldro, NULL, 1); // LDRO LowDataRateOptimize 0:OFF; 1:ON;

	spi_transfer(&reserve_bytes, NULL, 1); //
	spi_transfer(&reserve_bytes, NULL, 1); //
	spi_transfer(&reserve_bytes, NULL, 1); //
	spi_transfer(&reserve_bytes, NULL, 1); //
}

void SetPacketParams(uint8_t payload_len)
{
	uint8_t Opcode;
	uint16_t prea_len;
	uint8_t prea_len_h, prea_len_l;
	uint8_t crc_type = 0x01; // CRCType 0:OFF 1:ON
	uint8_t invertIQ = 0x00; // InvertIQ 0:Standard 1:Inverted
	uint8_t reserve_bytes = 0xFF;
	uint8_t header_type = 0x00; // HeaderType 0:Variable,explicit 1:Fixed,implicit
	CheckBusy();

	Opcode = 0x8C;

	prea_len = 8;
	prea_len_h = prea_len >> 8;
	prea_len_l = prea_len & 0xFF;

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&prea_len_h, NULL, 1);	 // PreambleLength MSB
	spi_transfer(&prea_len_l, NULL, 1);	 // PreambleLength LSB
	spi_transfer(&header_type, NULL, 1); // HeaderType 0:Variable,explicit 1:Fixed,implicit
	// spi_transfer(&0x01);
	spi_transfer(&payload_len, NULL, 1);   // PayloadLength: 0x00 to 0xFF
	spi_transfer(&crc_type, NULL, 1);	   // CRCType 0:OFF 1:ON
	spi_transfer(&invertIQ, NULL, 1);	   // InvertIQ 0:Standard
	spi_transfer(&reserve_bytes, NULL, 1); // reserve
	spi_transfer(&reserve_bytes, NULL, 1); // reserve
	spi_transfer(&reserve_bytes, NULL, 1); // reserve
}
void LLCC68_Config()
{
	uint32_t rf_freq_temp;
	int8_t power_temp;
	uint8_t sf_temp;
	uint8_t bw_temp;
	uint8_t cr_temp;
	uint8_t size_temp;

	rf_freq_temp = lora_para_pt->rf_freq;
	power_temp = lora_para_pt->tx_power;
	sf_temp = lora_para_pt->lora_sf;
	bw_temp = lora_para_pt->band_width;
	cr_temp = lora_para_pt->code_rate;
	size_temp = lora_para_pt->payload_size;

	SetStandby(0); // 0:STDBY_RC; 1:STDBY_XOSC
	SetRegulatorMode();
	SetPaConfig();

	SetPacketType(1);					   // 0:GFSK; 1:LORA
	SetRfFrequency(rf_freq_temp);		   // RF_Freq = freq_reg*32M/(2^25)
	SetTxParams(power_temp, SET_RAMP_10U); // set power and ramp_time

	SetModulationParams(sf_temp, bw_temp, cr_temp, LDRO_ON);

	SetPacketParams(size_temp); // PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
}

void SetSleep(void)
{
	uint8_t Opcode, sleepConfig;

	CheckBusy();
	Opcode = SET_SLEEP; // 0x84
	sleepConfig = 0x00; // 0x04;	//bit2: 1:warm start; bit0:0: RTC timeout disable
	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&sleepConfig, NULL, 1);
}
void SetBufferBaseAddress(uint8_t TX_base_addr, uint8_t RX_base_addr)
{
	uint8_t Opcode;

	CheckBusy();
	Opcode = SET_BUF_BASE_ADDR; // 0x8F

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&TX_base_addr, NULL, 1);
	spi_transfer(&RX_base_addr, NULL, 1);
}
void SetDioIrqParams(uint16_t irq)
{
	uint8_t Opcode;
	uint16_t Irq_Mask;
	uint8_t Irq_Mask_h, Irq_Mask_l;
	uint16_t DIO1Mask;
	uint8_t DIO1Mask_h, DIO1Mask_l;
	uint16_t DIO2Mask;
	uint8_t DIO2Mask_h, DIO2Mask_l;
	uint16_t DIO3Mask;
	uint8_t DIO3Mask_h, DIO3Mask_l;

	Irq_Mask = irq;
	DIO1Mask = irq;
	DIO2Mask = 0;
	DIO3Mask = 0;

	Irq_Mask_h = Irq_Mask >> 8;
	Irq_Mask_l = Irq_Mask & 0xFF;
	DIO1Mask_h = DIO1Mask >> 8;
	DIO1Mask_l = DIO1Mask & 0xFF;
	DIO2Mask_h = DIO2Mask >> 8;
	DIO2Mask_l = DIO2Mask & 0xFF;
	DIO3Mask_h = DIO3Mask >> 8;
	DIO3Mask_l = DIO3Mask & 0xFF;
	Opcode = 0x08;

	CheckBusy();

	spi_transfer(&Opcode, NULL, 1);

	spi_transfer(&Irq_Mask_h, NULL, 1); // Irq_Mask MSB
	spi_transfer(&Irq_Mask_l, NULL, 1); // Irq_Mask LSB
	spi_transfer(&DIO1Mask_h, NULL, 1); //
	spi_transfer(&DIO1Mask_l, NULL, 1); //

	spi_transfer(&DIO2Mask_h, NULL, 1); //
	spi_transfer(&DIO2Mask_l, NULL, 1); //
	spi_transfer(&DIO3Mask_h, NULL, 1); //
	spi_transfer(&DIO3Mask_l, NULL, 1); //
}
void SetRx(uint32_t timeout)
{
	uint8_t Opcode;
	uint8_t time_out[3];

	CheckBusy();

	Opcode = SET_RX;					  // 0x82
	time_out[0] = (timeout >> 16) & 0xFF; // MSB
	time_out[1] = (timeout >> 8) & 0xFF;
	time_out[2] = timeout & 0xFF; // LSB

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(&time_out[0], NULL, 1);
	spi_transfer(&time_out[1], NULL, 1);
	spi_transfer(&time_out[2], NULL, 1);
}
void RxInit(void)
{
	SetBufferBaseAddress(0, 0); //(TX_base_addr,RX_base_addr)
	// SetPacketParams(payload_length);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
	SetDioIrqParams(RxDone_IRQ); // RxDone IRQ

	SetRx(0); // timeout = 0
}
void SetTx(uint32_t timeout)
{
	uint8_t Opcode;
	uint8_t time_out[3];

	CheckBusy();
	Opcode = SET_TX;					  // 0x83
	time_out[0] = (timeout >> 16) & 0xFF; // MSB
	time_out[1] = (timeout >> 8) & 0xFF;
	time_out[2] = timeout & 0xFF; // LSB

	spi_transfer(&Opcode, NULL, 1);
	spi_transfer(time_out, NULL, 3);
}
bool Lora_init(loRa_Para_t *lp_pt)
{
	lora_para_pt = lp_pt;
	hw_init();
	spi_init();
	reset_llcc68();
	LLCC68_Config();
	return true;
}
uint16_t GetIrqStatus(void)
{
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];
	uint16_t irq_status = 0;

	CheckBusy();

	tx_buf[0] = 0x12; // 命令码
	tx_buf[1] = 0xff;
	tx_buf[2] = 0xff;
	tx_buf[3] = 0xff;

	spi_transfer(tx_buf, rx_buf, 4);
	irq_status = (rx_buf[1] << 8) | rx_buf[2];

	return irq_status;
}
void GetRxBufferStatus(uint8_t *payload_len, uint8_t *buf_pointer)
{
	uint8_t tx_buf[4]; // 发送缓冲区：1字节命令 + 3字节哑元数据
	uint8_t rx_buf[4]; // 接收缓冲区

	// 1. 检查设备是否繁忙
	CheckBusy();

	// 2. 构建发送数据
	tx_buf[0] = 0x13; // GetRxBufferStatus 命令码
	tx_buf[1] = 0xFF;
	tx_buf[2] = 0xFF;
	tx_buf[3] = 0xFF;
	spi_transfer(tx_buf, rx_buf, 4);
	// 字节0: 状态 (Status)
	// 字节1: 有效负载长度 (payload_len)
	// 字节2: 缓冲区指针 (buf_pointer)
	// 字节3: 保留或未使用
	// 原始函数中 Status 未被使用，因此这里忽略 rx_buf[0]
	*payload_len = rx_buf[1];
	*buf_pointer = rx_buf[2];
}
void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
    if (data == NULL || length < 1)
        return;

    if ((3 + length) > sizeof(tx_buf) || (3 + length) > sizeof(rx_buf))
        return;

    CheckBusy();

    uint32_t total_len = 3 + length;

    tx_buf[0] = 0x1E;
    tx_buf[1] = offset;
    tx_buf[2] = 0xFF;
    memset(&tx_buf[3], 0xFF, length);
    memset(rx_buf, 0, total_len);

    spi_transfer(tx_buf, rx_buf, total_len);

    memcpy(data, &rx_buf[3], length);
}
void ClearIrqStatus(uint16_t irq)
{
	uint8_t tx_buf[3];
	CheckBusy();
	tx_buf[0] = 0x02;		  // ClearIrqStatus 命令码
	tx_buf[1] = (irq >> 8);	  // 中断掩码高字节
	tx_buf[2] = (irq & 0xFF); // 中断掩码低字节

	spi_transfer(tx_buf, NULL, 3);
}
uint8_t WaitForIRQ_RxDone(void)
{
	uint16_t Irq_Status;
	uint8_t packet_size;
	uint8_t buf_offset;
	uint8_t RF_DIO1;
	if (gpiod_line_get_value(line_dio1)) // if IRQ check
	{
		Irq_Status = GetIrqStatus(); // read Irq Status
		if ((Irq_Status & 0x02) == RxDone_IRQ)
		{
			GetRxBufferStatus(&packet_size, &buf_offset);
			if (packet_size == 0 || packet_size > lora_para_pt->payload_size)
			{
				ClearIrqStatus(RxDone_IRQ);
				RxInit();
				return 0;
			}
			ReadBuffer(buf_offset, rxbuf_pt, packet_size);
			*rxcnt_pt = packet_size;
			ClearIrqStatus(RxDone_IRQ); // Clear the IRQ RxDone flag
			RxInit();
			return 1;
		}
	}
	return 0;
}

uint8_t WaitForIRQ_TxDone(void)
{
	uint8_t time_out;

	time_out = 0;
	if (gpiod_line_get_value(line_dio1))
	{
		time_out++;
		usleep(10 * 1000);	// 10 ms
		if (time_out > 200) // if timeout , reset the the chip
		{
			ClearIrqStatus(TxDone_IRQ); // Clear the IRQ TxDone flag
			SetStandby(0);				// 0:STDBY_RC; 1:STDBY_XOSC
			reset_llcc68();				// reset RF
			LLCC68_Config();
			printf("WaitFor IRQ_TxDone time out\n");
			return 0;
		}
	}

	// Irq_Status = GetIrqStatus();
	ClearIrqStatus(TxDone_IRQ); // Clear the IRQ TxDone flag
	return 1;
}
void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
	uint8_t tx_buf[2];

	if (length < 1)
		return;

	CheckBusy();
	tx_buf[0] = 0x0E;
	tx_buf[1] = offset;
	spi_transfer(tx_buf, NULL, 2);
	spi_transfer(data, NULL, length);
}
void Lora_send(uint8_t *payload, uint8_t size)
{
	pthread_mutex_lock(&g_lora_lock);
	SetStandby(0);				// 0:STDBY_RC; 1:STDBY_Xosc
	SetBufferBaseAddress(0, 0); //(TX_base_addr,RX_base_addr)

	WriteBuffer(0, payload, size); //(offset,*data,length)
	SetPacketParams(size);		   // PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ

	SetDioIrqParams(TxDone_IRQ); // TxDone IRQ

	SetTx(0); // timeout = 0

	// Wait for the IRQ TxDone or Timeout (implement in another function)
	// printf("Sending data: %s\n", payload);
	// 7. 等待发送完成中断
	if (WaitForIRQ_TxDone())
	{
		printf("Data sent successfully.\n");
	}
	else
	{
		printf("Data send failed or timeout.\n");
	}
	RxInit(); // 发完恢复接收
	pthread_mutex_unlock(&g_lora_lock);
}
int lora_unpack(uint8_t *buf, int len,
				uint8_t expect_netid,
				uint8_t expect_devid,
				uint8_t *out_payload,
				int *out_len)
{
	if (len < 4)
		return -1; // 至少 NETID+DEVID+CRC

	// 1. CRC校验
	uint16_t recv_crc = buf[len - 2] | (buf[len - 1] << 8);
	uint16_t calc_crc = crc16(buf, len - 2);

	if (recv_crc != calc_crc)
	{
		printf("[LORA] CRC error\n");
		return -2;
	}

	// 2. NETID / DEVID 校验
	if (buf[0] != expect_netid || buf[1] != expect_devid)
	{
		printf("[LORA] ID mismatch\n");
		return -3;
	}

	// 3. 去重（完全一样才丢）
	if (len == last_len && memcmp(buf, last_frame, len) == 0)
	{
		printf("[LORA] Duplicate packet dropped\n");
		return -4;
	}

	// 保存为最新帧
	memcpy(last_frame, buf, len);
	last_len = len;

	// 4. 提取payload
	int payload_len = len - 4;
	memcpy(out_payload, &buf[2], payload_len);
	*out_len = payload_len;

	return 0;
}
void Lora_receive(uint8_t *payload, uint8_t size)
{
	rxbuf_pt = payload;
	rxcnt_pt = &size;
	SetBufferBaseAddress(0, 0);
	SetDioIrqParams(RxDone_IRQ);
	SetRx(0);
	if (WaitForIRQ_RxDone())
	{
		uint8_t payload[256];
		int payload_len;

		int ret = lora_unpack(rxbuf_pt, *rxcnt_pt,
							  0x01, 0x02, // 你的NETID/DEVID
							  payload, &payload_len);

		if (ret == 0)
		{
			printf("[LORA] Valid packet (%d bytes): ", payload_len);
			for (int i = 0; i < payload_len; i++)
				printf("%02X ", payload[i]);
			printf("\n");
		}
	}
}
int lora_pack(uint8_t netid, uint8_t devid,
			  uint8_t *payload, uint8_t payload_len,
			  uint8_t *out_buf)
{
	int len = 0;

	out_buf[len++] = netid;
	out_buf[len++] = devid;

	memcpy(&out_buf[len], payload, payload_len);
	len += payload_len;

	uint16_t crc = crc16(out_buf, len);

	out_buf[len++] = crc & 0xFF;		// 低字节
	out_buf[len++] = (crc >> 8) & 0xFF; // 高字节

	return len;
}
void Lora_send_packet(uint8_t netid, uint8_t devid,
					  uint8_t *payload, uint8_t size)
{
	uint8_t buf[256];
	int len = lora_pack(netid, devid, payload, size, buf);

	Lora_send(buf, len);
}

int Lora_recv_packet(uint8_t *payload, uint8_t *out_len)
{
	uint16_t irq_status;
	uint8_t packet_size = 0;
	uint8_t buf_offset = 0;

	if (payload == NULL || out_len == NULL)
		return -1;

	pthread_mutex_lock(&g_lora_lock);

	// 没有中断，表示暂时没有收到数据
	if (!gpiod_line_get_value(line_dio1))
	{
		pthread_mutex_unlock(&g_lora_lock);
		return 0;
	}

	irq_status = GetIrqStatus();

	if ((irq_status & RxDone_IRQ) != RxDone_IRQ)
	{
		ClearIrqStatus(irq_status);
		pthread_mutex_unlock(&g_lora_lock);
		return 0;
	}

	GetRxBufferStatus(&packet_size, &buf_offset);

	if (packet_size == 0 || packet_size > lora_para_pt->payload_size)
	{
		ClearIrqStatus(RxDone_IRQ);
		RxInit();
		pthread_mutex_unlock(&g_lora_lock);
		return -2;
	}

	ReadBuffer(buf_offset, payload, packet_size);
	*out_len = packet_size;

	ClearIrqStatus(RxDone_IRQ);
	RxInit(); // 收完立刻回到接收态

	pthread_mutex_unlock(&g_lora_lock);
	return 1;
}
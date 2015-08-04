/*
 * Cold-Junction Compensated Thermocouple-to-Digital Converter
 * http://www.maximintegrated.com/datasheet/index.mvp/id/7273
 */
#include "c_types.h"
#include "ets_sys.h"
#include "osapi.h"
#include "console.h"

#include "driver/spi_master.h"
#include "driver/spi_register.h"

#define SPI_DEV 1	// HSPI
static bool max31855_initialized = 0;
/*
 * For SPI, dev is our SS
 */
bool
max31855_init()
{
#if 0
	spi_init(1);
	spi_tx_byte_order(1, 1);	// 31:0
	spi_rx_byte_order(1, 1);	// 31:0
#else
	spi_init_gpio(SPI_DEV, SPI_CLK_USE_DIV);
	spi_clock(SPI_DEV, 4, 4); //5MHz
	spi_tx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);
	spi_rx_byte_order(SPI_DEV, SPI_BYTE_ORDER_HIGH_TO_LOW);
	//spi_rx_byte_order(SPI_DEV, SPI_BYTE_ORDER_LOW_TO_HIGH);
	SET_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_CS_SETUP|SPI_CS_HOLD);
	CLEAR_PERI_REG_MASK(SPI_USER(SPI_DEV), SPI_FLASH_MODE);
#endif

	max31855_initialized = 1;
	return true;
}

#define TC_FAULT_MASK	0x1	// One of SCV, SCG, or OC faults detected
#define SCV_FAULT_MASK	0x4	// TC shorted to Vcc
#define SCG_FAULT_MASK  0x2	// TC shorted to gnd
#define OC_FAULT_MASK   0x1	// TC is open.

/*
 * Read either the kprobe or internal temperature
 */
bool
max31855_read_temps(int16_t *kprobe, int16_t *internal)
{
	int16_t itemp=0;
	int16_t ktemp=0;
	int16_t xtemp;

#ifdef AT1284P

	// assert SS (active low)
	spi_ss(dev, 0);	// opt is our port.
	// Get k-probe temp
	xtemp = spi_xfer(0);	// [31:24]
	xtemp <<=8;
	xtemp |= spi_xfer(0);	// [23:16]
	//
	itemp |= spi_xfer(0);
	itemp <<=8;
	itemp |= spi_xfer(0);
	// release SS
	spi_ss(dev, 1);
#else
	uint32_t data;

	if (max31855_initialized == 0)
		max31855_init();

	// spi(1), cmd_bits(0), cmd(0), addr_bits(0), addr(0), dout_bits(0), dout(0), din_bits(32), dummy_bits(0)
	data = spi_transaction(1, 0, 0, 0, 0, 0, 0, 32, 0);
	console_printf("Received from SPI: %x\r\n", data);
	xtemp = ((data >>16)&0xffff);
	itemp = ((data & 0xffff));
	console_printf("xtemp is: 0x%x\r\nitemp is: 0x%x\r\n", xtemp, itemp);
#endif

	/*
	 * Check for faults.
	 */
	if (xtemp & TC_FAULT_MASK) {
		console_printf( "TC_FAULT\n");
		return false;
	}

	if (itemp & (SCV_FAULT_MASK|SCG_FAULT_MASK|OC_FAULT_MASK)) {
		console_printf( "Fault is: %x\n", itemp&(SCV_FAULT_MASK|SCG_FAULT_MASK|OC_FAULT_MASK));
		return false;
	}

		/*
		 * Convert TC temperature into degC
		 */
		xtemp >>= 2;
		ktemp = (xtemp & 0x1fff);	// 14 bit TC temp.
		if (xtemp&0x2000) {	// < 0 degC?
			ktemp = ~itemp;	// yes, 2's complement
			ktemp = data & 0x1fff;	// mask off any garbage we picked up.
			ktemp++;
			ktemp *= -1;
		}
		console_printf("ktemp*4 is %d\r\n", ktemp);
		ktemp /= 4;	// convert to degree celsius
		*kprobe = ktemp;
		/*
		 * Convert internal temperature into degC
		 */
		itemp &= 0xfff8;	// [15:4] contain a 12 bit internal temp.
		itemp = (itemp>>4)&0xfff;
		if (itemp & 0x800) {	// < 0 degC?
			itemp = ~itemp&0x7ff;	// 2's complement
			itemp += 1;
			itemp *= -1;
		}
		else
		{
			itemp &= 0x7ff;
		}
		console_printf("itemp*16 is %d\r\n", itemp);
		itemp /= 16;
		*internal = itemp;
	return true;
}

uint16_t
max31855_read_itemp()
{
	int16_t kprobe, internal;
	bool ret;

	ret = max31855_read_temps(&kprobe, &internal);
	if (ret == true)
		return internal;
	else
		return 0xffff;	// MAX31855 reported an error.
}

uint16_t
max31855_read_ktemp()
{
	int16_t kprobe, internal;
	bool ret;

	ret = max31855_read_temps(&kprobe, &internal);
	if (ret == true)
		return kprobe;
	else
		return 0xffff;
}

#ifdef CONFIG_ENABLE_MQTT
#include "lib/mqtt.h"

/*
 * Is this a reasonable limit?
 */
#define TOPIC_LEN 128

static int 
do_max31855_pub_kprobe(int argc, const char* const* argv)
{
	MQTT_Client *client = mqttGetConnectedClient();
	char buf[6];
	int buflen;
	char topic[TOPIC_LEN];

	if (client == NULL) {
		console_printf("MQTT Client not bound to broker\r\n");
		return -1;
	}

	os_sprintf(topic, "%s/max31855/kprobe/0", client->connect_info.client_id);
	buflen = os_sprintf(buf, "%d", max31855_read_ktemp());
	MQTT_Publish(client, topic, buf, buflen, 0, 0);
	return 0;
}

CONSOLE_CMD(max31855_pub_kprobe, 1, 1, 
		do_max31855_pub_kprobe, NULL, NULL, 
		"Publish Thermocouple Temperature via MQTT"
);

static int 
do_max31855_pub_internal(int argc, const char* const* argv)
{
	MQTT_Client *client = mqttGetConnectedClient();
	char buf[6];
	int buflen;
	char topic[TOPIC_LEN];

	if (client == NULL) {
		console_printf("MQTT Client not bound to broker\r\n");
		return -1;
	}

	os_sprintf(topic, "%s/max31855/internal/0", client->connect_info.client_id);
	buflen = os_sprintf(buf, "%d", max31855_read_itemp());
	MQTT_Publish(client, topic, buf, buflen, 0, 0);
	return 0;
}

static int do_max31855_temps(int argc, const char* const* argv)
{
	int16_t kprobe, internal;

	max31855_read_temps(&kprobe, &internal);
	console_printf("Internal: %d\r\n Kprobe: %d\r\n", internal, kprobe);
	return 0;
}

CONSOLE_CMD(max31855_pub_internal, 1, 1, 
		do_max31855_pub_internal, NULL, NULL, 
		"Publish On-chip Temperature via MQTT"
);

CONSOLE_CMD(temps, 1, 1, do_max31855_temps, NULL, NULL, "");

#endif // MQTT

/* Diagnostic program for DMK Engineering URI USB Radio Interface 
 *
 * Copyright (c) 2007-2009, Jim Dixon <jim@lambdatel.com>. All rights
 * reserved.
 *
 * Analog test levels changed from 700/150 to 610/130 
 * (passband/stopband) by DMK 8/22/2009.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 * 		 
 * Changes:
 * --------
 * 09/18/2023 - Danny Lloyd, KB4MDD <kb4mdd@arrl.net>
 *	moved user memory from address 6 to address 51 - we cannot overwrite manufacturer data
 * 	added ability to dump the EEPROM memory
 *	added ability to list manufacturer settings
 *	added ability to program manufacturer settings
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <usb.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <signal.h>
#include <alsa/asoundlib.h>

#ifdef __linux
#include <linux/soundcard.h>
#elif defined(__FreeBSD__)
#include <sys/soundcard.h>
#else
#include <soundcard.h>
#endif

#define C108_VENDOR_ID   	0x0d8c
#define C108_PRODUCT_ID  	0x000c
#define C108B_PRODUCT_ID  	0x0012
#define C108AH_PRODUCT_ID  	0x013c
#define C119_PRODUCT_ID 	0x0008
#define C119A_PRODUCT_ID  	0x013a
#define C119B_PRODUCT_ID    0x0013
#define N1KDO_PRODUCT_ID  	0x6a00

#define HID_REPORT_GET 0x01
#define HID_REPORT_SET 0x09

#define HID_RT_INPUT 0x01
#define HID_RT_OUTPUT 0x02

#define	AUDIO_BLOCKSIZE 4096
#define	AUDIO_SAMPLES_PER_BLOCK (AUDIO_BLOCKSIZE / 4)
#define	NFFT 1024
#define	NFFTSQRT 10

#define	AUDIO_IN_SETTING 800

#define MIXER_PARAM_MIC_PLAYBACK_SW "Mic Playback Switch"
#define MIXER_PARAM_MIC_PLAYBACK_VOL "Mic Playback Volume"
#define MIXER_PARAM_MIC_CAPTURE_SW "Mic Capture Switch"
#define MIXER_PARAM_MIC_CAPTURE_VOL "Mic Capture Volume"
#define MIXER_PARAM_MIC_BOOST "Auto Gain Control"
#define MIXER_PARAM_SPKR_PLAYBACK_SW "Speaker Playback Switch"
#define MIXER_PARAM_SPKR_PLAYBACK_VOL "Speaker Playback Volume"
#define	MIXER_PARAM_SPKR_PLAYBACK_SW_NEW "Headphone Playback Switch"
#define	MIXER_PARAM_SPKR_PLAYBACK_VOL_NEW "Headphone Playback Volume"

/*!
 * \brief EEPROM memory layout
 *	The AT93C46 eeprom has 64 addresses that contain 2 bytes (one word).
 *	The CMxxx sound card device will use this eeprom to read manuafacturer
 *	specific configuration data.
 *	The CM108 and CM119 reserves memory addresses 0 to 6.
 *	The CM119A reserves memory addresses 0 to 44.
 *	The CM119B reserves memory addresses 0 to 50.
 *
 *	The usb channel drivers store user configuration information
 *	in addresses 51 to 63.
 *
 *	The user data is zero indexed to the EEPROM_START_ADDR.
 *
 *	chan_simpleusb radio does not populate all of the available fields.
 *
 * \note Some USB devices are not manufacturered with an eeprom.
 *	Never overwrite the manufacture stored information.
 */
#define	EEPROM_START_ADDR		51	/* Start after the manufacturer info */
#define EEPROM_PHYSICAL_LEN		64
#define	EEPROM_USER_LEN			13
#define	EEPROM_MAGIC			34329
#define	EEPROM_USER_MAGIC_ADDR	 0
#define	EEPROM_USER_RXMIXERSET	 1
#define	EEPROM_USER_TXMIXASET	 2
#define	EEPROM_USER_TXMIXBSET	 3
#define	EEPROM_USER_RXVOICEADJ	 4	/* Requires 2 memory slots, stored as a float */
#define	EEPROM_USER_RXCTCSSADJ	 6	/* Requires 2 memory slots, stored as a float */
#define	EEPROM_USER_TXCTCSSADJ	 8
#define	EEPROM_USER_RXSQUELCHADJ 9
#define EEPROM_USER_TXDSPLVL	10
#define EEPROM_USER_SPARE		11	/* Reserved for future use */
#define	EEPROM_USER_CS_ADDR		12

#define PASSBAND_LEVEL		550.0
#define STOPBAND_LEVEL		117.0

/* The CM-119B requires manuafacturer specific data  in 
 * memory positions 0 to 50
 */
unsigned short cm119b_manufacturer_data[51] = {0x670d, 0x0d8c, 0x0013, 0x0000, 0x0000, 
	0x0000, 0x0000,	0x0000, 0x0000, 0x0000, 0x5522, 0x4253, 0x4120, 0x6475, 0x6f69, 
	0x4420, 0x7665, 0x6369, 0x0065, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
	0x0000, 0x4332, 0x4d2d, 0x6465, 0x6169, 0x4520, 0x656c, 0x7463, 0x6f72, 0x696e, 
	0x7363, 0x4920, 0x636e, 0x002e, 0x0000, 0x0000, 0x0000, 0x14c8, 0xf21a, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

struct tonevars {
	float mycr;
	float myci;
};

enum { DEV_C108, DEV_C108AH, DEV_C119 };

char *devtypestrs[] = { "CM108", "CM108AH", "CM119" };

void cdft(int, int, double *, int *, double *);

float myfreq1 = 0.0, myfreq2 = 0.0, lev = 0.0, lev1 = 0.0, lev2 = 0.0;

unsigned int frags = (((6 * 5) << 16) | 0xc);
int devtype = 0;
int devproductid = 0;
int devnum = -1;

/* Call with:  devnum: alsa major device number, param: ascii Formal
Parameter Name, val1, first or only value, val2 second value, or 0 
if only 1 value. Values: 0-99 (percent) or 0-1 for baboon.

Note: must add -lasound to end of linkage */

int shutdown = 0;

/*!
 * \brief Get mixer max value
 * 	Gets the mixer max value for the specified device and control.
 *
 * \param devnum		The sound device number to update.
 * \param param			Pointer to the string mixer device name (control) to retrieve.
 * 
 * \retval 				The maximum value.
 */
static int amixer_max(int devnum, char *param)
{
	int rv, type;
	char str[100];
	snd_hctl_t *hctl;
	snd_ctl_elem_id_t *id;
	snd_hctl_elem_t *elem;
	snd_ctl_elem_info_t *info;

	sprintf(str, "hw:%d", devnum);
	if (snd_hctl_open(&hctl, str, 0)) {
		return (-1);
	}
	snd_hctl_load(hctl);
	snd_ctl_elem_id_alloca(&id);
	snd_ctl_elem_id_set_interface(id, SND_CTL_ELEM_IFACE_MIXER);
	snd_ctl_elem_id_set_name(id, param);
	elem = snd_hctl_find_elem(hctl, id);
	if (!elem) {
		snd_hctl_close(hctl);
		return (-1);
	}
	snd_ctl_elem_info_alloca(&info);
	snd_hctl_elem_info(elem, info);
	type = snd_ctl_elem_info_get_type(info);
	rv = 0;
	switch (type) {
	case SND_CTL_ELEM_TYPE_INTEGER:
		rv = snd_ctl_elem_info_get_max(info);
		break;
	case SND_CTL_ELEM_TYPE_BOOLEAN:
		rv = 1;
		break;
	}
	snd_hctl_close(hctl);
	return (rv);
}

/* Call with:  devnum: alsa major device number, param: ascii Formal
Parameter Name, val1, first or only value, val2 second value, or 0 
if only 1 value. Values: 0-99 (percent) or 0-1 for baboon.

Note: must add -lasound to end of linkage */
/*!
 * \brief Set mixer
 * 	Sets the mixer values for the specified device and control.
 *
 * \param devnum		The sound device number to update.
 * \param param			Pointer to the string mixer device name (control) to update.
 * \param v1			Value 1 to set.
 * \param v2			Value 2 to set.
 */
static int setamixer(int devnum, char *param, int v1, int v2)
{
	int type;
	char str[100];
	snd_hctl_t *hctl;
	snd_ctl_elem_id_t *id;
	snd_ctl_elem_value_t *control;
	snd_hctl_elem_t *elem;
	snd_ctl_elem_info_t *info;

	sprintf(str, "hw:%d", devnum);
	if (snd_hctl_open(&hctl, str, 0)) {
		return (-1);
	}
	snd_hctl_load(hctl);
	snd_ctl_elem_id_alloca(&id);
	snd_ctl_elem_id_set_interface(id, SND_CTL_ELEM_IFACE_MIXER);
	snd_ctl_elem_id_set_name(id, param);
	elem = snd_hctl_find_elem(hctl, id);
	if (!elem) {
		snd_hctl_close(hctl);
		return (-1);
	}
	snd_ctl_elem_info_alloca(&info);
	snd_hctl_elem_info(elem, info);
	type = snd_ctl_elem_info_get_type(info);
	snd_ctl_elem_value_alloca(&control);
	snd_ctl_elem_value_set_id(control, id);
	switch (type) {
	case SND_CTL_ELEM_TYPE_INTEGER:
		snd_ctl_elem_value_set_integer(control, 0, v1);
		if (v2 > 0)
			snd_ctl_elem_value_set_integer(control, 1, v2);
		break;
	case SND_CTL_ELEM_TYPE_BOOLEAN:
		snd_ctl_elem_value_set_integer(control, 0, (v1 != 0));
		break;
	}
	if (snd_hctl_elem_write(elem, control)) {
		snd_hctl_close(hctl);
		return (-1);
	}
	snd_hctl_close(hctl);
	return (0);
}

/*!
 * \brief Set USB HID outputs
 * 	This routine, depending on the outputs passed can set the GPIO states 
 *	and/or setup the chip to read/write the eeprom.
 *
 *	The passed outputs should be 4 bytes.
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param outputs		Pointer to buffer that contains the data to send to the HID.
 */
static void set_outputs(struct usb_dev_handle *handle, unsigned char *outputs)
{
	usleep(1500);
	usb_control_msg(handle,
					USB_ENDPOINT_OUT + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
					HID_REPORT_SET,
					0 + (HID_RT_OUTPUT << 8), 3, (char *) outputs, 4, 5000);
}

/* Set USB outputs */
static void setout(struct usb_dev_handle *usb_handle, unsigned char c)
{
	unsigned char buf[4];

	buf[0] = buf[3] = 0;
	if (devtype == DEV_C119) {
		buf[2] = 0x3d;			/* set GPIO 1,3,4,5,6 as output */
	} else {
		buf[2] = 0xd;			/* set GPIO 1,3,4 as output */
	}
	buf[1] = c;					/* set GPIO 1,3,4 (5,7) outputs appropriately */
	set_outputs(usb_handle, buf);
	usleep(100000);
}

/*!
 * \brief Get USB HID inputs
 * 	This routine will retrieve the GPIO states or data the eeprom.
 *
 *	The passed inputs should be 4 bytes.
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param inputs		Pointer to buffer that will contain the data received from the HID.
 */
static void get_inputs(struct usb_dev_handle *handle, unsigned char *inputs)
{
	usleep(1500);
	usb_control_msg(handle,
					USB_ENDPOINT_IN + USB_TYPE_CLASS + USB_RECIP_INTERFACE,
					HID_REPORT_GET, 0 + (HID_RT_INPUT << 8), 3, (char *) inputs, 4, 5000);
}

/* Set USB inputs */
unsigned char getin(struct usb_dev_handle *usb_handle)
{
	unsigned char buf[4];
	unsigned short c;

	buf[0] = buf[1] = 0;
	get_inputs(usb_handle, buf);
	c = buf[1] & 0xf;
	c += (buf[0] & 3) << 4;
	if (devtype == DEV_C119) {
		c += buf[1] & 0xc0;
	}
	/* in the AH part, the HOOK comes in on buf[0] bit 4, undocumented */
	if (devtype == DEV_C108AH) {
		c &= 0xfd;
		if (!(buf[0] & 0x10)) {
			c += 2;
		}
	}
	return (c);
}

/*!
 * \brief Read CM-xxx EEPROM
 * 	Read a memory position from the EEPROM attached to the CM-XXX device.
 *	One memory position is two bytes.
 *
 *	Four bytes are passed to the device to configure it for an EEPROM read.
 *	The first byte should be 0x80, the fourth byte should be 0x80 or'd with
 *	the address to read.  
 *
 *	After the address has been set, a get input is done to read the returned
 *	bytes.
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param addr			Integer address to read from the EEPROM.  The valid
 *						range is 0 to 63.
 */
static unsigned short read_eeprom(struct usb_dev_handle *usb_handle, int addr)
{
	unsigned char buf[4];

	buf[0] = 0x80;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0x80 | (addr & 0x3f);

	usleep(500);
	set_outputs(usb_handle, buf);
	memset(buf, 0, sizeof(buf));
	usleep(500);
	get_inputs(usb_handle, buf);

	return (buf[1] + (buf[2] << 8));
}

/*!
 * \brief Read user memory segment from the CM-XXX EEPROM.
 * 	Reads the memory range associated with user data from the EEPROM.
 *
 *	The user memory segment is from address position 51 to 63.
 *	Memory positions 0 to 50 are reserved for manufacturer's data.
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param buf			Pointer to buffer to receive the EEPROM data.  The buffer
 *						must be an array of 13 unsigned shorts.
 *
 * \retval				Checksum of the received data.  If the check sum is correct,
 *						the calculated checksum will be zero.  This indicates valid data..
 *						Any	other value indicates bad EEPROM data.
 */
static unsigned short get_eeprom(struct usb_dev_handle *handle, unsigned short *buf)
{
	int i;
	unsigned short cs;

	cs = 0xffff;
	for (i = EEPROM_START_ADDR; i <= EEPROM_START_ADDR + EEPROM_USER_CS_ADDR; i++) {
		cs += buf[i - EEPROM_START_ADDR] = read_eeprom(handle, i);
	}


	return (cs);
}

/*!
 * \brief Read all memory from the CM-XXX EEPROM.
 * 	Reads the entire memory range from the EEPROM.
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param buf			Pointer to buffer to receive the EEPROM data.  The buffer
 *						must be an array of 13 unsigned shorts.
 *
 * \retval				Checksum of the received data.  If the check sum is correct,
 *						the calculated checksum will be zero.  This indicates valid data..
 *						Any	other value indicates bad EEPROM data.
 */
static void get_eeprom_dump(struct usb_dev_handle *handle, unsigned short *buf)
{
	int i;

	for (i = 0; i < 64; i++) {
		buf[i] = read_eeprom(handle, i);
	}
}

/*!
 * \brief Write CM-xxx EEPROM
 * 	Write a memory position in the EEPROM attached to the CM-XXX device.
 *	One memory position is two bytes.
 *
 *	Four bytes are passed to the device to write the value.  The first byte 
 *	should be 0x80, the second byte should be the lsb of the data, the third
 *	byte is the msb of the data, the fourth byte should be 0xC0 or'd with
 *	the address to write.
 *
 * \note This routine will write to any valid memory address.  Never write
 *	to address 0 to 50.  These are reserved for manufacturer data.
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param addr			Integer address to read from the EEPROM.  The valid
 *						range is 0 to 63.
 * \param data			Unsigned short data to store.
 */
static void write_eeprom(struct usb_dev_handle *usb_handle, int addr, unsigned short data)
{
	unsigned char buf[4];

	buf[0] = 0x80;
	buf[1] = data & 0xff;
	buf[2] = data >> 8;
	buf[3] = 0xc0 | (addr & 0x3f);

	usleep(2000);
	set_outputs(usb_handle, buf);
}

/*!
 * \brief Write user memory segment to the CM-XXX EEPROM.
 * 	Writes the memory range associated with user data to the EEPROM.
 *
 *	The user memory segment is from address position 51 to 63.
 *	
 *  \note Memory positions 0 to 50 are reserved for manufacturer's data.  Do not
 *	write into this segment!
 *
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param buf			Pointer to buffer that contains the the EEPROM data.  
 *						The buffer must be an array of 13 unsigned shorts.
 */
static void put_eeprom(struct usb_dev_handle *handle, unsigned short *buf)
{
	int i;
	unsigned short cs;

	cs = 0xffff;
	buf[EEPROM_USER_MAGIC_ADDR] = EEPROM_MAGIC;
	for (i = EEPROM_START_ADDR; i < EEPROM_START_ADDR + EEPROM_USER_CS_ADDR; i++) {
		write_eeprom(handle, i, buf[i - EEPROM_START_ADDR]);
		cs += buf[i];
	}
	buf[EEPROM_USER_CS_ADDR] = (65535 - cs) + 1;
	usleep(2000);
	write_eeprom(handle, i, buf[EEPROM_USER_CS_ADDR]);
}

/*!
 * \brief Write manufacturer memory segment to the CM-119B EEPROM.
 * 	Writes the memory range associated with manufacturer data to the EEPROM.
 *
 *	The manufacturer memory segment is from address position 0 to 50.
 *	
 * \param handle		Pointer to usb_dev_handle associated with the HID.
 * \param buf			Pointer to buffer that contains the the EEPROM data.  
 *						The buffer must be an array of 13 unsigned shorts.
 */
static void put_eeprom_mfg_data(struct usb_dev_handle *handle)
{
	int i;

	for (i = 0; i < sizeof(cm119b_manufacturer_data); i++) {
		write_eeprom(handle, i, cm119b_manufacturer_data[i]);
	}
	
	printf("CM-119B Manufacturer data updated.\n");
}

/*!
 * \brief Initialize a USB device.
 * 	Searches for the first USB device that is compatible.
 *
 * \note It will only evaluate USB devices known to work with this application.
 *
 * \retval 					Returns a usb_device structure with the found device.
 *							If the device was not found, it returns null.
 */
static struct usb_device *device_init(void)
{
	struct usb_bus *usb_bus;
	struct usb_device *dev;
	char devstr[10000], str[200], desdev[200], *cp;
	int i;
	FILE *fp;

	usb_init();
	usb_find_busses();
	usb_find_devices();
	for (usb_bus = usb_busses; usb_bus; usb_bus = usb_bus->next) {
		for (dev = usb_bus->devices; dev; dev = dev->next) {
			if ((dev->descriptor.idVendor == C108_VENDOR_ID) &&
				(((dev->descriptor.idProduct & 0xfffc) == C108_PRODUCT_ID) ||
				 (dev->descriptor.idProduct == C108B_PRODUCT_ID) ||
				 (dev->descriptor.idProduct == C108AH_PRODUCT_ID) ||
				 (dev->descriptor.idProduct == C119A_PRODUCT_ID) ||
				 (dev->descriptor.idProduct == C119B_PRODUCT_ID) ||
				 ((dev->descriptor.idProduct & 0xff00) == N1KDO_PRODUCT_ID) ||
				 (dev->descriptor.idProduct == C119_PRODUCT_ID))) {
				sprintf(devstr, "%s/%s", usb_bus->dirname, dev->filename);

				for (i = 0; i < 32; i++) {
					sprintf(str, "/proc/asound/card%d/usbbus", i);
					fp = fopen(str, "r");
					if (!fp) {
						continue;
					}
					if ((!fgets(desdev, sizeof(desdev) - 1, fp)) || (!desdev[0])) {
						fclose(fp);
						continue;
					}
					fclose(fp);
					if (desdev[strlen(desdev) - 1] == '\n')
						desdev[strlen(desdev) - 1] = 0;
					if (strcasecmp(desdev, devstr)) {
						continue;
					}
					if (i) {
						sprintf(str, "/sys/class/sound/dsp%d/device", i);
					} else {
						strcpy(str, "/sys/class/sound/dsp/device");
					}
					memset(desdev, 0, sizeof(desdev));
					if (readlink(str, desdev, sizeof(desdev) - 1) == -1) {
						sprintf(str, "/sys/class/sound/controlC%d/device", i);
						memset(desdev, 0, sizeof(desdev));
						if (readlink(str, desdev, sizeof(desdev) - 1) == -1) {
							continue;
						}
					}
					cp = strrchr(desdev, '/');
					if (cp) {
						*cp = 0;
					} else {
						continue;
					}
					cp = strrchr(desdev, '/');
					if (!cp) {
						continue;
					}
					cp++;
					break;
				}
				if (i >= 32) {
					continue;
				}
				devtype = DEV_C108;
				devproductid = dev->descriptor.idProduct;
				if (dev->descriptor.idProduct == C108AH_PRODUCT_ID) {
					devtype = DEV_C108AH;
				} else if (dev->descriptor.idProduct == C119_PRODUCT_ID) {
					devtype = DEV_C119;
				}
				printf("Found %s USB Radio Interface at %s\n", devtypestrs[devtype],
					   devstr);
				devnum = i;
				return dev;
			}
		}
	}
	return NULL;
}

/* Evaluate the integer and return "1" or "0" */
static inline char *baboons(int v)
{
	if (v) {
		return "1";
	}
	return "0";
}

/* Print errors that were encountered */
static int dioerror(unsigned char got, unsigned char should)
{
	unsigned char err = got ^ should;
	int n = 0;

	if (err & 0x2) {
		printf("Error on GPIO1/GPIO2, got %s, should be %s\n",
			   baboons(got & 2), baboons(should & 2));
		n++;
	}
	if (err & 0x10) {
		printf("Error on GPIO3/PTT/COR IN, got %s, should be %s\n",
			   baboons(got & 0x10), baboons(should & 0x10));
		n++;
	}
	if (err & 0x40) {
		printf("Error on GPIO5/GPIO7, got %s, should be %s\n",
			   baboons(got & 0x40), baboons(should & 0x40));
		n++;
	}
	if (err & 0x80) {
		printf("Error on GPIO5/GPIO7, got %s, should be %s\n",
			   baboons(got & 0x80), baboons(should & 0x80));
		n++;
	}
	return (n);
}

/* Test output */
static int testio(struct usb_dev_handle *usb_handle, unsigned char toout,
				  unsigned char toexpect)
{
	unsigned char c;

	setout(usb_handle, toout);	/* should readback 0 */
	c = getin(usb_handle) & 0xf2;
	return (dioerror(c, toexpect));
}

/* get tone sample */
static float get_tonesample(struct tonevars *tvars, float ddr, float ddi)
{

	float t;

	t = tvars->mycr * ddr - tvars->myci * ddi;
	tvars->myci = tvars->mycr * ddi + tvars->myci * ddr;
	tvars->mycr = t;

	t = 2.0 - (tvars->mycr * tvars->mycr + tvars->myci * tvars->myci);
	tvars->mycr *= t;
	tvars->myci *= t;
	if ((devtype == DEV_C108AH) || (devtype == DEV_C119))
		return tvars->mycr;
	return tvars->mycr * 0.9092;
}

/* Output audio */
static int outaudio(int fd, float freq1, float freq2)
{
	unsigned short buf[AUDIO_SAMPLES_PER_BLOCK * 2];
	float f, ddr1, ddi1, ddr2, ddi2;
	int i;
	static struct tonevars t1, t2;

	if (freq1 > 0.0) {
		ddr1 = cos(freq1 * 2.0 * M_PI / 48000.0);
		ddi1 = sin(freq1 * 2.0 * M_PI / 48000.0);
	} else {
		t1.mycr = 1.0;
		t1.myci = 0.0;
	}

	if (freq2 > 0.0) {
		ddr2 = cos(freq2 * 2.0 * M_PI / 48000.0);
		ddi2 = sin(freq2 * 2.0 * M_PI / 48000.0);
	} else {
		t2.mycr = 1.0;
		t2.myci = 0.0;
	}
	for (i = 0; i < AUDIO_SAMPLES_PER_BLOCK * 2; i += 2) {
		if (freq1 > 0.0) {
			f = get_tonesample(&t1, ddr1, ddi1);
			buf[i] = f * 32765;
		} else
			buf[i] = 0;
		if (freq2 > 0.0) {
			f = get_tonesample(&t2, ddr2, ddi2);
			buf[i + 1] = f * 32765;
		} else
			buf[i + 1] = 0;
	}
	if (write(fd, buf, AUDIO_BLOCKSIZE) != AUDIO_BLOCKSIZE) {
		return (-1);
	}
	return 0;
}

/* Open the sound device */
static int soundopen(int devicenum)
{
	int fd, res, fmt, desired;
	char device[200];

	strcpy(device, "/dev/dsp");
	if (devicenum) {
		sprintf(device, "/dev/dsp%d", devicenum);
	}
	fd = open(device, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		printf("Unable to re-open DSP device %d: %s\n", devicenum, device);
		return -1;
	}
#if __BYTE_ORDER == __LITTLE_ENDIAN
	fmt = AFMT_S16_LE;
#else
	fmt = AFMT_S16_BE;
#endif
	res = ioctl(fd, SNDCTL_DSP_SETFMT, &fmt);
	if (res < 0) {
		printf("Unable to set format to 16-bit signed\n");
		return -1;
	}
	res = ioctl(fd, SNDCTL_DSP_SETDUPLEX, 0);
	/* Check to see if duplex set (FreeBSD Bug) */
	res = ioctl(fd, SNDCTL_DSP_GETCAPS, &fmt);
	if ((res != 0) || (!(fmt & DSP_CAP_DUPLEX))) {
		printf("Doesnt have full duplex mode\n");
		return -1;
	}
	fmt = 1;
	res = ioctl(fd, SNDCTL_DSP_STEREO, &fmt);
	if (res < 0) {
		printf("Failed to set audio device to mono\n");
		return -1;
	}
	fmt = desired = 48000;
	res = ioctl(fd, SNDCTL_DSP_SPEED, &fmt);
	if (res < 0) {
		printf("Failed to set audio device to 48k\n");
		return -1;
	}
	if (fmt != desired) {
		printf("Requested %d Hz, got %d Hz -- sound may be choppy\n", desired, fmt);
	}
	/*
	 * on Freebsd, SETFRAGMENT does not work very well on some cards.
	 * Default to use 256 bytes, let the user override
	 */
	if (frags) {
		fmt = frags;
		res = ioctl(fd, SNDCTL_DSP_SETFRAGMENT, &fmt);
		if (res < 0) {
			printf("Unable to set fragment size -- sound may be choppy\n");
		}
	}
	/* on some cards, we need SNDCTL_DSP_SETTRIGGER to start outputting */
	res = PCM_ENABLE_INPUT | PCM_ENABLE_OUTPUT;
	res = ioctl(fd, SNDCTL_DSP_SETTRIGGER, &res);
	/* it may fail if we are in half duplex, never mind */
	return fd;
}

/* Sound card processing thread */
void *soundthread(void *this)
{
	int fd, micmax, spkrmax;
	char newname = 0;

	fd = soundopen(devnum);
	micmax = amixer_max(devnum, MIXER_PARAM_MIC_CAPTURE_VOL);
	spkrmax = amixer_max(devnum, MIXER_PARAM_SPKR_PLAYBACK_VOL);

	if (spkrmax == -1) {
		newname = 1;
		spkrmax = amixer_max(devnum, MIXER_PARAM_SPKR_PLAYBACK_VOL_NEW);
	}

	setamixer(devnum, MIXER_PARAM_MIC_PLAYBACK_SW, 0, 0);
	setamixer(devnum, MIXER_PARAM_MIC_PLAYBACK_VOL, 0, 0);
	setamixer(devnum,
			  (newname) ? MIXER_PARAM_SPKR_PLAYBACK_SW_NEW : MIXER_PARAM_SPKR_PLAYBACK_SW,
			  1, 0);
	setamixer(devnum,
			  (newname) ? MIXER_PARAM_SPKR_PLAYBACK_VOL_NEW :
			  MIXER_PARAM_SPKR_PLAYBACK_VOL, spkrmax, spkrmax);
	setamixer(devnum, MIXER_PARAM_MIC_CAPTURE_VOL, AUDIO_IN_SETTING * micmax / 1000, 0);
	setamixer(devnum, MIXER_PARAM_MIC_BOOST, 0, 0);
	setamixer(devnum, MIXER_PARAM_MIC_CAPTURE_SW, 1, 0);

	while (!shutdown) {
		fd_set rfds, wfds;
		int res;
		char buf[AUDIO_BLOCKSIZE];
		float mylev, mylev1, mylev2;

		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		FD_SET(fd, &rfds);
		FD_SET(fd, &wfds);
		res = select(fd + 1, &rfds, &wfds, NULL, NULL);
		if (!res) {
			continue;
		}
		if (res < 0) {
			perror("poll");
			exit(255);
		}
		if (FD_ISSET(fd, &wfds)) {
			outaudio(fd, myfreq1, myfreq2);
			continue;
		}
		if (FD_ISSET(fd, &rfds)) {
			short *sbuf = (short *) buf;
			static double afft[(NFFT + 1) * 2 + 1], wfft[NFFT * 5 / 2];
			float buck;
			float gfac;
			static int ipfft[NFFTSQRT + 2], i;

			res = read(fd, buf, AUDIO_BLOCKSIZE);
			if (res < AUDIO_BLOCKSIZE) {
				printf("Warining, short read!!\n");
				continue;
			}
			memset(afft, 0, sizeof(double) * 2 * (NFFT + 1));
			gfac = 1.0;
			if ((devtype == DEV_C108AH) || (devtype == DEV_C119)) {
				gfac = 0.7499;
			}
			for (i = 0; i < res / 2; i++) {
				sbuf[i] = (int) (((float) sbuf[i] + 32768) * gfac) - 32768;

			}
			for (i = 0; i < NFFT * 2; i += 2) {
				afft[i] = (double) (sbuf[i] + 32768) / (double) 65536.0;
			}
			ipfft[0] = 0;
			cdft(NFFT * 2, -1, afft, ipfft, wfft);
			mylev = 0.0;
			mylev1 = 0.0;
			mylev2 = 0.0;
			for (i = 1; i < NFFT / 2; i++) {
				float ftmp;

				ftmp = (afft[i * 2] * afft[i * 2]) + (afft[i * 2 + 1] * afft[i * 2 + 1]);

				mylev += ftmp;
				buck = (float) i *46.875;
				if (myfreq1 > 0.0) {
					if (fabs(buck - myfreq1) < 1.5 * 46.875)
						mylev1 += ftmp;
				}
				if (myfreq2 > 0.0) {
					if (fabs(buck - myfreq2) < 1.5 * 46.875) {
						mylev2 += ftmp;
					}
				}
			}
			lev = (sqrt(mylev) / (float) (NFFT / 2)) * 4096.0;
			lev1 = (sqrt(mylev1) / (float) (NFFT / 2)) * 4096.0;
			lev2 = (sqrt(mylev2) / (float) (NFFT / 2)) * 4096.0;
		}
	}
	close(fd);
	pthread_exit(NULL);
}

/* Digital I/O test */
static int digital_test(struct usb_dev_handle *usb_handle)
{
	int nerror = 0;

	printf("Testing digital I/O (PTT,COR,TONE and GPIO)....\n");
	nerror += testio(usb_handle, 8, 0);	/* NONE */
	nerror += testio(usb_handle, 9, 2);	/* GPIO1 -> GPIO2 */
	nerror += testio(usb_handle, 0xc, 0x10);	/* GPIO3/PTT -> CTCSS */
	nerror += testio(usb_handle, 0, 0x20);	/* GPIO4 -> COR */
	if (devtype == DEV_C119) {
		nerror += testio(usb_handle, 0x18, 0x40);	/* GPIO5 -> GPIO7 */
		nerror += testio(usb_handle, 0x28, 0x80);	/* GPIO6 -> GPIO8 */
	}
	nerror += testio(usb_handle, 8, 0);	/* NONE */
	if (!nerror) {
		printf("Digital I/O passed!!\n");
	} else {
		printf("Digital I/O had %d errors!!\n", nerror);
	}
	return (nerror);
}

/* Test audio */
static int analog_test_one(float freq1, float freq2, float dlev1, float dlev2, int v)
{
	int nerror = 0;
	myfreq1 = freq1;
	myfreq2 = freq2;
	printf("Testing Analog at %1.f (and %1.f) Hz...\n", freq1, freq2);
	usleep(1000000);
	if (fabs(lev1 - dlev1) > (dlev1 * 0.2)) {
		printf("Analog level on left channel for %.1f Hz (%.1f) is out of range!!\n",
			   freq1, lev1);
		printf("Must be between %.1f and %.1f\n", dlev1 * .8, dlev1 * 1.2);
		nerror++;
	} else if (v) {
		printf("Left channel level %.1f OK at %.1f Hz\n", lev1, freq1);
	}
	if (fabs(lev2 - dlev2) > (dlev2 * 0.2)) {
		printf("Analog level on right channel for %.1f Hz (%.1f) is out of range!!\n",
			   freq2, lev2);
		printf("Must be between %.1f and %.1f\n", dlev2 * .8, dlev2 * 1.2);
		nerror++;
	} else if (v) {
		printf("Right channel level %.1f OK at %.1f Hz\n", lev2, freq2);
	}
	return (nerror);
}

/* Perform analog test */
static int analog_test(int v)
{
	int nerror = 0;

	nerror += analog_test_one(204.0, 700.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(504.0, 700.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(1004.0, 700.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(2004.0, 700.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(3004.0, 700.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(5004.0, 700.0, STOPBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(700.0, 204.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(700.0, 504.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(700.0, 1004.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(700.0, 2004.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(700.0, 3004.0, PASSBAND_LEVEL, PASSBAND_LEVEL, v);
	nerror += analog_test_one(700.0, 5004.0, PASSBAND_LEVEL, STOPBAND_LEVEL, v);
	if (!nerror) {
		printf("Analog Test Passed!!\n");
	}
	return (nerror);
}
/* Test the EEPROM by writing a short to our spare memory position */
static int eeprom_test(struct usb_dev_handle *usb_handle)
{
	int i, nerror = 0;
	
	write_eeprom(usb_handle, EEPROM_START_ADDR + EEPROM_USER_SPARE, 0x6942);
	
	i = read_eeprom(usb_handle, EEPROM_START_ADDR + EEPROM_USER_SPARE);
	if (i != 0x6942) {
		printf("Error!! EEPROM wrote 6942 hex, read %04x hex\n", i);
		nerror++;
	} else {
		printf("Success - EEPROM wrote 6942 hex, read %04x hex\n", i);
	}
	return (nerror);
}

/* List the user EEPROM settings */
static int eeprom_list(struct usb_dev_handle *usb_handle)
{
	unsigned short sbuf[EEPROM_USER_LEN];
	int i, nerror = 0;
	float f;

	i = get_eeprom(usb_handle, sbuf);

	if (i) {
		printf("Failure!! EEPROM fail checksum or not present\n");
		printf("Check Sum, %i, is invalid.\n", i);
		nerror++;
	}
	if (sbuf[EEPROM_USER_MAGIC_ADDR] != EEPROM_MAGIC) {
		printf("Error!! EEPROM MAGIC BAD or not present, got %04x hex, expected %04x hex\n",
			   sbuf[EEPROM_USER_MAGIC_ADDR], EEPROM_MAGIC);
		nerror++;
	}
	if (nerror) {
		return (nerror);
	}
	printf("Magic       = %i\n", sbuf[EEPROM_USER_MAGIC_ADDR]);
	printf("rxmixerset  = %i\n", sbuf[EEPROM_USER_RXMIXERSET]);
	printf("txmixaset   = %i\n", sbuf[EEPROM_USER_TXMIXASET]);
	printf("txmixbset   = %i\n", sbuf[EEPROM_USER_TXMIXBSET]);
	memcpy(&f, &sbuf[EEPROM_USER_RXVOICEADJ], sizeof(float));
	printf("rxvoiceadj  = %f\n", f);
	memcpy(&f, &sbuf[EEPROM_USER_RXCTCSSADJ], sizeof(float));
	printf("rxctcssadj  = %f\n", f);
	printf("txctcssadj  = %i\n", sbuf[EEPROM_USER_TXCTCSSADJ]);
	printf("rxsquelchadj= %i\n", sbuf[EEPROM_USER_RXSQUELCHADJ]);
	printf("txdsplvl    = %i\n", sbuf[EEPROM_USER_TXDSPLVL]);
	printf("spare       = %i\n", sbuf[EEPROM_USER_SPARE]);
	printf("check sum   = %i\n", sbuf[EEPROM_USER_CS_ADDR]);

	return (0);
}

/* Print the entire EEPROM memory contents */
static int eeprom_dump(struct usb_dev_handle *usb_handle)
{
	unsigned short sbuf[64];
	int i;

	get_eeprom_dump(usb_handle, sbuf);

	printf("EEPROM dump\n");

	for (i = 0; i < 64; i++) {
		printf("%02i - %04x - %d\n", i, sbuf[i], sbuf[i]);
	}
	printf("\n");

	return (0);
}

/* Print the manufacturer programmed data */
static int eeprom_list_manufacturer(struct usb_dev_handle *usb_handle)
{
	unsigned short sbuf[64];
	char s[31];

	get_eeprom_dump(usb_handle, sbuf);

	printf("Device id %04x\n", devproductid);
	printf("EEPROM manufacturer data...\n");
	
	if ((sbuf[0] & 0x6700) != 0x6700) {
		printf("No manufacturer data present.  Magic %04x did not match 0x670x.\n", sbuf[0]);
		return 0;
	}

	printf("Magic            : %04x\n", sbuf[0]);
	if (devproductid == C108_PRODUCT_ID || devproductid == C108AH_PRODUCT_ID ||
		devproductid == C108B_PRODUCT_ID || devproductid == C119_PRODUCT_ID) {
			printf("  Serial #       : %s\n", sbuf[0] & 0x02 ? "enabled" : "disabled");
			printf("  Product string : %s\n", sbuf[0] & 0x01 ? "enabled" : "disabled");
	}
	if (devproductid == C119A_PRODUCT_ID) {
			printf("  Address 0x2A   : %s\n", sbuf[0] & 0x08 ? "enabled" : "disabled");
			printf("  Manufact String: %s\n", sbuf[0] & 0x04 ? "enabled" : "disabled");
			printf("  Serial #       : %s\n", sbuf[0] & 0x02 ? "enabled" : "disabled");
			printf("  Product string : %s\n", sbuf[0] & 0x01 ? "enabled" : "disabled");
	}
	if (devproductid == C119B_PRODUCT_ID) {
			printf("  Address 0x2A   : %s\n", sbuf[0] & 0x08 ? "enabled" : "disabled");
			printf("  Reserved       : %s\n", sbuf[0] & 0x04 ? "1" : "0");
			printf("  Serial #       : %s\n", sbuf[0] & 0x02 ? "enabled" : "disabled");
			printf("  Reserved       : %s\n", sbuf[0] & 0x01 ? "1" : "0");
	}
	printf("  VID            : %04x\n", sbuf[1]);
	printf("  PID            : %04x\n", sbuf[2]);
	if (devproductid == C119A_PRODUCT_ID || devproductid == C119B_PRODUCT_ID) {
			printf("  Serial # length: %i\n", sbuf[3]);
			memset(s, 0, sizeof(s));
			printf("  Serial #       : %.12s\n", (char *)sbuf +4);
			
			if (devproductid == C119B_PRODUCT_ID) {
				printf("  Product length : %i\n", sbuf[10] & 0xFF);
				printf("  Product        : %c%.30s\n", sbuf[10] >> 8, (char *)sbuf + (11 * 2));
				printf("  Mfg length     : %i\n", sbuf[26] & 0xFF);
				printf("  Manufacturer   : %c%.30s\n", sbuf[26] >> 8, (char *)sbuf + (27 * 2));
				printf("  DAC Volume     : %i\n", (sbuf[42] & 0xFE00) >> 9);
				printf("  ADC Volume     : %i\n", (sbuf[42] & 0x1F8) >>3);
				printf("  DAC Max/Min    : %s\n", sbuf[42] & 0x4 ? "valid" : "invalid");
				printf("  ADC Max/Min    : %s\n", sbuf[42] & 0x2 ? "valid" : "invalid");
				printf("  AA Max/Min     : %s\n", sbuf[42] & 0x1 ? "valid" : "invalid");
				printf("  AA Volume      : %i\n", (sbuf[43] & 0xF800) >> 11);
				printf("  Reserved       : %s\n", sbuf[43] & 0x400 ? "1" : "0");
				printf("  Boost Mode     : %s\n", sbuf[43] & 0x200 ? "22db" : "12db");
				printf("  Reserved       : %s\n", sbuf[43] & 0x100 ? "1" : "0");
				printf("  Power control  : %s\n", sbuf[43] & 0x80 ? "enabled" : "disabled");
				printf("  Reserved       : %s\n", sbuf[43] & 0x40? "1" : "0");
				printf("  MIC high pass  : %s\n", sbuf[43] & 0x20 ? "enabled" : "disabled");
				printf("  MIC PLL Adjust : %s\n", sbuf[43] & 0x10 ? "enabled" : "disabled");
				printf("  MIC boost      : %s\n", sbuf[43] & 0x8 ? "enabled" : "disabled");
				printf("  DAC output     : %s\n", sbuf[43] & 0x4 ? "headset" : "speaker");
				printf("  HID enable     : %s\n", sbuf[43] & 0x2 ? "enabled" : "disabled");
				printf("  Wakeup         : %s\n", sbuf[43] & 0x1 ? "enabled" : "disabled");
				printf("  DAC Min Volume : %i\n", sbuf[44]);
				printf("  DAC Max Volume : %i\n", sbuf[45]);
				printf("  ADC Min Volume : %i\n", sbuf[46]);
				printf("  ADC Max Volume : %i\n", sbuf[47]);
				printf("  AA Min Volume  : %i\n", sbuf[48]);
				printf("  AA Max Volume  : %i\n", sbuf[49]);
				printf("  Option 2 Reg   : 0x%04x\n", sbuf[50]);
			} else {
				printf("  Product length : %i\n", sbuf[10]);
				printf("  Product        : %.30s\n", (char *)sbuf + 11);
				printf("  Mfg length     : %i\n", sbuf[26]);
				printf("  Manufacturer   : %.30s\n", (char *)sbuf + 27);
				printf("  DAC Volume     : %i\n", (sbuf[42] & 0xFF00) >> 8);
				printf("  ADC Volume     : %i\n", (sbuf[42] & 0xFF));
				printf("  Reserved       : %i\n", (sbuf[43] & 0xFE00) >> 9);
				printf("  Shutdown DAC   : %s\n", sbuf[43] & 0x100 ? "enabled" : "disabled");
				printf("  Power control  : %s\n", sbuf[43] & 0x80 ? "enabled" : "disabled");
				printf("  Reserved       : %s\n", sbuf[43] & 0x40? "1" : "0");
				printf("  MIC high pass  : %s\n", sbuf[43] & 0x20 ? "enabled" : "disabled");
				printf("  ADC sync mode  : %s\n", sbuf[43] & 0x10 ? "enabled" : "disabled");
				printf("  MIC boost      : %s\n", sbuf[43] & 0x8 ? "enabled" : "disabled");
				printf("  DAC output     : %s\n", sbuf[43] & 0x4 ? "headset" : "speaker");
				printf("  HID enable     : %s\n", sbuf[43] & 0x2 ? "enabled" : "disabled");
				printf("  Wakeup         : %s\n", sbuf[43] & 0x1 ? "enabled" : "disabled");
			}
	}

	printf("\n");

	return (0);
}

/* Initialize the user EEPROM memory */
static void eeprom_init(struct usb_dev_handle *usb_handle)
{
	unsigned short sbuf[64];

	memset(sbuf, 0, sizeof(sbuf));
	put_eeprom(usb_handle, sbuf);
}

/* Main program start */
int main(int argc, char **argv)
{
	struct usb_device *usb_dev;
	struct usb_dev_handle *usb_handle;
	int retval = 1;
	char c;
	pthread_t sthread;
	pthread_attr_t attr;
	struct termios t, t0;
	float myfreq;

	printf("URIDiag, diagnostic program for the DMK Engineering URI\n");
	printf("USB Radio Interface, version 1.0, 09/28/2023\n\n");

	usb_dev = device_init();
	if (usb_dev == NULL) {
		fprintf(stderr, "Device not found\n");
		exit(255);
	}
	usb_handle = usb_open(usb_dev);
	if (usb_handle == NULL) {
		fprintf(stderr, "Not able to open USB device\n");
		goto exit;
	}
	if (usb_claim_interface(usb_handle, 3) < 0) {
		if (usb_detach_kernel_driver_np(usb_handle, 3) < 0) {
			goto exit;
		}
		if (usb_claim_interface(usb_handle, 3) < 0) {
			goto exit;
		}
	}

	setout(usb_handle, 8);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&sthread, &attr, soundthread, NULL);

	usleep(500000);

	tcgetattr(fileno(stdin), &t0);
	for (;;) {
		char str[80];
		int errs = 0;

		tcsetattr(fileno(stdin), TCSANOW, &t0);
		myfreq = 0.0;
		myfreq1 = 0.0;
		myfreq2 = 0.0;
		printf("Menu:\r\n\n");
		printf("For Left Channel:\n");
		printf("1 - 1004Hz, 2 - 204Hz, 3 - 300Hz, 4 - 404Hz, 5 - 502Hz\n");
		printf("6 - 1502Hz, 7 - 2004Hz, 8 - 3004Hz, 9 - 5004Hz\n");
		printf("For Right Channel:\n");
		printf("11 - 1004Hz, 22 - 204Hz, 33 - 300Hz, 44 - 404Hz, 55 - 502Hz\n");
		printf("66 - 1502Hz, 77 - 2004Hz, 88 - 3004Hz, 99 - 5004Hz\n");
		printf("Tests....\n");
		printf("t - test normal operation (use uppercase 'T' for verbose output)\n");
		printf("i - test digital signals only (COR,TONE,PTT,GPIO)\n");
		printf("e - test EEPROM, E - Initialize EEPROM\n");
		printf("l - list EEPROM contents\n");
		printf("d - dump EEPROM contents\n");
		printf("m - list manufacturer settings, M - write manufacturer settings (CM119B)\n");
		printf("c - show test (loopback) connector pinout\n");
		printf("q,x - exit program\r\n\n");
		printf("Enter your selection: ");
		fflush(stdout);
		
		fgets(str, sizeof(str) - 1, stdin);
		c = str[0];
		if (isupper(c)) {
			c = tolower(str[0]);
		}
		switch (c) {
		case 'x':
		case 'q':
			goto exit;
		case '1':
			myfreq = 1004.0;
			break;
		case '2':
			myfreq = 204.0;
			break;
		case '3':
			myfreq = 300.0;
			break;
		case '4':
			myfreq = 404.0;
			break;
		case '5':
			myfreq = 502.0;
			break;
		case '6':
			myfreq = 1502.0;
			break;
		case '7':
			myfreq = 2004.0;
			break;
		case '8':
			myfreq = 3004.0;
			break;
		case '9':
			myfreq = 5004.0;
			break;
		case 0:
		case '\n':
		case '\r':
			myfreq = 0;
			break;
		case 'i':
			digital_test(usb_handle);
			continue;
		case 't':
		case 'T':
			errs = digital_test(usb_handle);
			errs += analog_test(str[0] == 'T');
			if (!errs)
				printf("System Tests all Passed successfully!\n");
			else
				printf("%d Error(s) found during test(s)!\n", errs);
			printf("\n\n");
			continue;
		case 'e':
		case 'E':
			if (str[0] == 'E') {
				eeprom_init(usb_handle);
				printf("\nEEPROM Initialized\n\n");
				continue;
			}
			printf("\n\n");
			errs = eeprom_test(usb_handle);
			if (!errs) {
				printf("EEPROM test Passed successfully!\n");
			} else {
				printf("%d Error(s) found during test(s)!\n", errs);
			}
			printf("\n\n");
			continue;
		case 'l':
			printf("\n");
			errs = eeprom_list(usb_handle);
			if (!errs) {
				printf("EEPROM list successful!\n");
			} else {
				printf("%d Error(s) found during operation!\n", errs);
			}
			printf("\n");
			continue;
		case 'd':
			printf("\n");
			eeprom_dump(usb_handle);
			printf("\n");
			continue;
		case 'm':
			printf("\n");
			eeprom_list_manufacturer(usb_handle);
			printf("\n");
			continue;
		case 'M':
			if (devproductid != C119B_PRODUCT_ID) {
				printf("This option is only for the CM-119B chip.\n");
				continue;
			}
			put_eeprom_mfg_data(usb_handle);
			printf("\n");
			continue;
		case 'c':				/* show test cable pinout */
			printf("Special Test Cable Pinout:\n\n");
			printf("25 pin D-shell connector\n");
			printf("  Pin 1 to Pin 7\n");
			printf("  Pin 2 to Pin 3\n");
			printf("  Pin 4 to Pin 8\n");
			printf("  If (current) URIx:\n");
			printf("     Pin 23 to Pin 24\n");
			printf("  Or, if (original) URI:\n");
			printf("     Pin 11 to Pin 24\n");
			printf("  10K Resistor between Pins 21 & 22\n");
			printf("  10K Resistor between Pins 21 & 23\n");
			printf("  For URIx (CM119), also include the following:\n");
			printf("  Pin 5 to Pin 10\n");
			printf("  Pin 6 to Pin 11\n");
			printf("  Do *NOT* include these for the standard URI!!\n\n");
			continue;
		default:
			continue;
		}
		
		if ((strlen(str) > 1) && (str[0] == str[1])) {
			myfreq2 = myfreq;
		} else {
			myfreq1 = myfreq;
		}
		
		tcgetattr(fileno(stdin), &t);
		cfmakeraw(&t);
		t.c_lflag &= ~ICANON;
		tcsetattr(fileno(stdin), TCSANOW, &t);
		fcntl(fileno(stdin), F_SETFL, fcntl(fileno(stdin), F_GETFL) | O_NONBLOCK);
		for (;;) {
			int c = getc(stdin);
			if (c > 0) {
				break;
			}
			usleep(500000);
			printf("Level at %.1f Hz: %.1f mV (RMS) %.1f mV (P-P)\r\n", myfreq, lev,
				   lev * 2.828);
		}
		tcgetattr(fileno(stdin), &t);
		t.c_lflag &= ICANON;
		tcsetattr(fileno(stdin), TCSANOW, &t);
		fcntl(fileno(stdin), F_SETFL, fcntl(fileno(stdin), F_GETFL) & ~O_NONBLOCK);
	}
	
  exit:
	shutdown = 1;
	pthread_join(sthread, NULL);
	usb_close(usb_handle);
	
	return retval;
}

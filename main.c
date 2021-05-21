#include<stdio.h>
#include<stdlib.h>

#include "gc5035mipi_Sensor.h"
#include "camera_AE_PLineTable_gc5035mipiraw.h"
#include "OtpDumpInfo.h"


int main() {
	/*kal_uint16 Pline_index = 85;
	kal_uint16 shutter = sPreviewPLineTable_50Hz.sPlineTable[Pline_index].u4Eposuretime;
	kal_uint16 gain = sPreviewPLineTable_50Hz.sPlineTable[Pline_index].u4AfeGain;
	shutter = shutter / 16.667;
	gain = gain / 4;
	printf("shutter£º%d\n", shutter);
	printf("gain£º%d\n", gain);
	open();
	set_shutter(shutter);
	set_gain(gain);*/

	gc5035_gcore_read_dpc();
	gc5035_gcore_read_reg();
	gc5035_otp_read_module_info();

	return 0;
}

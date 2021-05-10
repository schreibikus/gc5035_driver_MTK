#include<stdio.h>
#include<stdlib.h>

#include "gc5035mipi_Sensor.h"
#include "camera_AE_PLineTable_gc5035mipiraw.h"

int main() {
	//printf("hello\n");
	//float sss;
	kal_uint16 Pline_index = 85;
	kal_uint16 shutter = sPreviewPLineTable_50Hz.sPlineTable[Pline_index].u4Eposuretime;
	kal_uint16 gain = sPreviewPLineTable_50Hz.sPlineTable[Pline_index].u4AfeGain;

	//shutter = (kal_uint16)(shutter / 16.667);
	//sss = (float)(shutter / 16.667);
	shutter = shutter / 16.667;
	gain = gain / 4;
	printf("shutter£º%d\n", shutter);
	printf("gain£º%d\n", gain);

	open();
	//set_dummy();
	set_shutter(shutter);
	set_gain(gain);
	//set_max_framerate(30 ,1);

	return 0;
}

#include<stdio.h>
#include<stdlib.h>

#include "gc5035mipi_Sensor.h"
#include "camera_AE_PLineTable_gc5035mipiraw.h"

int main() {
	printf("hello\n");
	kal_uint16 Pline_index = 135;
	kal_uint16 shutter = sPreviewPLineTable_50Hz.sPlineTable[Pline_index].u4Eposuretime;
	kal_uint16 gain = sPreviewPLineTable_50Hz.sPlineTable[Pline_index].u4AfeGain;
	printf("shutter£º%d\n", shutter);
	printf("gain£º%d\n", gain);

	open();
	//set_dummy();
	set_shutter(shutter);
	set_gain(gain);
	//set_max_framerate(30 ,1);

	return 0;
}

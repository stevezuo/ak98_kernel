#ifndef _AK98_SENSOR_H_
#define _AK98_SENSOR_H_

#define SENSOR_ACCELEROMETER	"1"
#define SENSOR_MAGNETIC_FIELD	"2"
#define SENSOR_ORIENTATION		"3"

struct sensor_t {
	char *name;
	char *vendor;
	char *type;
	char *maxRange;		/* maxRange/10 */
	char *resolution;	/* resolution/10000 */
	char *power;		/* power/1000 A */
	char *dir;
	int exist;
};

struct sensor_platform_data {
	struct sensor_t *sensors;
};

#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <mach/ak_sensor.h>

struct sensor_kobj {
	struct kobject kobj;
	struct sensor_t *sensor;
};

struct sensor_attribute {
	struct attribute attr;
	ssize_t (*show)(struct sensor_kobj *kobj, struct sensor_attribute *attr,
			char *buf);
	ssize_t (*store)(struct sensor_kobj *kobj, struct sensor_attribute *attr,
			 const char *buf, size_t count);
};

static struct kset *sensor_kset =NULL;
static struct sensor_kobj *gsensor_kobj = NULL;
static struct sensor_kobj *msensor_kobj = NULL;
static struct sensor_kobj *osensor_kobj = NULL;

#define SENSOR_ATTR(_type, _name, _mode, _show, _store) 				\
struct sensor_attribute attr_##_type##_##_name = __ATTR(_name, _mode, _show, _store)

#define SENSOR_SYSFS(type, name)										\
static ssize_t show_##type##_##name(struct sensor_kobj *kobj,			\
				struct sensor_attribute *attr, char *buf)				\
{																		\
	ssize_t ret = 0;													\
																		\
	sprintf(buf, "%s", kobj->sensor->name); 							\
	ret = strlen(buf) + 1;												\
																		\
	return ret;															\
}																		\
static SENSOR_ATTR(type, name, S_IRUGO, show_##type##_##name, NULL);	

SENSOR_SYSFS(gsensor, name);
SENSOR_SYSFS(gsensor, vendor);
SENSOR_SYSFS(gsensor, type);
SENSOR_SYSFS(gsensor, maxRange);
SENSOR_SYSFS(gsensor, resolution);
SENSOR_SYSFS(gsensor, power);
SENSOR_SYSFS(gsensor, dir);

SENSOR_SYSFS(msensor, name);
SENSOR_SYSFS(msensor, vendor);
SENSOR_SYSFS(msensor, type);
SENSOR_SYSFS(msensor, maxRange);
SENSOR_SYSFS(msensor, resolution);
SENSOR_SYSFS(msensor, power);
SENSOR_SYSFS(msensor, dir);

SENSOR_SYSFS(osensor, name);
SENSOR_SYSFS(osensor, vendor);
SENSOR_SYSFS(osensor, type);
SENSOR_SYSFS(osensor, maxRange);
SENSOR_SYSFS(osensor, resolution);
SENSOR_SYSFS(osensor, power);

static struct attribute *gsensor_attributes[] = {
	&attr_gsensor_name.attr,
	&attr_gsensor_vendor.attr,
	&attr_gsensor_type.attr,
	&attr_gsensor_maxRange.attr,
	&attr_gsensor_resolution.attr,
	&attr_gsensor_power.attr,
	&attr_gsensor_dir.attr,
	NULL
};

static struct attribute *msensor_attributes[] = {
	&attr_msensor_name.attr,
	&attr_msensor_vendor.attr,
	&attr_msensor_type.attr,
	&attr_msensor_maxRange.attr,
	&attr_msensor_resolution.attr,
	&attr_msensor_power.attr,
	&attr_msensor_dir.attr,
	NULL
};

static struct attribute *osensor_attributes[] = {
	&attr_osensor_name.attr,
	&attr_osensor_vendor.attr,
	&attr_osensor_type.attr,
	&attr_osensor_maxRange.attr,
	&attr_osensor_resolution.attr,
	&attr_osensor_power.attr,
	NULL
};

static ssize_t sensor_attr_show(struct kobject *kobj, 
						struct attribute *attr,
						char *buf)
{
	struct sensor_attribute *attribute;
	struct sensor_kobj *sensor;

	attribute = container_of(attr, struct sensor_attribute, attr);
	sensor = container_of(kobj, struct sensor_kobj, kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(sensor, attribute, buf);
}					
											
static struct sysfs_ops sensor_sysfs_ops = {
	.show = sensor_attr_show,
	.store = NULL,
};

static void sensor_release(struct kobject *kobj)
{
	struct sensor_kobj *sensor;

	sensor = container_of(kobj, struct sensor_kobj, kobj);
	kfree(sensor);
}

static struct kobj_type gsensor_ktype = {
	.sysfs_ops		= &sensor_sysfs_ops,
	.release		= sensor_release,
	.default_attrs	= gsensor_attributes,
}; 

static struct kobj_type msensor_ktype = {
	.sysfs_ops		= &sensor_sysfs_ops,
	.release		= sensor_release,
	.default_attrs	= msensor_attributes,
}; 

static struct kobj_type osensor_ktype = {
	.sysfs_ops		= &sensor_sysfs_ops,
	.release		= sensor_release,
	.default_attrs	= osensor_attributes,
}; 


static struct sensor_kobj *create_sensor_kobj(const char *name, struct kobj_type *ktype)
{
	struct sensor_kobj *sensor;
	int ret;

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return NULL;

	sensor->kobj.kset = sensor_kset;

	ret = kobject_init_and_add(&sensor->kobj, ktype, NULL, "%s", name);
	if (ret) {
		kobject_put(&sensor->kobj);
		kfree(sensor);
		return NULL;
	}

	kobject_uevent(&sensor->kobj, KOBJ_ADD);

	return sensor;
}

static int __init ak_sensor_probe(struct platform_device *pdev)
{
	int i;
	struct sensor_platform_data *pdata = NULL;
	struct sensor_t *sensor = NULL; 

	pdata = pdev->dev.platform_data;
	sensor = pdata->sensors;

	sensor_kset = kset_create_and_add("sensor_list", NULL, &pdev->dev.kobj);
	if (!sensor_kset) {
		printk("failed to create sensor kset\n");
		return -ENOMEM;
	}

	for(i = 0; ; i++) {
		if (sensor[i].name == NULL) 
			break;

		if (sensor[i].exist == 0) 
			continue;

		if (!strcmp(sensor[i].type, SENSOR_ACCELEROMETER)) 
		{
			gsensor_kobj = create_sensor_kobj("gsensor", &gsensor_ktype);
			if (!gsensor_kobj) 
				printk("failed to create gsensor kobject\n");
			else  
				gsensor_kobj->sensor = sensor + i;
		} 
		else if (!strcmp(sensor[i].type, SENSOR_MAGNETIC_FIELD)) 
		{
			msensor_kobj = create_sensor_kobj("msensor", &msensor_ktype);
			if (!msensor_kobj) 
				printk("failed to create msensor kobject\n");
			else 
				msensor_kobj->sensor = sensor + i;
		} 
		else if (!strcmp(sensor[i].type, SENSOR_ORIENTATION)) 
		{
			if (gsensor_kobj && msensor_kobj) {
				osensor_kobj = create_sensor_kobj("osensor", &osensor_ktype);
				if (!osensor_kobj)
					printk("failed to create osensor kobject\n");
				else 
					osensor_kobj->sensor = sensor + i;
			}
		}
	}

	return 0;
}

static int ak_sensor_remove(struct platform_device *pdev)
{
	if (gsensor_kobj)
		kobject_put(&gsensor_kobj->kobj);
	if (msensor_kobj)
		kobject_put(&msensor_kobj->kobj);
	if (osensor_kobj)
		kobject_put(&osensor_kobj->kobj);
	kset_unregister(sensor_kset);

	return 0;
}

static struct platform_driver ak_sensor_driver = {
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "ak_sensor",
	},
	.probe		= ak_sensor_probe,
	.remove		= ak_sensor_remove,
};

static int __init ak_sensor_init(void)
{
	return platform_driver_register(&ak_sensor_driver);
}

static void __exit ak_sensor_exit(void)
{
	platform_driver_unregister(&ak_sensor_driver);
}

module_init(ak_sensor_init);	
module_exit(ak_sensor_exit);	

MODULE_DESCRIPTION("Anyka Sensor Driver");
MODULE_AUTHOR("Anyka");
MODULE_LICENSE("GPL");

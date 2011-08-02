#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include <mach/ts.h>
#include <mach/adc1.h>
#include <mach/bat.h>


/* BATTERY DEFINE*/
#define AK98_BAT_DELAY					(HZ * 1)		/* delay 1s */
#define AK98_DELAY_MINUTE				(1)				/* time to update charge capacity */
#define AK98_CHARGE_DELAY				(HZ * 60 * AK98_DELAY_MINUTE)		
#define AK98_USB_DELAY  				100				/* delay 100ms */
#define AK98_AC_DELAY   				100				/* delay 100ms */

#define PK(fmt...) 		  		//printk(fmt)  // debug ac usb interrupt and read ad4 voltage
#define PK_SAMPLE(fmt...) 	 	//printk(fmt)  //debug read voltage sample
#define PK_CHARGE(fmt...)  		//printk(fmt)  //debug charge 
#define PK_DISCHARGE(fmt...)  	//printk(fmt) //debug discharge

static void ak98usb_timer_handler(unsigned long data);
static void ak98ac_timer_handler(unsigned long data);
static void charge_timer_handler(unsigned long data);
static void voltage_timer_handler(unsigned long data);
static void ak98_update_bat_state(void);
static void ak98_update_ac_state(void);
static void ak98_update_usb_state(void);
static void ak98_bat_data_init(void);



struct battery_data{
	int		bat_voltage;			// battery read voltage
	int		bat_capacity;			// battery capacity
	int		bat_status;				// battery charge status
	int		usb_status;				// battery charge usb status
	int		ac_status;				// ac online status
	int		charge_time;			// charge time
};

struct read_voltage_sample{
	int 	voltage_index;			// voltage[6] index
	int 	voltage_max;			// sample max voltage
	int 	voltage_min;			// sample min voltage
	int		voltage_sum;			// sum of read voltage
};

struct ak98_bat{
	struct ak98_bat_mach_info *pdata;
	struct battery_data batdata;
	struct read_voltage_sample voltage;
};

static struct ak98_bat bat_main ={
	.pdata	= NULL,
};

static struct timer_list ak98usb_timer			= TIMER_INITIALIZER(ak98usb_timer_handler, 0, 0);
static struct timer_list ak98ac_timer   		= TIMER_INITIALIZER(ak98ac_timer_handler, 0, 0);
static struct timer_list update_voltage_timer	= TIMER_INITIALIZER(voltage_timer_handler, 0, 0);
static struct timer_list update_charge_timer 	= TIMER_INITIALIZER(charge_timer_handler, 0, 0);

	
/**
 * @brief get adc1 ad4 value. if input voltage from 0 to AVDD, it will return the value from 0 to 1023 
 * @author  
 * @date 
 * @return int
 */
static int ak98_getbat_voltage(void)
{	
    int ad4_value = 0;
	unsigned int count = 0;

	PK("##################%s:\n",__func__);
	
   	//open adc1 and juge if touch scheen is reading.
	ak98_power_ADC1(POWER_ON);
    //enable ad4
    ak98_enable_bat_mon();	
	        
	//sample 5 times
	count = 5;
    while ( count-- )
    {
    	
        mdelay(1);							//delay for get next point ad4
        ad4_value += ak98_read_voltage();	//0x3ff for 10 bit adc

	}

	//disable ad4
    ak98_disable_bat_mon();
	//power off adc1
	ak98_power_ADC1(POWER_OFF);   

	//caculate battery value
	ad4_value =	ad4_value / 5; 							// average of read voltage
	PK("ad4_data = %d:\n",ad4_value);
	
	ad4_value = (ad4_value * bat_main.pdata->bat_adc.adc_avdd) / (1024);	// real read voltage
	PK("ad4_voltage = %d:\n",ad4_value);

	ad4_value = (ad4_value * (bat_main.pdata->bat_adc.up_resistance + bat_main.pdata->bat_adc.dw_resistance))
				/ bat_main.pdata->bat_adc.dw_resistance;
	PK("bat_voltage = %d:\n",ad4_value);
		
	ad4_value += bat_main.pdata->bat_adc.voltage_correct;				// correct battery voltage
	PK("correct_voltage = %d:\n",ad4_value);
		    
    return ad4_value;
}

static void bat_set_int_inverse(unsigned int pin)
{

	if (ak98_gpio_getpin(pin) == AK98_GPIO_HIGH)
	{
		ak98_gpio_intpol(pin, AK98_GPIO_INT_LOWLEVEL);	
	}
	else
	{
		ak98_gpio_intpol(pin, AK98_GPIO_INT_HIGHLEVEL);
	}

}


// battery power supply define
static int ak98_battery_get_property(struct power_supply *bat_ps,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	ak98_update_bat_state();

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_main.batdata.bat_status;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat_main.batdata.bat_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = bat_main.pdata->bat_mach_info.max_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = bat_main.pdata->bat_mach_info.min_voltage;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 25;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = bat_main.batdata.charge_time * 60;
		break;		
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_main.batdata.bat_capacity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void ak98_battery_external_power_changed(struct power_supply *bat_ps)
{
	// NULL

}


static enum power_supply_property ak98_battery_main_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

struct power_supply bat_ps = {
	.name					= "battery",
	.type					= POWER_SUPPLY_TYPE_BATTERY,
	.properties				= ak98_battery_main_props,
	.num_properties			= ARRAY_SIZE(ak98_battery_main_props),
	.get_property			= ak98_battery_get_property,
	.external_power_changed = ak98_battery_external_power_changed,
	.use_for_apm			= 1,
};

#ifdef ANYKA_BATTERY_DEBUG
static char *status_text[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN] =		"Unknown",
	[POWER_SUPPLY_STATUS_CHARGING] =	"Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING] =	"Discharging",
};
#endif



// battery power supply define end


// usb power supply define
static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	

	ak98_update_usb_state();	
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bat_main.batdata.usb_status;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct power_supply usb_ps ={
	.name				= "usb",
	.type				= POWER_SUPPLY_TYPE_USB,
	.properties 		= power_props,
	.num_properties 	= ARRAY_SIZE(power_props),
	.get_property		= usb_get_property,
};
// usb power supply define end!


// ac power supply
static int ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	
	ak98_update_ac_state();	
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bat_main.batdata.ac_status;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}


static enum power_supply_property ac_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct power_supply ac_ps ={
	.name				= "ac",
	.type				= POWER_SUPPLY_TYPE_MAINS,
	.properties 		= ac_power_props,
	.num_properties 	= ARRAY_SIZE(ac_power_props),
	.get_property		= ac_get_property,
};
// ac power supply define end!



#ifdef CONFIG_PM
static int ak98_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int ak98_battery_resume(struct platform_device *dev)
{
	ak98_bat_data_init();	
	power_supply_changed(&bat_ps);
	power_supply_changed(&ac_ps);
	power_supply_changed(&usb_ps);	
	return 0;
}
#else
#define ak98_battery_suspend NULL
#define ak98_battery_resume NULL
#endif

/*
*   
*  return array_y value
*/
static int get_arrayy_from_arrayx(const int array_x[],const int array_y[],
								  int array_nums, int x_num )
{
	int x0, y0;
	int x1, y1;
	int n;
	int num = x_num;
	int ret = 0;

	PK_DISCHARGE("array_x:%4d,%4d,%4d,%4d,%4d\n",array_x[0],array_x[1],
		array_x[2],array_x[3],array_x[4]);
	PK_DISCHARGE("array_y:%4d,%4d,%4d,%4d,%4d\n",array_y[0],array_y[1],
		array_y[2],array_y[3],array_y[4]);
	PK_DISCHARGE("array_nums=%d;x_num=%d\n",array_nums,x_num);
	PK_DISCHARGE("length=%d;points=%d\n",bat_main.pdata->discharge.length
		,bat_main.pdata->discharge.points);
	
	if (num < array_x[0])
	{
		num = array_x[0];
	}

	if (num > array_x[array_nums-1])
	{
		num = array_x[array_nums-1];
	}

	PK_DISCHARGE("array_nums=%d;num=%d\n",array_nums,num);
	

	for (n=1; n<array_nums; n++)
	{
		if (num <= array_x[n])
		{
			x0 = array_x[n-1];
			y0 = array_y[n-1];
			x1 = array_x[n];
			y1 = array_y[n];
			ret = ((num -x0) * (y1 - y0)) / (x1 - x0) + y0;
			PK_DISCHARGE("ret=%d;\n",ret);
			break;
		}
	}
	
	return ret;
}


//caculate charge time from capacity,modify ak98_bat_data.charge_time
static void get_charge_time_from_cap(int bat_capacity)
{
	bat_main.batdata.charge_time = get_arrayy_from_arrayx(bat_main.pdata->charge.capacity,
										bat_main.pdata->charge.time,
										bat_main.pdata->charge.points, bat_capacity);
}

//caculate capacity from charge time,modify ak98_bat_data.bat_capacity
static void get_charge_cap_from_time(int time)
{
	bat_main.batdata.bat_capacity = get_arrayy_from_arrayx(bat_main.pdata->charge.time,
										bat_main.pdata->charge.capacity,
										bat_main.pdata->charge.points, time);
}

//caculate capacity from charge voltage,modify bat_data.bat_capacity
static void get_charge_cap_from_voltage(int voltage)
{
	bat_main.batdata.bat_capacity = get_arrayy_from_arrayx(bat_main.pdata->charge_cv.voltage
								,bat_main.pdata->charge_cv.capacity
								,bat_main.pdata->charge_cv.points, voltage);
}


//caculate capacity from discharge voltage,modify bat_data.bat_capacity
static void get_discharge_cap_from_voltage(int voltage)
{
	bat_main.batdata.bat_capacity= get_arrayy_from_arrayx(bat_main.pdata->discharge.voltage,
										bat_main.pdata->discharge.capacity,
										bat_main.pdata->discharge.points, voltage);
}

static void ak98_add_battime_min(int n)
{
	bat_main.batdata.charge_time += n;
	
	if (bat_main.batdata.charge_time < 0)
	{
		bat_main.batdata.charge_time =0;		
	}
	
	if (bat_main.batdata.charge_time >= bat_main.pdata->bat_mach_info.charge_max_time)
	{
		bat_main.batdata.charge_time = bat_main.pdata->bat_mach_info.charge_max_time - 1;
	}
}

static void ak98_init_sample_value(void)
{
	bat_main.voltage.voltage_index		= 0;		
	bat_main.voltage.voltage_sum		= 0;
	bat_main.voltage.voltage_max	  	= 0;
	bat_main.voltage.voltage_min	  	= bat_main.pdata->bat_mach_info.max_voltage * 2;
}

static void ak98_update_voltage(void)
{
	int temp;
	
	PK_SAMPLE("\n###############%s; \n",__func__);

	// if voltage_index in correct range
	if ((bat_main.voltage.voltage_index > bat_main.pdata->bat_mach_info.voltage_sample) 
		|| (bat_main.voltage.voltage_index < 0))		
	{
		printk("%s:error voltage sample index\n",__func__);
		ak98_init_sample_value();
	}
	
	// read voltage sample,and save voltage max and min;
	temp = ak98_getbat_voltage();
	if (temp > bat_main.voltage.voltage_max)
	{
		bat_main.voltage.voltage_max = temp;	
	}

	if (temp < bat_main.voltage.voltage_min)
	{
		bat_main.voltage.voltage_min = temp;
	}
	
	PK_SAMPLE("read_voltage=%d\n",temp);
	PK_SAMPLE("voltage_max=%d;	voltage_min=%d;\n",
			  bat_main.voltage.voltage_max,bat_main.voltage.voltage_min);
	PK_SAMPLE("voltage_index=%d; voltage_sum=%d;\n",bat_main.voltage.voltage_index,
				bat_main.voltage.voltage_sum);
	
	bat_main.voltage.voltage_sum += temp;
	
	if (++bat_main.voltage.voltage_index == bat_main.pdata->bat_mach_info.voltage_sample)
	{
		bat_main.voltage.voltage_sum	 -= bat_main.voltage.voltage_min 
											+ bat_main.voltage.voltage_max;
		bat_main.voltage.voltage_index -= 2;
		bat_main.batdata.bat_voltage	= bat_main.voltage.voltage_sum 
										/ bat_main.voltage.voltage_index;

		PK_SAMPLE("=========bat_data.voltage = %d;=======\n",bat_main.batdata.bat_voltage);
		ak98_init_sample_value();
	}
	
}

static void update_discharge_capacity(void)
{
	ak98_update_bat_state();
	if (bat_main.batdata.bat_status == POWER_SUPPLY_STATUS_DISCHARGING)
	{
		int old_capacity = bat_main.batdata.bat_capacity;		

		PK_DISCHARGE("###########%s:\n",__func__);
		
		get_discharge_cap_from_voltage(bat_main.batdata.bat_voltage);

		if (bat_main.batdata.bat_capacity > old_capacity)
		{
			bat_main.batdata.bat_capacity = old_capacity;
		}
		
		if (bat_main.batdata.bat_capacity <= bat_main.pdata->bat_mach_info.power_down_level)
		{
			bat_main.batdata.bat_capacity = 0;
		}

		if (old_capacity != bat_main.batdata.bat_capacity)
		{
			get_charge_time_from_cap(bat_main.batdata.bat_capacity);
			power_supply_changed(&bat_ps);
		}

		PK_DISCHARGE("voltage = %d;capacity = %d;\n",
					 bat_main.batdata.bat_voltage,bat_main.batdata.bat_capacity);
	}
}

static void ak98usb_timer_handler(unsigned long data)
{	
	PK("##################%s:\n", __func__);
	
	power_supply_changed(&usb_ps);	
	bat_set_int_inverse(bat_main.pdata->usb_gpio.pindata.pin);	
	enable_irq(bat_main.pdata->usb_gpio.irq);
}

static void ak98ac_timer_handler(unsigned long data)
{
	PK("##################%s:\n", __func__);
	
	power_supply_changed(&bat_ps);
	power_supply_changed(&ac_ps);
	
	bat_set_int_inverse(bat_main.pdata->ac_gpio.pindata.pin);	
	enable_irq(bat_main.pdata->ac_gpio.irq);
}

static int read_pin_is_active(int pin, int active)
{
	if (pin >= 0)
	{
		return (ak98_gpio_getpin((unsigned int)pin)) == active ? AK_TRUE : AK_FALSE;
	}
	else
	{
		return AK_FALSE; 
	}	


}

// battery from not full to full
static void ak98_capacity_is_full(void)
{
	static int full_count = 1;
	
	ak98_update_bat_state();
	if ((bat_main.batdata.bat_status == POWER_SUPPLY_STATUS_CHARGING)
		&& (bat_main.batdata.bat_capacity != bat_main.pdata->bat_mach_info.full_capacity)
		&& read_pin_is_active(bat_main.pdata->state_gpio.pindata.pin,bat_main.pdata->state_gpio.active)) 
	{
		if (++full_count > 10)
		{
			// full_count reach 10, battery capacity is real full
			full_count = 1;
			bat_main.batdata.bat_capacity = bat_main.pdata->bat_mach_info.full_capacity; 		
			get_charge_time_from_cap(bat_main.batdata.bat_capacity);
			PK_CHARGE("###############%s:\n",__func__);
		}			
	}
	else
	{
		full_count = 1;
	}
	
}

// battery from full to not full
static void ak98_capacity_isnot_full(void)
{
	ak98_update_bat_state();
	if ((bat_main.batdata.bat_status == POWER_SUPPLY_STATUS_CHARGING)
		&& (bat_main.batdata.bat_capacity == bat_main.pdata->bat_mach_info.full_capacity)
		&& !read_pin_is_active(bat_main.pdata->state_gpio.pindata.pin,bat_main.pdata->state_gpio.active)) 
	{
		// battery capacity is not full
		bat_main.batdata.bat_capacity = bat_main.pdata->bat_mach_info.full_capacity - 1;
		get_charge_time_from_cap(bat_main.batdata.bat_capacity);
		PK_CHARGE("###############%s:\n",__func__);
	}		
}

static void voltage_timer_handler(unsigned long data)
{
	PK_DISCHARGE("###############%s:\n",__func__);
	
	ak98_update_voltage();
	if (0 == bat_main.voltage.voltage_index)
	{
		update_discharge_capacity();
		PK_DISCHARGE("voltage=%d; time=%d;capacity=%d;\n",
					 bat_main.batdata.bat_voltage,bat_main.batdata.charge_time,
					 bat_main.batdata.bat_capacity);
	}
	
	mod_timer(&update_voltage_timer,jiffies + AK98_BAT_DELAY);
}

static void update_charge_capacity(void)
{
	int old_capacity = bat_main.batdata.bat_capacity;
	
	ak98_update_bat_state();
	if (bat_main.batdata.bat_status == POWER_SUPPLY_STATUS_CHARGING)
	{
		PK_CHARGE("###############%s:\n",__func__);
	
		if ( bat_main.pdata->bat_mach_info.full_capacity != bat_main.batdata.bat_capacity )
		{
			ak98_add_battime_min(AK98_DELAY_MINUTE);
			get_charge_cap_from_time(bat_main.batdata.charge_time);
			ak98_capacity_is_full();		
		}
		else
		{
			ak98_capacity_isnot_full();			
		}

		if (bat_main.batdata.bat_capacity != old_capacity)
		{
			power_supply_changed(&bat_ps);
		}
					
		PK_CHARGE("voltage = %d; time = %d; capacity = %d;\n",
		  bat_main.batdata.bat_voltage,bat_main.batdata.charge_time
		  ,bat_main.batdata.bat_capacity);
	}	  
}

static void charge_timer_handler(unsigned long data)
{
	PK_CHARGE("###############%s:\n",__func__);

	update_charge_capacity();
	mod_timer(&update_charge_timer,jiffies + AK98_CHARGE_DELAY);
//	mod_timer(&update_charge_timer,jiffies + HZ * 1);
}

static int ak98_get_charge_state(int pinstate,int active)
{
	if (pinstate >= 0)
	{
		return (ak98_gpio_getpin((unsigned int)pinstate)) == active ? 
			   POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else
	{
		return POWER_SUPPLY_STATUS_UNKNOWN; 
	}	
}

static void ak98_update_bat_state(void)
{
	bat_main.batdata.bat_status = ak98_get_charge_state(bat_main.pdata->ac_gpio.pindata.pin
									,bat_main.pdata->ac_gpio.active);	
}

static void ak98_update_ac_state(void)
{
	bat_main.batdata.ac_status = ak98_get_charge_state(bat_main.pdata->ac_gpio.pindata.pin
									,bat_main.pdata->ac_gpio.active);	
}

static void ak98_update_usb_state(void)
{
	if (bat_main.pdata->usb_gpio.irq > 0)
	{
		bat_main.batdata.usb_status = ak98_get_charge_state(bat_main.pdata->usb_gpio.pindata.pin
										,bat_main.pdata->usb_gpio.active);
	}
	else
	{
		bat_main.batdata.usb_status = POWER_SUPPLY_STATUS_DISCHARGING;				
	}
}

static irqreturn_t ak98bat_ac_irqhandler(int irq, void *handle)
{

	PK("##################%s:\n", __func__);

	disable_irq_nosync(irq);
	mod_timer(&ak98ac_timer,jiffies + msecs_to_jiffies(AK98_AC_DELAY));
	
	return IRQ_HANDLED;	
}

static irqreturn_t ak98bat_usb_irqhandler(int irq, void *handle) 
{

	PK("##################%s:\n", __func__);

	disable_irq_nosync(irq);
	mod_timer(&ak98usb_timer,jiffies + msecs_to_jiffies(AK98_USB_DELAY));
	
	return IRQ_HANDLED;		
}

static void ak98_bat_data_init(void)
{
	ak98_update_bat_state();
	ak98_update_ac_state();
	ak98_update_usb_state();
	
	bat_main.batdata.bat_voltage = ak98_getbat_voltage();
	if (bat_main.batdata.bat_status == POWER_SUPPLY_STATUS_CHARGING)
	{
		get_charge_cap_from_voltage(bat_main.batdata.bat_voltage);
	}
	else
	{
		get_discharge_cap_from_voltage(bat_main.batdata.bat_voltage);
	}
	
	get_charge_time_from_cap(bat_main.batdata.bat_capacity);
	ak98_capacity_isnot_full();
	ak98_init_sample_value();
}

static int __devinit ak98_battery_probe(struct platform_device *dev)
{
	int ret = 0;
	struct ak98_bat_mach_info *info;

	printk("%s:\n",__func__);

	info = (struct ak98_bat_mach_info *)dev->dev.platform_data;
	if (!info) 
	{
		printk(KERN_ERR "%s:no platform data for battery\n",__func__);
		ret = -EINVAL;
		goto out;
	}	

	bat_main.pdata	= info;

	// use for ac charge in irq
	if (info->ac_gpio.irq >= 0)
	{
		info->gpio_init(&info->ac_gpio.pindata);
		bat_set_int_inverse(info->ac_gpio.pindata.pin);	
		if (request_irq(info->ac_gpio.irq, ak98bat_ac_irqhandler, 0, "ac_charge", dev))
		{
			printk(KERN_ERR "%s:Could not allocate IRQ %d\n", __func__,info->ac_gpio.irq);
			ret = -EIO;
			goto out;
		}
	}
	
	// use for usb charge in irq
	if (info->usb_gpio.irq >= 0)
	{
		info->gpio_init(&info->usb_gpio.pindata);
		bat_set_int_inverse(info->usb_gpio.pindata.pin);	
		if (request_irq(info->usb_gpio.irq, ak98bat_usb_irqhandler, 0, "usb_charge", dev))
		{
			printk(KERN_ERR "%s:Could not allocate IRQ %d\n", __func__,info->usb_gpio.irq);
			ret = -EIO;
			goto free_acirq_out;
		}
	}

	// use for charge full state 
	if (info->state_gpio.pindata.pin >= 0)
	{
		info->gpio_init(&info->state_gpio.pindata);
	}
	
	/* initialize hardware */
	ak98_init_ADC1(info->bat_adc.sample_rate, info->bat_adc.wait_time);
	ak98_bat_data_init();
	
	mod_timer(&update_voltage_timer,jiffies + AK98_BAT_DELAY);
	mod_timer(&update_charge_timer,jiffies + AK98_CHARGE_DELAY);
		
	ret = power_supply_register(&dev->dev, &bat_ps);
	if (ret != 0)
	{
		goto free_out;
	}
	
	ret = power_supply_register(&dev->dev,&usb_ps);
	if (ret != 0)
	{
		goto free_bat_ps_out;
	}
	
	ret = power_supply_register(&dev->dev,&ac_ps);
	if (ret != 0)
	{
		goto free_usb_ps_out;
	}

	
	return ret;
	
free_usb_ps_out:
	power_supply_unregister(&usb_ps);
free_bat_ps_out:
	power_supply_unregister(&bat_ps);
free_out:
	if (info->usb_gpio.irq > 0)
	{
		disable_irq(info->usb_gpio.irq);
		free_irq(info->usb_gpio.irq, dev);
	}
free_acirq_out:
	if (info->ac_gpio.irq > 0)
	{
		disable_irq(info->ac_gpio.irq);
		free_irq(info->ac_gpio.irq, dev);
	}
out:
	printk(KERN_ERR "###########%s:ERR out##########\n",__func__);
	return ret;


}

static int __devexit ak98_battery_remove(struct platform_device *dev)
{	
	struct ak98_bat_mach_info *info;
	
	info = (struct ak98_bat_mach_info *)dev->dev.platform_data;
	if (!info) {
		printk(KERN_ERR "no platform data for battery\n");
		return -EINVAL;
	}	

	if (info->ac_gpio.irq > 0)
	{
		disable_irq(info->ac_gpio.irq);
		free_irq(info->ac_gpio.irq, dev);
	}
	
	if (info->usb_gpio.irq > 0)
	{
		disable_irq(info->usb_gpio.irq);
		free_irq(info->usb_gpio.irq, dev);
	}
	
	power_supply_unregister(&bat_ps);
	power_supply_unregister(&usb_ps);
	power_supply_unregister(&ac_ps);
	return 0;
}

static struct platform_driver ak98_battery_driver = {
	.driver		= {
		.name	= "fake_battery",
	},
	.probe		= ak98_battery_probe,
	.remove		= __devexit_p(ak98_battery_remove),
	.suspend	= ak98_battery_suspend,
	.resume		= ak98_battery_resume,
};

static int __init ak98_battery_init(void)
{
	return platform_driver_register(&ak98_battery_driver);
}

static void __exit ak98_battery_exit(void)
{
	platform_driver_unregister(&ak98_battery_driver);
}

module_init(ak98_battery_init);
module_exit(ak98_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anyka <xxx@anyak.oa>");
MODULE_DESCRIPTION("Fake battery driver");

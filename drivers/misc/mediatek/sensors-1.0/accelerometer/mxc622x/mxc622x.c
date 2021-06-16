/* MXC622X motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
	 
//#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
	 
#include "accel.h"
#include "cust_acc.h"
#include "mxc622x.h"
	 /*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_MXC622X 150
	 /*----------------------------------------------------------------------------*/
#define DEBUG 1
	 /*----------------------------------------------------------------------------*/
	 //#define CONFIG_MXC622X_LOWPASS   /*apply low pass filter on output*/		 
#define SW_CALIBRATION
	 
	 /*----------------------------------------------------------------------------*/
#define MXC622X_AXIS_X          0
#define MXC622X_AXIS_Y          1
#define MXC622X_AXIS_Z          2
#define MXC622X_AXES_NUM        2
#define MXC622X_DATA_LEN        2

#define MXC622X_INIT_SUCC	(0)
#define MXC622X_INIT_FAIL	(-1)

#define MXC622X_DEV_NAME        "MXC622X"
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mxc622x_i2c_id[] = {{MXC622X_DEV_NAME,0},{}};
/*the adapter id will be available in customization*/
//static struct i2c_board_info __initdata i2c_mxc622x={ I2C_BOARD_INFO("MXC622X", MXC622X_I2C_SLAVE_ADDR>>1)};

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mxc622x_i2c_remove(struct i2c_client *client);
//static int mxc622x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int mxc622x_local_init(void);
static int mxc622x_remove(void);

static int mxc622x_suspend(struct i2c_client *client, pm_message_t msg);
static int mxc622x_resume(struct i2c_client *client);

//static int	s_nInitFlag = MXC622X_INIT_FAIL;
//add by major  for 35 
static struct acc_init_info  mxc622x_init_info =
{
	.name   = MXC622X_DEV_NAME,
	.init   = mxc622x_local_init,
	.uninit = mxc622x_remove,
};

static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};


/*----------------------------------------------------------------------------*/
typedef enum {
 ADX_TRC_FILTER  = 0x01,
 ADX_TRC_RAWDATA = 0x02,
 ADX_TRC_IOCTL	 = 0x04,
 ADX_TRC_CALI	 = 0X08,
 ADX_TRC_INFO	 = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
 u8  whole;
 u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
 struct scale_factor scalefactor;
 int	sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
 s16 raw[C_MAX_FIR_LENGTH][MXC622X_AXES_NUM];
 int sum[MXC622X_AXES_NUM];
 int num;
 int idx;
};
/*----------------------------------------------------------------------------*/
struct mxc622x_i2c_data {
 struct i2c_client *client;
 struct acc_hw hw;
 struct hwmsen_convert	 cvt;
 /*misc*/
 struct data_resolution *reso;
 atomic_t				 trace;
 atomic_t				 suspend;
 atomic_t				 selftest;
 atomic_t				 filter;
 s16					 cali_sw[MXC622X_AXES_NUM+1];

 /*data*/
 s8 					 offset[MXC622X_AXES_NUM+1];  /*+1: for 4-byte alignment*/
 s16					 data[MXC622X_AXES_NUM+1];

#if defined(CONFIG_MXC622X_LOWPASS)
 atomic_t				 firlen;
 atomic_t				 fir_en;
 struct data_filter 	 fir;
#endif 
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver mxc622x_i2c_driver = {
 .driver = {
	 .name		 = MXC622X_DEV_NAME,
	 .of_match_table = accel_of_match,
 },
 .probe 			 = mxc622x_i2c_probe,
 .remove			 = mxc622x_i2c_remove,
 .suspend			 = mxc622x_suspend,
 .resume			 = mxc622x_resume,
 .id_table = mxc622x_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *mxc622x_i2c_client = NULL;
static struct mxc622x_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static GSENSOR_VECTOR3D gsensor_gain;
//static char selftestRes[8]= {0}; 
	 
	 /*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(GSE_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static struct data_resolution mxc622x_data_resolution[1] = {
/* combination by {FULL_RES,RANGE}*/
 {{ 15, 6}, 64},   // dataformat +/-2g	in 8-bit resolution;  { 15, 6} = 15.6 = (2*2*1000)/(2^8);  64 = (2^8)/(2*2)		   
};
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc622x_offset_resolution = {{15, 6}, 64};
	 

static struct mutex g_gsensor_mutex;
//#define BMA222_I2C_GPIO_MODE
//#define BMA222_I2C_GPIO_MODE_DEBUG

#define MXC622X_ABS(a) (((a) < 0) ? -(a) : (a))

static int mxc622x_sqrt(int high, int low, int value)
{
	int med;
	int high_diff = 0;
	int low_diff = 0;
	
	if (value <= 0)
		return 0;

	high_diff = MXC622X_ABS(high * high - value);
	low_diff = MXC622X_ABS(low * low - value);
	
	while (MXC622X_ABS(high - low) > 1)
	{
		med = (high + low ) / 2;
		if (med * med > value)
		{
			high = med;
			high_diff = high * high - value;
		}
		else
		{
			low = med;
			low_diff = value - low * low;
		}
	}

	return high_diff <= low_diff ? high : low;
}

//add by major 
#define C_I2C_FIFO_SIZE     8
static int mxc622x_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2]={{0},{0}};
	
//	mutex_lock(&MC3XXX_i2c_mutex);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len =1;
	msgs[0].buf = &beg;
	
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len =len;
	msgs[1].buf = data;


	if (!client)
	{
//		mutex_unlock(&MC3XXX_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE)
	{
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
//		mutex_unlock(&MC3XXX_i2c_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2)
	{
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
		err = -EIO;
	}
	else
	{
		err = 0;
	}
//	mutex_unlock(&MC3XXX_i2c_mutex);
	return err;
}
static int mxc622x_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];
	err =0;
//	mutex_lock(&MC3XXX_i2c_mutex);
	
	if (!client)
	{
//		mutex_unlock(&MC3XXX_i2c_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE)
	{
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
//		mutex_unlock(&MC3XXX_i2c_mutex);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
	{
		buf[num++] = data[idx];
	}
	err = i2c_master_send(client, buf, num);
	if (err < 0)
	{
		GSE_ERR("send command error!!\n");
//		mutex_unlock(&MC3XXX_i2c_mutex);
		return -EFAULT;
	}
	else
	{
		err = 0;
	}
//	mutex_unlock(&MC3XXX_i2c_mutex);
	return err;

}	//add end

int cust_i2c_master_send(struct i2c_client *client, u8 *buf, u8 count)
{
	u8 slave_addr;
	u8 reg_addr;

	mutex_lock(&g_gsensor_mutex);
	
	slave_addr = MXC622X_I2C_SLAVE_ADDR ;	
	reg_addr = buf[0];

	mxc622x_i2c_write_block(client,reg_addr,&buf[1],count-1);

	mutex_unlock(&g_gsensor_mutex);

	return count;
}

int cust_i2c_master_read(struct i2c_client *client, u8 *buf, u8 count)
{
	u8 slave_addr;
	u8 reg_addr;

	slave_addr = MXC622X_I2C_SLAVE_ADDR ;	
	reg_addr = buf[0];
	mxc622x_i2c_read_block(client,reg_addr,&buf[0],count);

	return count;
}



int cust_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 buf[64] = {0};
	mutex_lock(&g_gsensor_mutex);
	buf[0] = addr;
	cust_i2c_master_read(client, buf, len);
	mutex_unlock(&g_gsensor_mutex);

	memcpy(data, buf, len);
	return 0;
}


/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int MXC622X_SetDataResolution(struct mxc622x_i2c_data *obj)
{
// int err;
// u8  dat, reso;

/*set g sensor dataresolution here*/

/*MXC622X only can set to 8-bit dataresolution, so do nothing in mxc622x driver here*/

/*end of set dataresolution*/



/*we set measure range from -2g to +2g in MXC622X_SetDataFormat(client, MXC622X_RANGE_2G), 
												 and set 10-bit dataresolution MXC622X_SetDataResolution()*/
												 
/*so mxc622x_data_resolution[0] set value as {{ 15, 6},64} when declaration, and assign the value to obj->reso here*/  

 obj->reso = &mxc622x_data_resolution[0];
 return 0;
 
/*if you changed the measure range, for example call: MXC622X_SetDataFormat(client, MXC622X_RANGE_4G), 
you must set the right value to mxc622x_data_resolution*/

}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadData(struct i2c_client *client, s16 data[MXC622X_AXES_NUM])
{
 struct mxc622x_i2c_data *priv = i2c_get_clientdata(client); 	   
 u8 addr = MXC622X_REG_DATAX0;
 u8 buf[MXC622X_DATA_LEN] = {0};
 int err = 0;
// int i;
// int tmp=0;
// u8 ofs[3];



 if(NULL == client)
 {
	 err = -EINVAL;
 }
 err = cust_hwmsen_read_block(client, addr, buf, 2);
 if(err)
 {
	 //printk("gsensor 11111\n");
	 GSE_ERR("error: %d\n", err);
 }
 else
 {
	 //printk("gsensor 222222\n");
	 data[MXC622X_AXIS_X] = (s16)buf[0];
	 data[MXC622X_AXIS_Y] = (s16)buf[1];

	if(data[MXC622X_AXIS_X]&0x80)
	 {
			 data[MXC622X_AXIS_X] = ~data[MXC622X_AXIS_X];
			 data[MXC622X_AXIS_X] &= 0xff;
			 data[MXC622X_AXIS_X]+=1;
			 data[MXC622X_AXIS_X] = -data[MXC622X_AXIS_X];
	 }
	 if(data[MXC622X_AXIS_Y]&0x80)
	 {
			 data[MXC622X_AXIS_Y] = ~data[MXC622X_AXIS_Y];
			 data[MXC622X_AXIS_Y] &= 0xff;
			 data[MXC622X_AXIS_Y]+=1;
			 data[MXC622X_AXIS_Y] = -data[MXC622X_AXIS_Y];
	 }


	 if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
	 {
		 GSE_LOG("[%08X %08X] => [%5d %5d]\n", data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y],
									data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y]);
	 }
#ifdef CONFIG_MXC622X_LOWPASS
	 if(atomic_read(&priv->filter))
	 {
		 if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
		 {
			 int idx, firlen = atomic_read(&priv->firlen);	 
			 if(priv->fir.num < firlen)
			 {				  
				 priv->fir.raw[priv->fir.num][MXC622X_AXIS_X] = data[MXC622X_AXIS_X];
				 priv->fir.raw[priv->fir.num][MXC622X_AXIS_Y] = data[MXC622X_AXIS_Y];

				 priv->fir.sum[MXC622X_AXIS_X] += data[MXC622X_AXIS_X];
				 priv->fir.sum[MXC622X_AXIS_Y] += data[MXC622X_AXIS_Y];

				 if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
				 {
					 GSE_LOG("add [%2d] [%5d %5d] => [%5d %5d]\n", priv->fir.num,
						 priv->fir.raw[priv->fir.num][MXC622X_AXIS_X], priv->fir.raw[priv->fir.num][MXC622X_AXIS_Y],
						 priv->fir.sum[MXC622X_AXIS_X], priv->fir.sum[MXC622X_AXIS_Y]);
				 }
				 priv->fir.num++;
				 priv->fir.idx++;
			 }
			 else
			 {
				 idx = priv->fir.idx % firlen;
				 priv->fir.sum[MXC622X_AXIS_X] -= priv->fir.raw[idx][MXC622X_AXIS_X];
				 priv->fir.sum[MXC622X_AXIS_Y] -= priv->fir.raw[idx][MXC622X_AXIS_Y];

				 priv->fir.raw[idx][MXC622X_AXIS_X] = data[MXC622X_AXIS_X];
				 priv->fir.raw[idx][MXC622X_AXIS_Y] = data[MXC622X_AXIS_Y];

				 priv->fir.sum[MXC622X_AXIS_X] += data[MXC622X_AXIS_X];
				 priv->fir.sum[MXC622X_AXIS_Y] += data[MXC622X_AXIS_Y];

				 priv->fir.idx++;
				 data[MXC622X_AXIS_X] = priv->fir.sum[MXC622X_AXIS_X]/firlen;
				 data[MXC622X_AXIS_Y] = priv->fir.sum[MXC622X_AXIS_Y]/firlen;

				 if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
				 {
					 GSE_LOG("add [%2d] [%5d %5d] => [%5d %5d] : [%5d %5d]\n", idx,
					 priv->fir.raw[idx][MXC622X_AXIS_X], priv->fir.raw[idx][MXC622X_AXIS_Y],
					 priv->fir.sum[MXC622X_AXIS_X], priv->fir.sum[MXC622X_AXIS_Y],
					 data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y]);
				 }
			 }
		 }
	 }	 
#endif         
 }
 return err;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadOffset(struct i2c_client *client, s8 ofs[MXC622X_AXES_NUM])
{	  
 int err=0;
#ifdef SW_CALIBRATION
 ofs[0]=ofs[1]=ofs[2]=0x0;
#else
/*
 if(err = hwmsen_read_block(client, MXC622X_REG_OFSX, ofs, MXC622X_AXES_NUM))
 {
	 GSE_ERR("error: %d\n", err);
 }
*/
#endif
 //printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
 
 return err;	
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ResetCalibration(struct i2c_client *client)
{
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
// u8 ofs[4]={0,0,0,0};
 int err=0;
 
#ifdef SW_CALIBRATION
	 
#else
/*
	 if(err = hwmsen_write_block(client, MXC622X_REG_OFSX, ofs, 4))
	 {
		 GSE_ERR("error: %d\n", err);
	 }
*/
#endif

 memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
 memset(obj->offset, 0x00, sizeof(obj->offset));
 return err;	
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadCalibration(struct i2c_client *client, int dat[MXC622X_AXES_NUM])
{
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
// int err=0;
 int mul;

#ifdef SW_CALIBRATION
	 mul = 0;//only SW Calibration, disable HW Calibration
#else

	 if ((err = MXC622X_ReadOffset(client, obj->offset))) {
	 GSE_ERR("read offset fail, %d\n", err);
	 return err;
	 }	  
	 mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;

#endif

 dat[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*(obj->offset[MXC622X_AXIS_X]*mul*GRAVITY_EARTH_1000/(obj->reso->sensitivity) + obj->cali_sw[MXC622X_AXIS_X]);
 dat[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*(obj->offset[MXC622X_AXIS_Y]*mul*GRAVITY_EARTH_1000/(obj->reso->sensitivity) + obj->cali_sw[MXC622X_AXIS_Y]);
					
									
 return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadCalibrationEx(struct i2c_client *client, int act[MXC622X_AXES_NUM], int raw[MXC622X_AXES_NUM])
{	
 /*raw: the raw calibration data; act: the actual calibration data*/
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
// int err;
 int mul;



#ifdef SW_CALIBRATION
	 mul = 0;//only SW Calibration, disable HW Calibration
#else
	 if(err = MXC622X_ReadOffset(client, obj->offset))
	 {
		 GSE_ERR("read offset fail, %d\n", err);
		 return err;
	 }	 
	 mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;
#endif
 
 raw[MXC622X_AXIS_X] = obj->offset[MXC622X_AXIS_X]*mul*GRAVITY_EARTH_1000/(obj->reso->sensitivity) + obj->cali_sw[MXC622X_AXIS_X];
 raw[MXC622X_AXIS_Y] = obj->offset[MXC622X_AXIS_Y]*mul*GRAVITY_EARTH_1000/(obj->reso->sensitivity) + obj->cali_sw[MXC622X_AXIS_Y];


 act[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*raw[MXC622X_AXIS_X];
 act[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*raw[MXC622X_AXIS_Y];
			
						
 return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_WriteCalibration(struct i2c_client *client, int dat[MXC622X_AXES_NUM])
{
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
 int err;
 int cali[MXC622X_AXES_NUM], raw[MXC622X_AXES_NUM];
// int lsb = mxc622x_offset_resolution.sensitivity;
// int divisor = obj->reso->sensitivity/lsb;
 err = MXC622X_ReadCalibrationEx(client, cali, raw);
 if(err)	 /*offset will be updated in obj->offset*/
 { 
	 GSE_ERR("read offset fail, %d\n", err);
	 return err;
 }

 GSE_LOG("OLDOFF: (%+3d %+3d): (%+3d %+3d) / (%+3d %+3d)\n", 
	 raw[MXC622X_AXIS_X], raw[MXC622X_AXIS_Y],
	 obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y],
	 obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y]);

 /*calculate the real offset expected by caller*/
 #if 0
 cali[MXC622X_AXIS_X] = cali[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
 cali[MXC622X_AXIS_Y] = cali[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		  

 #endif
 cali[MXC622X_AXIS_X] += dat[MXC622X_AXIS_X];
 cali[MXC622X_AXIS_Y] += dat[MXC622X_AXIS_Y];


 GSE_LOG("UPDATE: (%+3d %+3d)\n", 
	 dat[MXC622X_AXIS_X], dat[MXC622X_AXIS_Y]);

#ifdef SW_CALIBRATION
 obj->cali_sw[MXC622X_AXIS_X] = obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]]);
 obj->cali_sw[MXC622X_AXIS_Y] = obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]]);

#else
#if 0
 obj->offset[MXC622X_AXIS_X] = (s8)(obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]])/(divisor));
 obj->offset[MXC622X_AXIS_Y] = (s8)(obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]])/(divisor));
 obj->offset[MXC622X_AXIS_Z] = (s8)(obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]])/(divisor));

 /*convert software calibration using standard calibration*/
 obj->cali_sw[MXC622X_AXIS_X] = obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]])%(divisor);
 obj->cali_sw[MXC622X_AXIS_Y] = obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]])%(divisor);
 obj->cali_sw[MXC622X_AXIS_Z] = obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]])%(divisor);

 GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
	 obj->offset[MXC622X_AXIS_X]*divisor + obj->cali_sw[MXC622X_AXIS_X], 
	 obj->offset[MXC622X_AXIS_Y]*divisor + obj->cali_sw[MXC622X_AXIS_Y], 
	 obj->offset[MXC622X_AXIS_Z]*divisor + obj->cali_sw[MXC622X_AXIS_Z], 
	 obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y], obj->offset[MXC622X_AXIS_Z],
	 obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y], obj->cali_sw[MXC622X_AXIS_Z]);

 if(err = hwmsen_write_block(obj->client, MXC622X_REG_OFSX, obj->offset, MXC622X_AXES_NUM))
 {
	 GSE_ERR("write offset fail: %d\n", err);
	 return err;
 }
#endif
#endif

 return err;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetPowerMode(struct i2c_client *client, bool enable)
{
 u8 databuf[2];    
 int res = 0;
// u8 addr = MXC622X_REG_DETECTION;
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
 
 
 if(enable == sensor_power )
 {
	 GSE_LOG("Sensor power status is newest!\n");
	 return MXC622X_SUCCESS;
 }
#if 0	 
 if(hwmsen_read_block(client, addr, databuf, 0x01))
 {
	 GSE_ERR("read power ctl register err!\n");
	 return MXC622X_ERR_I2C;
 }
#endif	 
 
 if(enable == 1)
 {
	 databuf[1] =0x00;
 }
 else
 {
	 databuf[1] =0x01<<7;
 }
 
 databuf[0] = MXC622X_REG_DETECTION;
 

 res = cust_i2c_master_send(client, databuf, 0x2);

 if(res <= 0)
 {
	 GSE_LOG("set power mode failed!\n");
	 return MXC622X_ERR_I2C;
 }
 else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
 {
	 GSE_LOG("set power mode ok %d!\n", databuf[1]);
 }

 //GSE_LOG("MXC622X_SetPowerMode ok!\n");


 sensor_power = enable;

 mdelay(20);
 
 return MXC622X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{

 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
// u8 databuf[10];	
// int res = 0;
#if 0
 memset(databuf, 0, sizeof(u8)*10);    

 if(hwmsen_read_block(client, MXC622X_REG_DATA_FORMAT, databuf, 0x01))
 {
	 printk("mxc622x read Dataformat failt \n");
	 return MXC622X_ERR_I2C;
 }

 databuf[0] &= ~MXC622X_RANGE_MASK;
 databuf[0] |= dataformat;
 databuf[1] = databuf[0];
 databuf[0] = MXC622X_REG_DATA_FORMAT;


 res = cust_i2c_master_send(client, databuf, 0x2);

 if(res <= 0)
 {
	 return MXC622X_ERR_I2C;
 }
 
 //printk("MXC622X_SetDataFormat OK! \n");
 
#endif
 return MXC622X_SetDataResolution(obj);	  
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
#if 0
 u8 databuf[10];	
 int res = 0;

 memset(databuf, 0, sizeof(u8)*10);    

 if(hwmsen_read_block(client, MXC622X_REG_BW_RATE, databuf, 0x01))
 {
	 printk("mxc622x read rate failt \n");
	 return MXC622X_ERR_I2C;
 }

 databuf[0] &= ~MXC622X_BW_MASK;
 databuf[0] |= bwrate;
 databuf[1] = databuf[0];
 databuf[0] = MXC622X_REG_BW_RATE;


 res = cust_i2c_master_send(client, databuf, 0x2);

 if(res <= 0)
 {
	 return MXC622X_ERR_I2C;
 }
 
 //printk("MXC622X_SetBWRate OK! \n");
#endif	 
 return MXC622X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetIntEnable(struct i2c_client *client, u8 intenable)
{
#if 0
		 u8 databuf[10];	
		 int res = 0;
	 
		 res = hwmsen_write_byte(client, MXC622X_INT_REG, 0x00);
		 if(res != MXC622X_SUCCESS) 
		 {
			 return res;
		 }
		 printk("MXC622X disable interrupt ...\n");
	 
		 /*for disable interrupt function*/
#endif			 
		 return MXC622X_SUCCESS;    
}

static int MXC622X_CheckDeviceID(struct i2c_client *client)
{
 u8 databuf[10];
 int res = 0;

 while(res < 50)
 {
cust_hwmsen_read_block(client, 0x08, databuf, 1);
msleep(1);

 printk("zhaofei databuf[0] is %x  data&0x3F=%x\n", databuf[0],databuf[0]&0x3F);
 res++;

 if( (databuf[0] & 0x3F) == 0x5)
	 break;
 }
 if(res > 10)
	 return -1;

		 return MXC622X_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int mxc622x_init_client(struct i2c_client *client, int reset_cali)
{
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
 int res = 0;
 res = MXC622X_CheckDeviceID(client); 
 if(res != MXC622X_SUCCESS)
 {
	 return res;
 }	 
 
 res = MXC622X_SetPowerMode(client, false);
 if(res != MXC622X_SUCCESS)
 {
	 return res;
 }
 printk("MXC622X_SetPowerMode OK!\n");
 
 res = MXC622X_SetBWRate(client, MXC622X_BW_100HZ);
 if(res != MXC622X_SUCCESS ) 
 {
	 return res;
 }
 printk("MXC622X_SetBWRate OK!\n");
 
 res = MXC622X_SetDataFormat(client, MXC622X_RANGE_2G);
 if(res != MXC622X_SUCCESS) 
 {
	 return res;
 }
 printk("MXC622X_SetDataFormat OK!\n");

 gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


 res = MXC622X_SetIntEnable(client, 0x00);		 
 if(res != MXC622X_SUCCESS)
 {
	 return res;
 }
 printk("MXC622X disable interrupt function!\n");


 if(0 != reset_cali)
 { 
	 /*reset calibration only in power on*/
	 res = MXC622X_ResetCalibration(client);
	 if(res != MXC622X_SUCCESS)
	 {
		 return res;
	 }
 }
 printk("mxc622x_init_client OK!\n");
#ifdef CONFIG_MXC622X_LOWPASS
 memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

 mdelay(20);

 return MXC622X_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
 u8 databuf[10];	

 memset(databuf, 0, sizeof(u8)*10);

 if((NULL == buf)||(bufsize<=30))
 {
	 return -1;
 }
 
 if(NULL == client)
 {
	 *buf = 0;
	 return -2;
 }

 sprintf(buf, "MXC622X Chip");
 return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
 struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);
 u8 databuf[20];
 int acc[MXC622X_AXES_NUM];
 int res = 0;
 memset(databuf, 0, sizeof(u8)*10);

 if(NULL == buf)
 {
	 return -1;
 }
 if(NULL == client)
 {
	 *buf = 0;
	 return -2;
 }

 if(sensor_power == 0)
 {
	 res = MXC622X_SetPowerMode(client, true);
	 if(res)
	 {
		 GSE_ERR("Power on mxc622x error %d!\n", res);
	 }
 }
 res = MXC622X_ReadData(client, obj->data);
 if(res)
 {		  
	 GSE_ERR("I2C error: ret value=%d", res);
	 return -3;
 }
 else
 {
	 #if 1
	 obj->data[MXC622X_AXIS_X] = obj->data[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	 obj->data[MXC622X_AXIS_Y] = obj->data[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	#endif
	 //printk("raw data x=%d, y=%d, z=%d \n",obj->data[MXC622X_AXIS_X],obj->data[MXC622X_AXIS_Y],obj->data[MXC622X_AXIS_Z]);
	 obj->data[MXC622X_AXIS_X] += obj->cali_sw[MXC622X_AXIS_X];
	 obj->data[MXC622X_AXIS_Y] += obj->cali_sw[MXC622X_AXIS_Y];

	 
	 //printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[MXC622X_AXIS_X],obj->cali_sw[MXC622X_AXIS_Y],obj->cali_sw[MXC622X_AXIS_Z]);
	 
	 /*remap coordinate*/
	 acc[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*obj->data[MXC622X_AXIS_X];
	 acc[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*obj->data[MXC622X_AXIS_Y];

	 //printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[MXC622X_AXIS_X],obj->cvt.sign[MXC622X_AXIS_Y],obj->cvt.sign[MXC622X_AXIS_Z]);

	if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
	{
		GSE_LOG("Mapped gsensor data: %d, %d!\n", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y]);
	}

	 //Out put the mg
	 //printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[MXC622X_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
	#if 0
	 acc[MXC622X_AXIS_X] = acc[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	 acc[MXC622X_AXIS_Y] = acc[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	#endif
	if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
	{
		GSE_LOG("after * GRAVITY_EARTH_1000 / obj->reso->sensitivity: %d, %d!\n", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y]);
	}
 
	acc[MXC622X_AXIS_Z] = mxc622x_sqrt(9807, 0, 9807*9807-acc[MXC622X_AXIS_X]*acc[MXC622X_AXIS_X]-acc[MXC622X_AXIS_Y]*acc[MXC622X_AXIS_Y]);

	 sprintf(buf, "%04x %04x %04x", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y], acc[MXC622X_AXIS_Z]);
	 if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
	 {
		 GSE_LOG("gsensor data: %s!\n", buf);
	 }
 }
 
 return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadRawData(struct i2c_client *client, char *buf)
{
 struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);
 int res = 0;

 if (!buf || !client)
 {
	 return EINVAL;
 }
 res = MXC622X_ReadData(client, obj->data);
 if(res)
 {		  
	 GSE_ERR("I2C error: ret value=%d", res);
	 return EIO;
 }
 else
 {
	 sprintf(buf, "MXC622X_ReadRawData %04x %04x", obj->data[MXC622X_AXIS_X], 
		 obj->data[MXC622X_AXIS_Y]);
 
 }
 
 return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
 struct i2c_client *client = mxc622x_i2c_client;
 char strbuf[MXC622X_BUFSIZE];
 if(NULL == client)
 {
	 GSE_ERR("i2c client is null!!\n");
	 return 0;
 }
 
 MXC622X_ReadChipInfo(client, strbuf, MXC622X_BUFSIZE);
 return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);		 
}
 
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
 struct i2c_client *client = mxc622x_i2c_client;
 char strbuf[MXC622X_BUFSIZE];
 
 if(NULL == client)
 {
	 GSE_ERR("i2c client is null!!\n");
	 return 0;
 }
 MXC622X_ReadSensorData(client, strbuf, MXC622X_BUFSIZE);
 //MXC622X_ReadRawData(client, strbuf);
 return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			 
}

#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
 {
	 struct i2c_client *client = mxc622x_i2c_client;
	 char strbuf[MXC622X_BUFSIZE];
	 
	 if(NULL == client)
	 {
		 GSE_ERR("i2c client is null!!\n");
		 return 0;
	 }
	 //MXC622X_ReadSensorData(client, strbuf, MXC622X_BUFSIZE);
	 MXC622X_ReadRawData(client, strbuf);
	 return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			 
 }
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
 struct i2c_client *client = mxc622x_i2c_client;
 struct mxc622x_i2c_data *obj;
 int err, len = 0, mul;
 int tmp[MXC622X_AXES_NUM];

 if(NULL == client)
 {
	 GSE_ERR("i2c client is null!!\n");
	 return 0;
 }

 obj = i2c_get_clientdata(client);


 err = MXC622X_ReadOffset(client, obj->offset);
 if(err)
 {
	 return -EINVAL;
 }
 err = MXC622X_ReadCalibration(client, tmp);
 if(err )
 {
	 return -EINVAL;
 }
 
 	  
	 mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;
	 len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d) : (0x%02X, 0x%02X)\n", mul,						  
		 obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y],
		 obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y]);
	 len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d)\n", 1, 
		 obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y]);

	 len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d) : (%+3d, %+3d)\n", 
		 obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X],
		 obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y],
		 tmp[MXC622X_AXIS_X], tmp[MXC622X_AXIS_Y]);
	 
	 return len;
 
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
 struct i2c_client *client = mxc622x_i2c_client;  
 int err, x, y;//, z;
 int dat[MXC622X_AXES_NUM];

 if(!strncmp(buf, "rst", 3))
 {
	 err = MXC622X_ResetCalibration(client);
	 if(err)
	 {
		 GSE_ERR("reset offset err = %d\n", err);
	 }	 
 }
 else if(2 == sscanf(buf, "0x%02X 0x%02X", &x, &y))
 {
	 dat[MXC622X_AXIS_X] = x;
	 dat[MXC622X_AXIS_Y] = y;
	 err = MXC622X_WriteCalibration(client, dat);
	 if(err)
	 {
		 GSE_ERR("write calibration err = %d\n", err);
	 }		 
 }
 else
 {
	 GSE_ERR("invalid format\n");
 }
 
 return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC622X_LOWPASS
 struct i2c_client *client = mxc622x_i2c_client;
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
 if(atomic_read(&obj->firlen))
 {
	 int idx, len = atomic_read(&obj->firlen);
	 GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

	 for(idx = 0; idx < len; idx++)
	 {
		 GSE_LOG("[%5d %5d]\n", obj->fir.raw[idx][MXC622X_AXIS_X], obj->fir.raw[idx][MXC622X_AXIS_Y]);
	 }
	 
	 GSE_LOG("sum = [%5d %5d]\n", obj->fir.sum[MXC622X_AXIS_X], obj->fir.sum[MXC622X_AXIS_Y]);
	 GSE_LOG("avg = [%5d %5d]\n", obj->fir.sum[MXC622X_AXIS_X]/len, obj->fir.sum[MXC622X_AXIS_Y]/len);
 }
 return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
 return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_MXC622X_LOWPASS
 struct i2c_client *client = mxc622x_i2c_client;  
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
 int firlen;

 if(1 != sscanf(buf, "%d", &firlen))
 {
	 GSE_ERR("invallid format\n");
 }
 else if(firlen > C_MAX_FIR_LENGTH)
 {
	 GSE_ERR("exceeds maximum filter length\n");
 }
 else
 { 
	 atomic_set(&obj->firlen, firlen);
	 if(NULL == firlen)
	 {
		 atomic_set(&obj->fir_en, 0);
	 }
	 else
	 {
		 memset(&obj->fir, 0x00, sizeof(obj->fir));
		 atomic_set(&obj->fir_en, 1);
	 }
 }
#endif    
 return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
 ssize_t res;
 struct mxc622x_i2c_data *obj = obj_i2c_data;
 if (obj == NULL)
 {
	 GSE_ERR("i2c_data obj is null!!\n");
	 return 0;
 }
 
 res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));	   
 return res;	
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
 struct mxc622x_i2c_data *obj = obj_i2c_data;
 int trace;
 if (obj == NULL)
 {
	 GSE_ERR("i2c_data obj is null!!\n");
	 return 0;
 }
 
 if(1 == sscanf(buf, "0x%x", &trace))
 {
	 atomic_set(&obj->trace, trace);
 }	 
 else
 {
	 //GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
 }
 
 return count;	  
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;	 
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		 GSE_ERR("i2c_data obj is null!!\n");
		 return 0;
	}	 
	len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
			 obj->hw.i2c_num, obj->hw.direction, obj->hw.power_id, obj->hw.power_vol);	 
	return len;	
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
 if(sensor_power)
	 printk("G sensor is in work mode, sensor_power = %d\n", sensor_power);
 else
	 printk("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

 return 0;
}
	 
static ssize_t show_chip_orientation(struct device_driver *ddri, char *pbBuf)
{
	ssize_t _tLength = 0;
	struct mxc622x_i2c_data *obj = obj_i2c_data;

	if (obj == NULL)
		return 0;

	GSE_LOG("[%s] default direction: %d\n", __func__, obj->hw.direction);

	_tLength = snprintf(pbBuf, PAGE_SIZE, "default direction = %d\n", obj->hw.direction);

	return _tLength;
}


static ssize_t store_chip_orientation(struct device_driver *ddri, const char *pbBuf, size_t tCount)
{
	int _nDirection = 0;
	struct mxc622x_i2c_data *_pt_i2c_obj = obj_i2c_data;

	if (NULL == _pt_i2c_obj)
		return 0;

	if (!kstrtoint(pbBuf, 10, &_nDirection)) {
		if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
			GSE_ERR("ERR: fail to set direction\n");
	}

	GSE_LOG("[%s] set direction: %d\n", __func__, _nDirection);

	return tCount;
}

static u8 i2c_dev_reg;

static ssize_t show_register(struct device_driver *pdri, char *buf)
{
	GSE_LOG("i2c_dev_reg is 0x%2x\n", i2c_dev_reg);

	return 0;
}

static ssize_t store_register(struct device_driver *ddri, const char *buf, size_t count)
{
	i2c_dev_reg = simple_strtoul(buf, NULL, 16);
	GSE_LOG("set i2c_dev_reg = 0x%2x\n", i2c_dev_reg);

	return 0;
}

static ssize_t store_register_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	u8 databuf[2];
	unsigned long input_value;
	int res;

	memset(databuf, 0, sizeof(u8) * 2);

	input_value = simple_strtoul(buf, NULL, 16);
	GSE_LOG("input_value = 0x%2x\n", (unsigned int)input_value);

	if (NULL == obj) {
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}

	databuf[0] = i2c_dev_reg;
	databuf[1] = input_value;
	GSE_LOG("databuf[0]=0x%2x  databuf[1]=0x%2x\n", databuf[0], databuf[1]);

	res = i2c_master_send(obj->client, databuf, 0x2);

	if (res <= 0) {
		return -EINVAL;
	}
	return 0;

}

static ssize_t show_register_value(struct device_driver *ddri, char *buf)
{
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	u8 databuf[1];

	memset(databuf, 0, sizeof(u8) * 1);

	if (NULL == obj) {
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}

	if (hwmsen_read_block(obj->client, i2c_dev_reg, databuf, 0x01)) {
		GSE_ERR("read power ctl register err!\n");
		return -EINVAL;
	}

	GSE_LOG("i2c_dev_reg=0x%2x  data=0x%2x\n", i2c_dev_reg, databuf[0]);

	return 0;

}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,	 	S_IRUGO, show_chipinfo_value, 	  NULL);
static DRIVER_ATTR(sensordata, 		S_IRUGO, show_sensordata_value,	  NULL);
static DRIVER_ATTR(cali,	S_IWUSR | S_IRUGO, show_cali_value, 	  store_cali_value);
static DRIVER_ATTR(firlen, 	S_IWUSR | S_IRUGO, show_firlen_value,	  store_firlen_value);
static DRIVER_ATTR(trace,	S_IWUSR | S_IRUGO, show_trace_value,	  store_trace_value);
static DRIVER_ATTR(status, 	S_IRUGO, show_status_value,		  NULL);
static DRIVER_ATTR(powerstatus,		S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(i2c, S_IWUSR | S_IRUGO, show_register_value, store_register_value);
static DRIVER_ATTR(register, S_IWUSR | S_IRUGO, show_register, store_register);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *mxc622x_attr_list[] = {
	&driver_attr_chipinfo, 	/*chip information*/
	&driver_attr_sensordata,	/*dump sensor data*/
	&driver_attr_cali, 		/*show calibration data*/
	&driver_attr_firlen,		/*filter length: 0: disable, others: enable*/
	&driver_attr_trace,		/*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_orientation,
	&driver_attr_register,
	&driver_attr_i2c,
};
/*----------------------------------------------------------------------------*/
static int mxc622x_create_attr(struct device_driver *driver) 
{
 int idx, err = 0;
 int num = (int)(sizeof(mxc622x_attr_list)/sizeof(mxc622x_attr_list[0]));
 if (driver == NULL)
 {
	 return -EINVAL;
 }

 for(idx = 0; idx < num; idx++)
 {
	 err = driver_create_file(driver, mxc622x_attr_list[idx]);
	 if(err)
	 {			  
		 GSE_ERR("driver_create_file (%s) = %d\n", mxc622x_attr_list[idx]->attr.name, err);
		 break;
	 }
 }	  
 return err;
}
/*----------------------------------------------------------------------------*/
static int mxc622x_delete_attr(struct device_driver *driver)
{
 int idx ,err = 0;
 int num = (int)(sizeof(mxc622x_attr_list)/sizeof(mxc622x_attr_list[0]));

 if(driver == NULL)
 {
	 return -EINVAL;
 }
 

 for(idx = 0; idx < num; idx++)
 {
	 driver_remove_file(driver, mxc622x_attr_list[idx]);
 }
 

 return err;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_suspend(struct i2c_client *client, pm_message_t msg) 
{
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);	  
 int err = 0;
 GSE_FUN();    

 if(msg.event == PM_EVENT_SUSPEND)
 {	 
	 if(obj == NULL)
	 {
		 GSE_ERR("null pointer!!\n");
		 return -EINVAL;
	 }
	 atomic_set(&obj->suspend, 1);
	 err = MXC622X_SetPowerMode(obj->client, false);
	 if(err)
	 {
		 GSE_ERR("write power control fail!!\n");
		 return err;
	 }		 
 }
 return err;
}
/*----------------------------------------------------------------------------*/
static int mxc622x_resume(struct i2c_client *client)
{
 struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);		  
 int err;
 GSE_FUN();

 if(obj == NULL)
 {
	 GSE_ERR("null pointer!!\n");
	 return -EINVAL;
 }

 err = mxc622x_init_client(client, 0);
 if(err)
 {
	 GSE_ERR("initialize client fail!!\n");
	 return err;		
 }
 atomic_set(&obj->suspend, 0);

 return 0;
}



	 /*----------------------------------------------------------------------------*/
//add by major 
static int mxc622x_open_report_data(int open)
{
	    //should queuq work to report event if  is_report_input_direct=true
	    return 0;
}
static int mxc622x_enable_nodata(int en)
{
    int res =0;
    int retry = 0;
    bool power=false;

    if(1==en)
    {
        power=true;
    }
    if(0==en)
    {
       power =false;
    }
	for(retry = 0; retry < 3; retry++){
	     res = MXC622X_SetPowerMode(obj_i2c_data->client, power);
	      if(res == 0)
	       {
	           GSE_LOG("mxc622x_SetPowerMode done\n");
	            break;
	        }
	        GSE_LOG("Mxc622x_SetPowerMode fail\n");
	 }
 	if(res != 0)
	 {
	      GSE_LOG("mxc622x_SetPowerMode fail!\n");
	       return -1;
	 }
     GSE_LOG("mxc622x_enable_nodata OK!\n");
	 return 0;
}
static int mxc622x_set_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs/1000/1000;
	/*Fix Me*/
	GSE_LOG("mxc622x set delay = (%d) OK!\n", value);

	return 0;

}
static int mxc622x_flush(void)
{
	return acc_flush_report();
}

static int mxc622x_set_delay(u64 ns)
{
	int value =0;
	value = (int)ns/1000/1000;

	GSE_LOG("mxc622x_set_delay (%d), chip only use 1024HZ \n",value);
	return 0;
}
static int mxc622x_get_data(int* x ,int* y,int* z, int* status)
{
	char buff[MXC622X_BUFSIZE];
	MXC622X_ReadSensorData(obj_i2c_data->client, buff, MXC622X_BUFSIZE);

	sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static int mxc622x_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;

	err = mxc622x_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		GSE_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = mxc622x_set_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
	return 0;
}
static int mxc622x_factory_get_data(int32_t data[3], int *status)
{
	return mxc622x_get_data(&data[0], &data[1], &data[2], status);

}

static int mxc622x_factory_get_raw_data(int32_t data[3])
{
	char strbuf[MXC622X_BUFSIZE] = { 0 };

	MXC622X_ReadRawData(mxc622x_i2c_client, strbuf);
	GSE_LOG("support mxc622x_factory_get_raw_data!\n");
	return 0;
}
static int mxc622x_factory_enable_calibration(void)
{
	return 0;
}
static int mxc622x_factory_clear_cali(void)
{
	int err = 0;

	err = MXC622X_ResetCalibration(mxc622x_i2c_client);
	if (err) {
		GSE_ERR("MXC622X_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int mxc622x_factory_set_cali(int32_t data[3])
{
	int err = 0;
	int cali[3] = { 0 };

	/* obj */
	obj_i2c_data->cali_sw[MXC622X_AXIS_X] += data[0];
	obj_i2c_data->cali_sw[MXC622X_AXIS_Y] += data[1];
	obj_i2c_data->cali_sw[MXC622X_AXIS_Z] += data[2];

	cali[MXC622X_AXIS_X] = data[0] * gsensor_gain.x / GRAVITY_EARTH_1000;
	cali[MXC622X_AXIS_Y] = data[1] * gsensor_gain.y / GRAVITY_EARTH_1000;
	cali[MXC622X_AXIS_Z] = data[2] * gsensor_gain.z / GRAVITY_EARTH_1000;
	err = MXC622X_WriteCalibration(mxc622x_i2c_client, cali);
	if (err) {
		GSE_ERR("MXC622X_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int mxc622x_factory_get_cali(int32_t data[3])
{
	data[0] = obj_i2c_data->cali_sw[MXC622X_AXIS_X];
	data[1] = obj_i2c_data->cali_sw[MXC622X_AXIS_Y];
	data[2] = obj_i2c_data->cali_sw[MXC622X_AXIS_Z];
	return 0;
}
static int mxc622x_factory_do_self_test(void)
{
	return 0;
}

static struct accel_factory_fops mxc622x_factory_fops = {
	.enable_sensor = mxc622x_factory_enable_sensor,
	.get_data = mxc622x_factory_get_data,
	.get_raw_data = mxc622x_factory_get_raw_data,
	.enable_calibration = mxc622x_factory_enable_calibration,
	.clear_cali = mxc622x_factory_clear_cali,
	.set_cali = mxc622x_factory_set_cali,
	.get_cali = mxc622x_factory_get_cali,
	.do_self_test = mxc622x_factory_do_self_test,
};

static struct accel_factory_public mxc622x_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &mxc622x_factory_fops,
};

static int mxc622x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mxc622x_i2c_data *obj;
	//struct hwmsen_object sobj;
	struct acc_control_path ctl={0};//add by major 
	struct acc_data_path data={0}; //add by major 
	int err = 0;
	int retry = 0;

	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
 
	memset(obj, 0, sizeof(struct mxc622x_i2c_data));

	err = get_accel_dts_func(client->dev.of_node, &obj->hw);
	if (err < 0) {
		GSE_ERR("get dts info fail\n");
		return -1;
	}

	err = hwmsen_get_convert(obj->hw.direction, &obj->cvt);
	if(err)
	{
		GSE_ERR("invalid direction: %d\n", obj->hw.direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
#ifdef FPGA_EARLY_PORTING
	obj->client->timing = 100;
#else
	obj->client->timing = 400;
#endif
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
 
#ifdef CONFIG_MXC622X_LOWPASS
	if(obj->hw.firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw.firlen);

	if(atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
 
#endif

	mxc622x_i2c_client = new_client; 
	
	for (retry = 0; retry < 3; retry++) {
		err = mxc622x_init_client(new_client, 1);
		if (0 != err) {
			GSE_ERR("mxc622x_device init cilent fail time: %d\n", retry);
			continue;
		}
	}
	if (err != 0)
		goto exit_init_failed;
 
	ctl.is_use_common_factory = false; //add by major 

	err = accel_factory_device_register(&mxc622x_factory_device);
	if (err) {
		GSE_ERR("acc_factory register failed.\n");
		goto exit_misc_device_register_failed;
	}

	err = mxc622x_create_attr(&(mxc622x_init_info.platform_diver_addr->driver));
	if(err)
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	//add by major begin 
	ctl.open_report_data= mxc622x_open_report_data;
	ctl.enable_nodata = mxc622x_enable_nodata;
	ctl.set_delay  = mxc622x_set_delay;
	ctl.batch = mxc622x_set_batch;
	ctl.flush = mxc622x_flush;
	ctl.is_report_input_direct = false;

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ctl.is_support_batch = obj->hw.is_batch_supported;
#else
	ctl.is_support_batch = false;
#endif

	err = acc_register_control_path(&ctl);
	if(err)
	{
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = mxc622x_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
		GSE_ERR("register acc data path err= %d\n", err);
		goto exit_kfree;
	}

	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	obj =NULL;
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);		  
	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_remove(struct i2c_client *client)
{
	int err = 0;	 
	err = mxc622x_delete_attr(&(mxc622x_init_info.platform_diver_addr->driver));
	if(err)
		GSE_ERR("mxc622x_delete_attr fail: %d\n", err);

	mxc622x_i2c_client = NULL;
	i2c_unregister_device(client);
	accel_factory_device_deregister(&mxc622x_factory_device);
	kfree(i2c_get_clientdata(client));
	return 0;
}
	 /*----------------------------------------------------------------------------*/

static int mxc622x_remove(void)
{
	struct mxc622x_i2c_data *obj = obj_i2c_data;

	GSE_FUN();

	i2c_del_driver(&mxc622x_i2c_driver);
	return 0;
}

static int  mxc622x_local_init(void)
{
        struct mxc622x_i2c_data *obj = obj_i2c_data;

	GSE_FUN();
	
	if(i2c_add_driver(&mxc622x_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}

	/*
	if(MXC622X_INIT_FAIL == s_nInitFlag)
	{
		return -1;
	}
	*/
	return 0;
}

//add end 
static int __init mxc622x_init(void)
{
//	GSE_FUN();

	acc_driver_add(&mxc622x_init_info); //add by major 
	mutex_init(&g_gsensor_mutex);
	return 0;	  
}
/*----------------------------------------------------------------------------*/
static void __exit mxc622x_exit(void)
{
//	GSE_FUN();
	mutex_destroy(&g_gsensor_mutex);
}
/*----------------------------------------------------------------------------*/
module_init(mxc622x_init);
module_exit(mxc622x_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC622X I2C driver");


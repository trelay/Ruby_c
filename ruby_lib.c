#include <stdio.h>
#include <time.h>
#include <sys/io.h>
#include <errno.h>
#include <string.h>
#include "ruby_lib.h"
unsigned char value=0xff;
unsigned short int port;
extern char *prgname;
unsigned char iicbuff[MAX_IIC];

static int read_ruby(unsigned short int port)
{
	static int iopldone = 0;
	if (!iopldone && iopl(3))
	{
		fprintf(stderr, "%s: iopl(): %s\n", prgname, strerror(errno));
	    return 1;
	}

	iopldone++;
	value=inb(port);
	return value;
}

static int write_ruby(unsigned short int port, unsigned char value)
{
    if((port<0x0a00)||(port>0x0aff))
    {
        fprintf(stderr, "invalid port\n");
        return -1;
    }
	/******************Attempt to win arbitration****************/
    if((port>0xa00)&&(port<0x0a3f))      //only channel 0 has arbitration
	{
	int i;
	int chan0_status;
	for(i=0;i<3;i++)
	{
		read_ruby(RUBY_CHAN_LAST_BYTE_REG);
		chan0_status = read_ruby(RUBY_CHAN_STATUS_REG);
		if((chan0_status & RUBY_ARB_LASTSW)|| (chan0_status & RUBY_ARB_SW1ST))
			continue;
		else break;
	} 
	if(i>=3)
		{
			fprintf(stderr,"Can not gain LPC ownership while writing!\n");
			return -1;
		}
    }
	/******************Attempt to win arbitration****************/
	static int iopldone = 0;
	if (!iopldone && iopl(3))
	{
		fprintf(stderr, "%s: iopl(): %s\n", prgname, strerror(errno));
	    return 1;
	}

	iopldone++;
	outb(value,port);
	outb(0x00,RUBY_CHAN_LAST_BYTE_REG); //free arbitration
	return 0;
}

static int iicread(unsigned int iicbus, unsigned int deviceadd, unsigned int offset, unsigned int length)
{
    int i;
    if(iicbus!=1&&iicbus!=2)
    {
        fprintf(stderr, "invalid iicbus value!");
        return -1;
    }
    deviceadd|=1;        //LSB =1, for read
    if((deviceadd!=(BQ34Z_ADDR|1))&&(deviceadd!=(RUBY_AT24C_ADDR|1))&&(deviceadd!=(EEPROM_AT24C_ADDR|1)))
    {
       fprintf(stderr, "invalid deviceadd value!");
        // fprintf(stderr, "value should be %d or %d, 
	    //but received %d",BQ34Z_ADDR|1,AT24C_ADDR|1,deviceadd);
       return -1;
    }
    if(length>MAX_IIC)
    {
        fprintf(stderr, "invalid length value!");
        return -1;
    }
    memset(iicbuff,0,MAX_IIC);
    write_ruby(0x0A40,0x02);	//clear read ready flag
	usleep(10000);
    for(i=0;i<100;i++)	//check the LSB of IIC Bridge Status Register until IIC is available
    {
		read_ruby(0x0A40);
        if((value&1)==0)
            break;
		usleep(10000);
    }
    if(i>=100)                          //out of time case
    {
        fprintf(stderr, "MCU is busy!");
        //fprintf(stderr, "iic status register=%d\r\n",value);
        return -1;
    }

    write_ruby(0x0A80,2);	//TXbytes, indicate two byte of TX data(1 byte address, 1 byte length)
    write_ruby(0x0A81,length);            //RXbytes
    write_ruby(0x0A82,deviceadd);	//TXBuff[0], the address of iic device
    write_ruby(0x0A83,offset);	//TXbuff[1], where start the read

    if(iicbus==1)
        write_ruby(0x0A44,1);	//0x0A44 is IIC bridge command register, start IIC transmition here
    else
        write_ruby(0x0A44,3);
    for(i=0;i<100;i++)
    {
	read_ruby(0x0A40);
        if((value&3)==2)
            break;
            usleep(10000);
    }
    if(i>=100)                          //out of time case
    {
        fprintf(stderr, "iic read out of time!");
        return -1;
    }
    for(i=0;i<length;i++)
    {
		read_ruby(IIC_RX_BUFF+i);
        iicbuff[i]=value;
    }
    return 0;
}

int present_check(int* bbu_index)
{
	if (*bbu_index==1)
		port=BBU1_PRST_N;
	else if (*bbu_index==2)
		port=BBU2_PRST_N;
	else
		fprintf(stderr,"BBU should be: 1 or 2\n");

	read_ruby(port);
	if (value==0x0)
		printf("BBU-%d is present.\n",*bbu_index);
	else if (value==0x1)
		printf("BBU-%d is not present.\n",*bbu_index);
	else
		printf("BBU-%d present unknown,code:0x%02x\n",*bbu_index,value);
	return 0;

}

int charge_stat(int* bbu_index)
{
    if (*bbu_index==1)
        port=BBU1_CHARGER_STAT;
    else if (*bbu_index==2)
        port=BBU2_CHARGER_STAT;
    else
        fprintf(stderr,"BBU should be: 1 or 2\n");

    read_ruby(port);
    if (value==0x0)
        printf("BBU-%d: Charge in progress\n",*bbu_index);
    else if (value==0x1)
        printf("BBU-%d: Charge complete or sleep mode\n",*bbu_index);
	else if (value==0x2)
        printf("BBU-%d:Charge suspend\n",*bbu_index);
    else
        printf("BBU-%d charge status unknown,code:0x%02x\n",*bbu_index,value);
    return 0;

}

int led_ctrl(int led_name, int led_stat)
{
	if (led_name<1 || led_name>5)
	{
	fprintf(stderr,"LED Name is not correct, type -h for help.\n");
	return -1;
	}

	if (led_stat<1 || led_stat>5)
	{
	fprintf(stderr,"LED Name is not correct, type -h for help.\n");
	return -1;
	}

	printf("led_name:%d led_stat:%d\n",led_name, led_stat);
	switch(led_name)
    {
        case 1:
            write_ruby(0x0a06,led_stat);
            if(read_ruby(0x0a06)!=led_stat)
                fprintf(stderr,"control BBU1 FAULT LED failed\n");
            else
                fprintf(stderr,"control BBU1 FAULT LED done\n");
            break;
        case 2:
            write_ruby(0x0a08,led_stat);
            if(read_ruby(0x0a08)!=led_stat)
                fprintf(stderr,"control BBU2 FAULT LED failed\n");
            else
                fprintf(stderr,"control BBU2 FAULT LED done\n");
            break;
        case 3:
            write_ruby(0x0a0a,led_stat);
            if(read_ruby(0x0a0a)!=led_stat)
                fprintf(stderr,"control Cluster Status LED failed\n");
            else
                fprintf(stderr,"control Cluster Status LED done\n");
            break;
        case 4:
            write_ruby(0x0a0c,led_stat);
            if(read_ruby(0x0a0c)!=led_stat)
                fprintf(stderr,"control System Fault LED failed\n");
            else
                fprintf(stderr,"control System Fault LED done\n");
            break;
        case 5:
            write_ruby(0x0a0e,led_stat);
            if(read_ruby(0x0a0e)!=led_stat)
                fprintf(stderr,"control Buttery Subsystem LED failed\n");
            else
                fprintf(stderr,"control Buttery Subsystem LED done\n");
            break;
    }
    return 0;

}

int ver_read()
{
	printf("Firmware Version: %d.%d\n",read_ruby(CODE_VER),read_ruby(CODE_TEST_VER));
}

int read_sn(int bbu)
{
    //I got the device address by reading all possible addresses from A0 to AF;
    //And the offset is 0x44, length is 12;
    int iicbus;
    unsigned int AT24C_ADDR;
    char * SN_Name;

    if(bbu!=0&&bbu!=1&&bbu!=2)
    {
        fprintf(stderr,"bbu value out of range!\n0.Ruby Backplate,  1.BBU1,    2.BBU2\n");
        return -1;
    }

    switch(bbu) {
      case 1:
        SN_Name="BBU1";
	iicbus = 1;
        AT24C_ADDR = EEPROM_AT24C_ADDR;
        break;
      case 2:
        SN_Name="BBU2";
	iicbus = 2;
        AT24C_ADDR = EEPROM_AT24C_ADDR;
        break;
      default:
        SN_Name="Ruby";
	iicbus = 2;
        AT24C_ADDR = RUBY_AT24C_ADDR;
    }

    if(iicread(iicbus, AT24C_ADDR, 0x44,12))
    printf("read SN failed!\n");

    printf("%s SN:%s\n",SN_Name,iicbuff);
}

int debug()
{
	//printf("BBU1_DSG_ENABLE_N: %d\n",read_ruby(BBU1_DSG_ENABLE_N));
	//printf("BBU1_CHG_ENABLE_N: %d\n",read_ruby(BBU1_CHG_ENABLE_N));
	//printf("BBU1_SLOW_CHARGE_N: %d\n",read_ruby(BBU1_SLOW_CHARGE_N));
	//printf("______________________________________________________\n");
	//printf("BBU2_DSG_ENABLE_N: %d\n",read_ruby(BBU2_DSG_ENABLE_N));
	//printf("BBU2_CHG_ENABLE_N: %d\n",read_ruby(BBU2_CHG_ENABLE_N));
	//printf("BBU2_SLOW_CHARGE_N: %d\n",read_ruby(BBU2_SLOW_CHARGE_N));
        unsigned char i;
	//if(iicread(1, 0xAA, 0x6d,0x1))
	//if(iicread(2, AT24C_ADDR, 0x44,0x1f))
	    printf("______________________________________________________\n");
            
	    if(iicread(2,0xA1, 0x44,12))
            printf("read failed!\n");
	    printf("GET:%s",iicbuff);
            printf("********************************************************\n");
}

int BBUctrl(int bbu, int ctrl)
{
    if(bbu!=1&&bbu!=2)
    {
        fprintf(stderr,"bbu value out of range!\n");
        return -1;
    }
    if(ctrl<1||ctrl>4)
    {
        fprintf(stderr,"ctrl value out of range: %d!\n",ctrl);
        return -1;
    }
    if(bbu==1)
    {
		read_ruby(BBU1_PRST_N);
        if(value)
        {
            fprintf(stderr,"BBU1 is not present, test quit!\n");
            return -1;
        }
    }
    else
    {
		read_ruby(BBU2_PRST_N);
        if(value)
        {
            fprintf(stderr,"BBU2 is not present, test quit!\n");
            return -1;
        }
    }
    switch(ctrl)
    {
        case 1:                 //disable charge, slow charge, discharge
            if(bbu==1)
            {
                write_ruby(BBU1_DSG_ENABLE_N,DISABLE);
                write_ruby(BBU1_CHG_ENABLE_N,DISABLE);
                write_ruby(BBU1_SLOW_CHARGE_N,DISABLE);
            }
            else
            {
                write_ruby(BBU2_DSG_ENABLE_N,DISABLE);
                write_ruby(BBU2_CHG_ENABLE_N,DISABLE);
                write_ruby(BBU2_SLOW_CHARGE_N,DISABLE);
            }
            break;
        case 2:                 //Enable normal charge, disable other
            if(bbu==1)
            {
                write_ruby(BBU1_DSG_ENABLE_N,DISABLE);
                write_ruby(BBU1_SLOW_CHARGE_N,DISABLE);
                write_ruby(BBU1_CHG_ENABLE_N,ENABLE);
            }
            else
            {
                write_ruby(BBU2_DSG_ENABLE_N,DISABLE);
                write_ruby(BBU2_SLOW_CHARGE_N,DISABLE);
                write_ruby(BBU2_CHG_ENABLE_N,ENABLE);
            }
            break;
        case 3:                 //enable slow charge, disable others
            if(bbu==1)
            {
                write_ruby(BBU1_DSG_ENABLE_N,DISABLE);
                write_ruby(BBU1_CHG_ENABLE_N,DISABLE);
                write_ruby(BBU1_SLOW_CHARGE_N,ENABLE);
            }
            else
            {
                write_ruby(BBU2_DSG_ENABLE_N,DISABLE);
                write_ruby(BBU2_CHG_ENABLE_N,DISABLE);
                write_ruby(BBU2_SLOW_CHARGE_N,ENABLE);
            }
            break;
        case 4:                     //enable discharge, disable others
            if(bbu==1)
            {
                write_ruby(BBU1_CHG_ENABLE_N,DISABLE);
                write_ruby(BBU1_SLOW_CHARGE_N,DISABLE);
                write_ruby(BBU1_DSG_ENABLE_N,ENABLE);
            }
            else
            {
                write_ruby(BBU2_CHG_ENABLE_N,DISABLE);
                write_ruby(BBU2_SLOW_CHARGE_N,DISABLE);
                write_ruby(BBU2_DSG_ENABLE_N,ENABLE);
            }
            break;
        case 5:                     //enable discharge, disable others
            if(bbu==1)
            {
                write_ruby(BBU1_CHG_ENABLE_N,ENABLE);
                write_ruby(BBU1_SLOW_CHARGE_N,ENABLE);
                write_ruby(BBU1_DSG_ENABLE_N,ENABLE);
            }
            else
            {
                write_ruby(BBU2_CHG_ENABLE_N,ENABLE);
                write_ruby(BBU2_SLOW_CHARGE_N,ENABLE);
                write_ruby(BBU2_DSG_ENABLE_N,ENABLE);
            }
            break;
    }
    sleep(1);               //sleep 1 second, then start to get the status
	if(bbu==1)
	{
		printf("BBU1_DSG_ENABLE_N: %d\n",read_ruby(BBU1_DSG_ENABLE_N));
		printf("BBU1_CHG_ENABLE_N: %d\n",read_ruby(BBU1_CHG_ENABLE_N));
		printf("BBU1_SLOW_CHARGE_N: %d\n",read_ruby(BBU1_SLOW_CHARGE_N));
	}
	else
	{
		printf("BBU2_DSG_ENABLE_N: %d\n",read_ruby(BBU2_DSG_ENABLE_N));
		printf("BBU2_CHG_ENABLE_N: %d\n",read_ruby(BBU2_CHG_ENABLE_N));
		printf("BBU2_SLOW_CHARGE_N: %d\n",read_ruby(BBU2_SLOW_CHARGE_N));
	}
}

int status_check(int bbu)
{
	unsigned int BBU_FCC;
	unsigned int BBU_RMC;
    if(bbu!=1&&bbu!=2)
    {
        fprintf(stderr,"bbu value out of range!\n");
        return -1;
    }
	
	//get the state of charge
    if(iicread(bbu, BQ34Z_ADDR, BQ34Z_SOC, 2))
        fprintf(stderr,"read state of charge failed!\r\n");
    else
		printf("State of Charge=%d%%\n",(iicbuff[1]*256+iicbuff[0]));
	
	//get the BBU voltage
    if(iicread(bbu, BQ34Z_ADDR, BQ34Z_VOLT, 2))
		fprintf(stderr,"read BBU voltage failed!\n");
    else
		printf("BBU voltage=%dmV\n",(iicbuff[1]*256+iicbuff[0]));
	
	//get the charge or discharge current of BBU
    if(iicread(bbu, BQ34Z_ADDR, BQ34Z_AI, 2))    
		fprintf(stderr,"read charge or discharge current failed!\n");
	else
	{
        if(iicbuff[1]&0x80)
            printf("Current=%dmA\n",((iicbuff[1]*256+iicbuff[0])-65536));
        else
            printf("Current=%dmA\n",(iicbuff[1]*256+iicbuff[0]));
    }
	
	//Read BBU FullChargeCapacity, RemainingCapacity
	if(iicread(bbu, BQ34Z_ADDR, BQ34Z_FCC, 2))
		fprintf(stderr,"read BBU FullChargeCapacity failed!\n");
    else
		BBU_FCC=iicbuff[1]*256+iicbuff[0];
	
	if(iicread(bbu, BQ34Z_ADDR, BQ34Z_RMC, 2))
		fprintf(stderr,"read BBU RemainingCapacity failed!\n");
    else
		BBU_RMC=iicbuff[1]*256+iicbuff[0];
	printf("FullChargeCapacity:%d, RemainingCapacity:%d\n",BBU_FCC,BBU_RMC);

	/*
	int BQ77908_CHG;
	int BQ77908_DSG;
    if(bbu==1)
	{
		BQ77908_CHG=BBU1_BQ77908_CHG;
		BQ77908_DSG=BBU1_BQ77908_DSG;
	}
    else
	{
		BQ77908_CHG=BBU2_BQ77908_CHG;
		BQ77908_DSG=BBU2_BQ77908_DSG;
	}
	//read_ruby(BQ77908_CHG);
	//int value_CHG=read_ruby(BQ77908_CHG);
	//read_ruby(BQ77908_DSG);
	//int value_DSG=read_ruby(BQ77908_DSG);	
	//printf("BBU%d BQ77908_CHG=%d,\tBQ77908_DSG=%d\n",bbu,read_ruby(BQ77908_CHG),read_ruby(BQ77908_DSG));
	*/
}

int gpio_read()
{
	printf("BBU1_PRST_N=%d,   \t\tBBU2_PRST_N=%d\n",read_ruby(BBU1_PRST_N),read_ruby(BBU2_PRST_N));
	printf("BBU1_DSG_ENABLE_N=%d,\t\tBBU2_DSG_ENABLE_N=%d\n",read_ruby(BBU1_DSG_ENABLE_N),read_ruby(BBU2_DSG_ENABLE_N));
	printf("BBU1_CHG_ENABLE_N=%d,\t\tBBU2_CHG_ENABLE_N=%d\n",read_ruby(BBU1_CHG_ENABLE_N),read_ruby(BBU2_CHG_ENABLE_N));
	printf("BBU1_TEST_CAL=%d,    \t\tBBU2_TEST_CAL=%d\n",read_ruby(BBU1_TEST_CAL),read_ruby(BBU2_TEST_CAL));
	printf("EPOW_CABLE_PRST_N=%d,\t\tPS2_PRESENT_N=%d\n",read_ruby(EPOW_CABLE_PRST_N),read_ruby(PS2_PRESENT_N));
	printf("BBU1_BQ77908_DSG=%d, \t\tBBU2_BQ77908_DSG=%d\n",read_ruby(BBU1_BQ77908_DSG),read_ruby(BBU2_BQ77908_DSG));
	printf("BBU1_BQ77908_CHG=%d, \t\tBBU2_BQ77908_CHG=%d\n",read_ruby(BBU1_BQ77908_CHG),read_ruby(BBU2_BQ77908_CHG));
	printf("BBU1_SLOW_CHARGE_N=%d,\t\tBBU2_SLOW_CHARGE_N=%d\n",read_ruby(BBU1_SLOW_CHARGE_N),read_ruby(BBU2_SLOW_CHARGE_N));
	printf("BBU1_CHARGER_STAT=%d,\t\tBBU2_CHARGER_STAT=%d\n",read_ruby(BBU1_CHARGER_STAT),read_ruby(BBU2_CHARGER_STAT));
	printf("EPOW_asserted_N=%d\n",read_ruby(EPOW_asserted_N));
}

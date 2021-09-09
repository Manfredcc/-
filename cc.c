#include <stdio.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
//#include "sensor.h"
//#include "math.h"
//#include "bmp.h"
#include <sys/mman.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

int *plcd=NULL;
int fd_lcd;

double Temp1=0;
double Pressure1=0;
double Humity1=0;




int init_serial(const char *file, int baudrate)
{ 
	int fd;
	
	fd = open(file, O_RDWR);
	if (fd == -1)
	{
		perror("open device error:");
		return -1;
	}

	struct termios myserial;
	//清空结构体
	memset(&myserial, 0, sizeof (myserial));
	//O_RDWR               
	myserial.c_cflag |= (CLOCAL | CREAD);
	//设置控制模式状态，本地连接，接受使能
	//设置 数据位
	myserial.c_cflag &= ~CSIZE;   //清空数据位
	myserial.c_cflag &= ~CRTSCTS; //无硬件流控制
	myserial.c_cflag |= CS8;      //数据位:8

	myserial.c_cflag &= ~CSTOPB;//   //1位停止位
	myserial.c_cflag &= ~PARENB;  //不要校验
	//myserial.c_iflag |= IGNPAR;   //不要校验
	//myserial.c_oflag = 0;  //输入模式
	//myserial.c_lflag = 0;  //不激活终端模式

	switch (baudrate)
	{
		case 9600:
			cfsetospeed(&myserial, B9600);  //设置波特率
			cfsetispeed(&myserial, B9600);
			break;
		case 115200:
			cfsetospeed(&myserial, B115200);  //设置波特率
			cfsetispeed(&myserial, B115200);
			break;
		case 19200:
			cfsetospeed(&myserial, B19200);  //设置波特率
			cfsetispeed(&myserial, B19200);
			break;
	}
	
	/* 刷新输出队列,清除正接受的数据 */
	tcflush(fd, TCIFLUSH);

	/* 改变配置 */
	tcsetattr(fd, TCSANOW, &myserial);

	return fd;
}

void parse_data(unsigned char data[], int len)
{
	if (data[0] != 0x5A || data[1] != 0x5A)
	{
		return ;
	}

	//0x5A 0x5A type len data[4]
	unsigned char type = data[2];
	if (type == 0x15)
	{
		//光照强度
		double Lux =  ((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7])) / 100.0 ;
		
		
		printf("Lux = %g\n",Lux);
		
		
	}
	else if (type == 0x45)
	{	
		//温度、湿度、海拔、气压  type = 0x45  len = 10  个完整的数据帧总共 14bytes
		//	0x5A 0x5A 0x45 0X0A data1 data2 data3 data4 data5 data6 data7 data8 data9 data10 checksum
		double Temp = ((data[4] << 8) | (data[5]))/100.0 ;
		double Pressure =  ((data[6] << 24) | (data[7] << 16) | (data[8] << 8) | (data[9]) ) /100.0 ;
		double Humity =  ((data[10] << 8) | (data[11]) ) / 100.0 ;
		double Altitude = ((data[12] << 8) | (data[13]) ) / 100.0;

		
		
		printf("Temp = %g C\n", Temp);
		
		
		printf("Pressure = %g KPa\n", Pressure/1000);

		
		
		printf("Humity = %g%\n", Humity);
		printf("Altitude = %gm\n",Altitude);
		printf("\n");
		sleep(1);
		Temp1=Temp*100;
		Pressure1=Pressure*100;
		Humity1=Humity*100;
	}
}


void *gy39_data()
{
	int data_fd = init_serial("/dev/ttySAC1",9600);
	if(data_fd==-1)
	{
		printf("open failed\n");
	}
	//帧头+命令+校验位低八位
	unsigned char cmd[] = {0xa5,0x83,0x28};
	write(data_fd,cmd,3);	

	//GY-39传感器数据输出格式为
	//0x5A 0x5A (type) (len) -- -- -- --
	
	unsigned char buf[32] = {0};
	unsigned char ch;
	while(1)
	{
		//先读取到第一个数据0x5A中去
		do
		{	
			read(data_fd,&ch,1);
			
		}while (ch != 0x5A);
		
		read(data_fd,&ch,1);
		if(ch != 0x5A)
		{
			continue;
		}

		buf[0]=0x5A;//帧头数据
		buf[1]=0x5A;//第二次的数据
		
		//读取数据类型
		read(data_fd,&buf[2],1);
		//读取数据长度
		read(data_fd,&buf[3],1);
		//读取传感器数据
		int len=buf[3];
		int i=0;
		for(i=0;i<len;i++)
		{
			read(data_fd,&buf[4+i],1);
		}
		//读取末尾帧数据
		read(data_fd,&buf[4+len],1);

		parse_data(buf,4+len+1);
	}
}

//解析数据
void parse_data2(unsigned char data[], int len)
{
	if (data[0] != 0xFF || data[1] != 0x86)
	{
		return ;
	}
	//接收的数据格式：FF 86 00 85 00 00 00 00 F5
	int yw = 0;
	yw = data[3]<<8 | data[4];
	int yw1=yw/100;
	printf("yw1 = %d\n",yw1);

	//显示数值
	//clear_lcd(700, 300, 100, 50, 0xffffff);
	//show_num(740, 200, yw, 0xff0000);

	//蜂鸣器报警
	

}

//获取MQ-2烟雾传感器数据
void* get_mq2(void * arg)
{
	int fd = init_serial("/dev/ttySAC2", 9600);
	if (fd == -1)
	{
		printf("failed to open\n");
		return NULL;
	}

	unsigned char wbuf[9] = {0xff, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79};
	unsigned char rbuf[9]={0};
	short int yw;//保存烟雾浓度
	int r; 
	while(1)
	{
		 //发送命令给传感器
		r = write(fd,wbuf,9);
		if(r != 9)//如果没有写入9个字节
		{
			sleep(1);
			continue;//重新写入
		}
		sleep(1);
		//printf("122221\n");
		r = read(fd,rbuf,9);
		if(r != 9)
		{
			sleep(1);
			continue;
		}
		//解析数据
		parse_data2(rbuf,9);
	}

}

//画点
void lcd_draw_point(int x,int y,int color)
{
	if(x>=0 && x<800 && y>=0 && y<480)
	{
		*(plcd+800*y+x)=color;
	}			
}



//横向写字模
void display_hanzi(int x0,int y0,char zi[],int w,int h)
	{
		int dian;//扫描时，扫描点的编号 0-w*h
		int x,y;//表示点在屏幕上的位置
		int color;//表示点的颜色
		for(dian=0;dian<w*h;dian++)
		{
			int xb=dian/8;
			int bit=7-dian%8;
			if(zi[xb] & (1<<bit))
			{
				color=0x000000;
			}
			else
			{
				color=0xffffff;
			}
			x=x0+dian%w;
			y=y0+dian/w;
			lcd_draw_point(x,y,color);
		}
	}
//lcd画点
void display_dian(int x0,int y0)
{
	/*--  文字:  .  --*/
	/*--  宋体12;  此字体下对应的点阵为：宽x高=8x16   --*/
	unsigned char dian[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00,0x00,0x00,0x00,0x00};
	
	display_hanzi(x0,y0,dian,8,16);
}

//lcd画数字
void display_shuzi(int x0,int y0,int a)
{
	char n0[] = {0x00,0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18,0x00,0x00};
	char n1[] = {0x00,0x00,0x00,0x08,0x38,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00};
	char n2[] = {0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x02,0x04,0x08,0x10,0x20,0x42,0x7E,0x00,0x00};
	char n3[] = {0x00,0x00,0x00,0x3C,0x42,0x42,0x02,0x04,0x18,0x04,0x02,0x42,0x42,0x3C,0x00,0x00};
	char n4[] = {0x00,0x00,0x00,0x04,0x0C,0x0C,0x14,0x24,0x24,0x44,0x7F,0x04,0x04,0x1F,0x00,0x00};
	char n5[] = {0x00,0x00,0x00,0x7E,0x40,0x40,0x40,0x78,0x44,0x02,0x02,0x42,0x44,0x38,0x00,0x00};
	char n6[] = {0x00,0x00,0x00,0x18,0x24,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x22,0x1C,0x00,0x00};
	char n7[] = {0x00,0x00,0x00,0x7E,0x42,0x04,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x00,0x00};
	char n8[] = {0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x24,0x18,0x24,0x42,0x42,0x42,0x3C,0x00,0x00};
	char n9[] = {0x00,0x00,0x00,0x38,0x44,0x42,0x42,0x42,0x46,0x3A,0x02,0x02,0x24,0x18,0x00,0x00};
	switch (a)
	{
		case 0:
		{
			display_hanzi( x0, y0, n0,8,16);
			break;
		}
		case 1:
		{
			display_hanzi( x0, y0, n1,8,16);
			break;
		}
		case 2:
		{
			display_hanzi( x0, y0, n2,8,16);
			break;
		}
		case 3:
		{
			display_hanzi( x0, y0, n3,8,16);
			break;
		}
		case 4:
		{
			display_hanzi( x0, y0, n4,8,16);
			break;
		}
		case 5:
		{
			display_hanzi( x0, y0, n5,8,16);
			break;
		}
		case 6:
		{
			display_hanzi( x0, y0, n6,8,16);
			break;
		}
		case 7:
		{
			display_hanzi( x0, y0, n7,8,16);
			break;
		}
		case 8:
		{
			display_hanzi( x0, y0, n8,8,16);
			break;
		}
		case 9:
		{
			display_hanzi( x0, y0, n9,8,16);
			break;
		}
	}
}


//四个数字的显示
void display_data4(int x0,int y0,int num)
{
	display_shuzi(x0,y0,num/1000);
	display_shuzi(x0+8,y0,(num%1000)/100);
	display_dian(x0+16,y0);
	display_shuzi(x0+24,y0,(num%100)/10);
	display_shuzi(x0+32,y0,num%10);
}
//五个数字的显示
void display_data5(int x0,int y0,int num)
{	
	display_shuzi(x0,y0,num/10000);
	display_shuzi(x0+8,y0,(num%10000)/1000);
	display_shuzi(x0+16,y0,(num%1000)/100);
	display_dian(x0+24,y0);
	display_shuzi(x0+32,y0,(num%100)/10);
	display_shuzi(x0+40,y0,num%10);
}
//六个数字的显示
void display_data6(int x0,int y0,int num)
{
	display_shuzi(x0,y0,num/100000);
	display_shuzi(x0+8,y0,(num%100000)/10000);
	display_shuzi(x0+16,y0,(num%100000)%10000/1000);
	display_shuzi(x0+24,y0,(num%1000)/100);
	display_dian(x0+32,y0);
	display_shuzi(x0+40,y0,(num%100)/10);
	display_shuzi(x0+48,y0,num%10);	
}
//七个数字的显示
void display_data7(int x0,int y0,int num)
{
	display_shuzi(x0,y0,num/1000000);
	display_shuzi(x0+8,y0,(num%1000000)/100000);
	display_shuzi(x0+16,y0,(num%100000)/10000);
	display_shuzi(x0+24,y0,(num%100000)%10000/1000);
	display_shuzi(x0+32,y0,(num%1000)/100);
	display_dian(x0+40,y0);
	display_shuzi(x0+48,y0,(num%100)/10);
	display_shuzi(x0+56,y0,num%10);	
}

//八个数字的显示
void display_data8(int x0,int y0,int num)
{
	display_shuzi(x0,y0,num/10000000);
	display_shuzi(x0+8,y0,(num%10000000)/1000000);
	display_shuzi(x0+16,y0,(num%1000000)/100000);
	display_shuzi(x0+24,y0,(num%100000)/10000);
	display_shuzi(x0+32,y0,(num%100000)%10000/1000);
	display_shuzi(x0+40,y0,(num%1000)/100);
	display_dian(x0+48,y0);
	display_shuzi(x0+56,y0,(num%100)/10);
	display_shuzi(x0+62,y0,num%10);	
}

//判断显示的数字num含有多少个数字，再去选择对应个数数字的显示
void display_data(int x0,int y0,int num)
{
	if(num/10000000!=0)
	{
		display_data8( x0,  y0,  num);
	}
	else if(num/1000000!=0)
	{
		display_data7( x0,  y0,  num);
	}
	else if(num/100000!=0)
	{
		display_data6( x0, y0,  num);
	}
	else if(num/10000!=0)
	{
		display_data5( x0, y0,  num);
	}
	else if(num/1000!=0)
	{
		display_data4( x0,  y0,  num);
	}

}

//lcd屏幕初始化
void lcd_init()
{
	//1.打开屏幕文件
	fd_lcd=open("/dev/fb0",O_RDWR);
	if(-1==fd_lcd)
	{
		perror("open error");
	}
	//2.将屏幕文件映射要内存区
	plcd=mmap(NULL,480*800*4,PROT_READ | PROT_WRITE,MAP_SHARED,fd_lcd,0);
}
//关闭lcd
void lcd_close()
{
	munmap(plcd,480*800*4);
	close(fd_lcd);
}

void *display_shuzhi()
{
	lcd_init();
		
	int i;
	while(1)
	{			
		display_data(720,240, Temp1);
		display_data(720,340, Humity1);
		display_data(720,440, Pressure1);
		
	}
	lcd_close();	


}






int main()
{
	pthread_t pid1,pid2;
	pthread_create(&pid1,NULL,gy39_data,NULL);
	pthread_create(&pid2,NULL,get_mq2,NULL);
	
	pthread_t pid3;
	pthread_create(&pid3,NULL,display_shuzhi,NULL);
	while(1);	
}


























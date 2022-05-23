#include "windows.h"
#include"winsock2.h"
#include "niimaq.h"
#include "nivision.h"
#include "formatio.h"
#include <cvirte.h>		
#include <userint.h>
#include <utility.h> 
#include <ansi_c.h>  
#include "MPlay.h"
#include "serial.h" 
#include"mbsupp.h"
  
static int panelHandle;


#include <stdlib.h>
#include <stdio.h> 
#include <string.h>
#include <windows.h>
#include "asynctmr.h"
#include "UDP.h"
#include <rs232.h>
#include <formatio.h>
#include <math.h>

/* Image scale. */
#define IMAGE_SCALE        0.5

   double Time            = 0.0;
   double TimeWait        = 0.0;
   double TotalReplay     = 0.0;
   double ImageScale      = IMAGE_SCALE;
   long   n               = 0;
   long   ExtendedAttrib  = 0;
   long   NbFrames        = 0;
   long   Selection       = 1;
   long   LicenseModules  = 0;

      int nCount;
         POINT Hor[2];
         POINT Ver[2];
         SIZE  TxtSz;
         RECT  Txt;
         Rect circle;

 
   long           ImageWidth, ImageHeight;
   long           TransparentColor;


int panel_handle,
    config_handle,
    scan_handle,
    cage_handle,
    track_handle,
    adjust_handle;
    scope_handle,
    yzmb_handle,
    cam_handle,
    comport,
    baudrate,
    portindex,
    parity,
    databits,
    stopbits,
    inputq,         /* Sets input queue length in OpenComConfig */
    outputq,        /* Sets output queue length in OpenComConfig */
    xmode,
    ctsmode,
    stringsize,
    bytes_sent,
    bytes_read,
    RS232Error,
    config_flag,
    breakstatus,
    port_open,
    com_status,
    send_mode,
    send_byte,
    send_term_index,
    read_term_index,
    read_term,
    inqlen,         /* Stores result from GetInQLen */
    outqlen;        /* Stores result from GetOutQLen */
	startOK=0;
	sumi,x,y,z;
int send_buf[1000];
int baseaddress=0xd000,keybuf,keyvalue1,keyvalue2,key1,key2,key3,key4;
int ad_buffer[4],adx,ady;
int yc[64],ptyc[128],yc1[64],yc2[64],yc3[64],yc4[64];
short read_cnt;
int command_bit,command_bit1;//;
double timeout;
double timebuf=0;
double gd,gd3,gd2,jl,fw,hx,qx,fy,f,fwj,gdj,dj_jl[20],dj_jl1,dj_jl2;
float gps_hx;
double glat,glon;//任务机经度和纬度
double glat0,glon0;  //控制站经度和纬度
double BZ,LZ,BZ1,LZ1;          //经纬度坐标
char devicename[30],
     send_data[500],
     read_data[2000],
     tbox_read_data[2000],
     com_msg[500],
     msg[100];
	Rect rectVal;
    Point pointVal1, pointVal2,pointVal3,pointVal4;
    Point pointArray[6];
int f_table;
int linex,liney;
double ex,ex1;
int ex2;
int sendbit=0;
int keybufbit=0; 
int *keybuf1; 
int zt,kzzt=0;
int qlbj=0,qlbj1=0;
int ImageStatus=0,Agree=0,ClearOnce=1,DrawOnce=0;
int drawx1,drawx2,drawy1,drawy2,drawx3,drawy3,drawx4,drawy4;
int tvx=0,tvy=0,linex1,liney1,ImageUpdata=0,mbok=0;
char string[80];
char Datastring[100],Datastring1[100],Datastring2[100];
char Timestring[100],Timestring1[100],Timestring2[100];
char mbdisdata[30],pcdisdata[30],mbdisdata1[30],pcdisdata1[30];
int tvctr=1,irctr=1;
float s1,s2,s3,s4,s5,c1,c2,c3,c4,c5;
float d0,d00,d000,d1,d2,d3,a11,a12,a1,a2,a3,a0,a00,b11,b12,b1,b2,b3,b0,b00;
float hx11,fy1,qx1,zj111,zj21,gd1,xm1,ym1,xd1,yd1,fjfw,ptfwj,ptgdj,xm2,ym2,xm3,ym3,xm4,ym4;
float ff,gd0;
float dj_hx1,dj_fy1,dj_qx1,dj_fwj1,dj_gdj1,dj_f1,dj_gd1,dj_xm1,dj_ym1;
float dj_hx2,dj_fy2,dj_qx2,dj_fwj2,dj_gdj2,dj_f2,dj_gd2,dj_xm2,dj_ym2;
float dj_hx[20],dj_fy[20],dj_qx[20],dj_fwj[20],dj_gdj[20],dj_f,dj_gd,dj_xm,dj_ym;
float chx[20],cfy[20],cqx[20],cfwj[20],cgdj[20],cf[20],cgd[20];   //遥测数据暂存单元
int  ci=1;
long cxs[20],cys[20];	  //遥测数据暂存单
float ahx[20],afy[20],aqx[20],afwj[20],agdj[20],af[20],agd[20];   //遥测数据暂存单元
int  ca=0,ca1=0,ca2=0,ca3=0;
long axs[20],ays[20];	  //遥测数据暂存单
unsigned pintaihao;
double xs,ys,xs_1,ys_1,xs1,ys1,xs4,ys4;
long xt,yt,xz,yz,dj_xs[20],dj_ys[20];
long dj_xs1,dj_ys1,dj_xs2,dj_ys2;
long prex,prey,pcx,pcy;
long yzmbx,yzmby,yzmbx1,yzmby1,yzmbx2,yzmby2,yzmbx3,yzmby3,yzmbx4,yzmby4,yzmbx5,yzmby5;
long yzmbx6,yzmby6,yzmbx7,yzmby7,yzmbx8,yzmby8,yzmbx9,yzmby9,yzmbx10,yzmby10,yzmbx11,yzmby11,yzmbx12,yzmby12; 
//char yzmbstring1[20],yzmbstring2[20],yzmbstring3[20],yzmbstring4[20],yzmbstring5[20];
char yzmbstring6[20],yzmbstring7[20],yzmbstring8[20],yzmbstring9[20],yzmbstring10[20],yzmbstring11[20],yzmbstring12[20]; 
char yzmbxs[200];
long xs0,ys0;//控制站直角坐标
float xm,ym,xd,yd;
int gd_jggd=0,gdch_buf=0,zdok;
int ycjs=0;
float fjhx,pthxjiao;
int mbxy,mbxbuf,mbybuf;//目标消影
int ptds_bj,ri,ri1,ptycok,portstatus=0;
int spx,czy,czyex,czyex1,czyex2;
int pt_ctr_status;
int hours, minutes, seconds; 
int month, day, year; 
int mb_zd_shift=0;
int lx_lx,lx_pf,lx_kj,lx_kt;
int active_ccd_ir=0,active_ccd_ir_dj=0; //=4当前为电视=0当前为红外
int time_set_status=0;
int xj_open20_status,xj_open60_status;
int lxj_backinf_display;
int lxjdata1,lxjdata2,lxjdata3,lxjdata4,jgdata1,jgdata2,jgdata3,jgdata4;
char lxj_string[100],lxj_string1[100],lxj_string2[100];
float jg_gd,cha_gd;//激光高度和激光高度与气压高度差
int jg_gd_get=0,gd_get_status,hx_get_status,hx_gpshx_get=0;
float dj_jggd,dj_qygd,dj_jggd1,dj_jggd2,dj_qygs1,dj_qygd2,dj_gpshx,dj_chx;
int filehandle;
char filestring[200];
char xsstring[20],ysstring[20],planestringx[20],planestringy[20],targetstringx[20],targetstringy[20],zdstringx[20],zdstringy[20],pcstringx[20],pcstringy[20];
int stringlength=0;
int pt_sfjg_status,fj_cm_status;   //平台收放机构和飞机仓门状态
int ir_view_status;
double ir_f;
int control_fact=6,control_fact1=6;;
int wzpzx_buf,wzpzy_buf;
int wzpz_first;
int active_overlayimage;
int beix,beiy,beijiao;
int plane_beix,plane_beiy;
double plane_beijiao;
char planestring[100];
int focus_ptqxt_shift=0,shiftok=0;
int DisCorrSel;  //直角坐标和经纬度坐标转换标志
char filename[30],filename1[50];
char pathname[MAX_PATHNAME_LEN];
int mbnum,bufnum;
int gzss;
int ljex;
int  globalX;
int  globalY;
int  leftBtn;
int  rightBtn;
int  keyStates;
#define QuitHelp        1
#define InputqHelp      2
int xxyy;
int ffhh;
int readbj=0;
double plane_x0=0;
double plane_y0=0;
int foc_sel=0;
int selstatus=0,selstatus0=1; 
int display_status=0;
int controlimage=0;
int newcount=0;
char mbstring[20],pcstring[20],zdstring[20],zbstring[20],mbtzstring[20],gdjdstring[20];
char scalestring[30];
int scalenum,distancex,distancey;
char first,second,three,four,five,six,seven,eight,nine,ten,eleven,twelve; 
int datasource=1;
//char *rdata=NULL;
//char *wdata=NULL;
int pp=0;
//int ppt=1;
char testbuf[40]; 
int djnum=0,djnum1;
int juliz,julil; 
int ledstatus=0,ledstatus1=0,tableRS=0; 
float image_x,image_y;
int zb_mode,mborzd=1,DWORCT=1;
double jwdx,jwdy,zjx,zjy;
int ctordw;
int image_buf=1;
static int flag=1;
int valuei=0,valuej=0;
int testx[4]={0,0,0,0},testy[4]={0,0,0,0};
static const RGBValue IMAQ_RGB_APP0 = {163,243,139,0};
static const RGBValue IMAQ_RGB_APP1 = {163,139,243,0};
static const RGBValue IMAQ_RGB_GRAY = {250,250,250,0};
static const RGBValue IMAQ_RGB_GRAY0 = {0,246,180,0}; 
OverlayTextOptions options={"Arial",18,0,0,0,0,IMAQ_LEFT,IMAQ_TOP,{0,0,0,1},0.0};
Point position,position1;

static int PlotFlag;
static SESSION_ID Sid;
static INTERFACE_ID Iid;
static int AcqWinWidth, AcqWinHeight;
static Int8* ImaqBuffer; 

/********************************************************************/

void DisplayRS232Error (void);
void SetConfigParms (void);
void GetConfigParms (void);
void DisplayHelp (int);
void EnablePanelControls (int);
void DisplayComStatus (void);
void ActivateSendControls (int);
void SendAscii (void);
void SendByte (void);
void ChangeData(void);
void ReadByte(void); 
void CoordinateShift(double,double);
void CoordinateShift1(long,long);
void Initilation(void);
void Drawimage(void);
void StartRS(void);
void CVICALLBACK WSACallback (WinMsgWParam wParam, WinMsgLParam lParam,void *callbackData);
void IMAQ_CALLBACK  ExtractImage (WindowEventType event, int windowNumber, Tool tool, Rect rect);
void CVICALLBACK RSCallback (int portNo,int eventMask,void *callbackData);
void CVICALLBACK Event_Char_Detect_Func (int portNo,int eventMask,void *callbackData);
static int CVICALLBACK MyThreadFunction (void *ctrlID);
static int CVICALLBACK ImagePlay(void *ctrlID); 
static int SetupApplication (void);
static int ShutdownApplication (void);
void drawcanvas(int scalenum);
void drawinitial(void);
void initYzmb(void);
/********************************************************************/
#define WINDOW_TO_USE 0 
#define MAX_THREADS 60
Image*     image; 
Image*     image1;
int status=0,m;
int imagenum;
int *mapbuf=NULL;
int udpHandle, err; 
int table1=1,table2=2;
char buf1[23657*2+10],imagedata[23456*2+4094];
int sign=1,result;
int j=0,t=0,q;
int centerX,centerY;
HANDLE myfile;
HWND hwnd; 
static int ImaqHandle;  // will contain the handle to the imaq window
static int windHandle;  // will contain the handle to the CVI window
unsigned long	bitDepth;
int readbjj=0,rii=0;
float adjust_heading;
int wucha_x,wucha_y;
unsigned int wmsg;
int NetID;
static int poolHandle = 0;
static volatile int exiting = 0; 
int lock;
int frame1;
int ir_or_ccd=0; //=0 ccd;=1 ir
float blxs_x,blxs_y;
/********************************************************************/ 

int main (int argc, char *argv[])
{
    if (InitCVIRTE (0, argv, 0) == 0)    /* Needed if linking in external compiler; harmless otherwise */
        return -1;    /* out of memory */
    
    panel_handle = LoadPanel (0, "serial.uir", SERIAL);
    DisplayPanel (panel_handle);
	keybuf1=(int*)malloc(2*sizeof(int)); //指针分配空间最好放在前
	MakeDir("D:\\temp2");
	MakeDir("D:\\temp1"); 
			myfile=CreateFileMapping((HANDLE)0xffffffff,0,PAGE_READWRITE,0,0x300000,"bmp*$#1992");
			mapbuf=(int *)MapViewOfFile(myfile,FILE_MAP_ALL_ACCESS, 0,0,0);
			initYzmb();
			SetCtrlAttribute (panel_handle, SERIAL_LED_3, ATTR_ON_COLOR, MakeColor(104,217,91));
		    SetCtrlAttribute (panel_handle, SERIAL_LED_3, ATTR_OFF_COLOR,MakeColor(32,101,24));
		    SetCtrlAttribute (panel_handle, SERIAL_LED_2, ATTR_ON_COLOR, MakeColor(104,217,91));
		    SetCtrlAttribute (panel_handle, SERIAL_LED_2, ATTR_OFF_COLOR,MakeColor(32,101,24));
 	   	    SetCtrlAttribute (panel_handle, SERIAL_LED_3, ATTR_ON_COLOR, MakeColor(104,217,91));
		    SetCtrlAttribute (panel_handle, SERIAL_LED_3, ATTR_OFF_COLOR,MakeColor(32,101,24));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_1, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_2, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_3, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_4, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_5, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_6, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_7, ATTR_ON_COLOR, MakeColor(149,138,47));
		    SetCtrlAttribute (panel_handle, SERIAL_YZMB_8, ATTR_ON_COLOR, MakeColor(149,138,47));
		    GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
		    if(datasource==0)
		      {
		        SetCtrlVal (panel_handle, SERIAL_LED_2, 0);
		        SetCtrlVal (panel_handle, SERIAL_LED_3, 1);
		      }
		    else
		      {
		        SetCtrlVal (panel_handle, SERIAL_LED_2, 1);
		        SetCtrlVal (panel_handle, SERIAL_LED_3, 0);
		      }
//		    drawinitial();
		    CmtNewLock (NULL, 0, &lock);
			SetupApplication(); 
			UDPOpen (5102, &udpHandle); /*open  local port  for udp Transmission*/
	        hwnd=(HWND)GetCVIWindowHandle();
	        /*acquire data from buffer whenever data arrived*/
        	wmsg=RegisterWinMsgCallback (WSACallback, NULL, NULL, 0, &NetID, TRUE);
	        WSAAsyncSelect (udpHandle, hwnd, wmsg, FD_READ);
			
			/*set the image window to be one part of the CVI window*/
			imaqSetupWindow(WINDOW_TO_USE,0);
			imaqSetWindowPalette(WINDOW_TO_USE,2,NULL,0);
	        windHandle = (int) FindWindow (NULL, "TARGET LOCATION"); /* get CVI window handle*/
			image = imaqCreateImage (IMAQ_IMAGE_RGB, 0);
			image1= imaqCreateImage (IMAQ_IMAGE_RGB, 0); 
			ImaqHandle = (int) imaqGetSystemWindowHandle(0);   /*get vision image window handle*/
			SetParent( (HWND) ImaqHandle, (HWND)windHandle);
			//imaqSetWindowTitle( WINDOW_TO_USE, "Show Image" );
    		imaqMoveWindow( WINDOW_TO_USE, imaqMakePoint(3,5));
    		imaqSetCurrentTool (IMAQ_POINT_TOOL);
		    
    		CmtScheduleThreadPoolFunction (poolHandle,ImagePlay,NULL, NULL);    
    		
    		SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 1);
    		SetCtrlVal (panel_handle, SERIAL_CHECKBOX, 1);
    		GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
    		if(datasource==2)
    		  {
    		     exiting=1;
    		     StartRS();
    		     tableRS=1;
    		  }
    		
	zb_mode=1;
	//串口初始化		
	comport=1;
	baudrate=38400;//9600;
	parity=0;//1;
	databits=8;
	stopbits=1;
	inputq=512;
	outputq=512;
    RS232Error = OpenComConfig (comport, devicename, baudrate, parity, databits, stopbits, inputq, outputq); 
    InstallComCallback (1, LWRS_RXCHAR, 0, 0, RSCallback, 0);    
    RunUserInterface ();
    CmtDiscardLock (lock);
    ShutdownApplication();
	imaqDispose( image ); 		  
	FlushInQ (5);
    return 0;
}

/********************************************************************/
/*functions of RS232 Interruption functions */ 
void CVICALLBACK RSCallback (int portNo,int eventMask,void *callbackData)
{
   
   GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
	  if(datasource==2)
	   {
            ReadByte();		// 每80ms接收遥测一次
		    
		      if(ledstatus1==0)
		          {
		            ledstatus1=1;
		            newcount=0;
		            SetCtrlVal (panel_handle, SERIAL_LED_2, 0);
		          }
		        else
		          {
		            ledstatus1=0;
		            newcount=0;
		            SetCtrlVal (panel_handle, SERIAL_LED_2, 1);
		          } 
	   }	      
}

void CVICALLBACK Event_Char_Detect_Func (int portNo,int eventMask,void *callbackData)
{
int readbuf;
printf("%%%%%%");
readbuf= ComRdByte (comport);
if(readbjj==0)
{
    if(readbuf==0xeb)
	{
	printf("&&&&&&");					
	readbjj=1;ptyc[0]=0xeb;rii++;
	}
}	
else
	{
	ptyc[rii]=readbuf;
	if(rii<127)
	rii++;
	else {readbjj=0;rii=0;}
	}
    return;
}   

/********************************************************************/
static int CVICALLBACK ImagePlay (void *ctrlID)
{
	long filesize;
	int test1;
	while(!exiting)	
	 {
	   if(controlimage==0)
	   	{
		   if(*mapbuf==1)
			{
				 imaqDuplicate(image1,image);
		         test1=imaqReadFile (image, "d:\\temp\\index1.bmp", NULL, NULL); 
		         if(test1==0)
		           {
		           		imaqDuplicate(image,image1);
		           }
				 Drawimage();
			     imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
				 display_status=1;
			}
	       else if(*mapbuf==2)
			{
				
                 imaqDuplicate(image1,image); 
		         test1=imaqReadFile (image, "d:\\temp\\index2.bmp", NULL, NULL); 
		         if(test1==0)
		           {
		           		imaqDuplicate(image,image1);
		           }
				 Drawimage();
			     imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
				 display_status=1;
			}
	    }		
	}
   return 0;	
}



static int CVICALLBACK MyThreadFunction (void *ctrlID)
{   
	int ii=0;
 	char *imagedata1;
 	if(!exiting)
 	{
	 	imagedata1=(char*)ctrlID;
	 	CmtGetLock (lock);
		while(ii<stringlength+23456*frame1)
	      {
	        	if(((*((int *)(imagedata1+ii)))&0xff)==0xfd)
	              if(((*((int *)(imagedata1+ii+1)))&0xff)==0xb1)
	                if(((*((int *)(imagedata1+ii+2)))&0xff)==0x85)
		              if(((*((int *)(imagedata1+ii+3)))&0xff)==0x40)
			            if(((*((int *)(imagedata1+ii+4)))&0xff)==0xfd)
				          if(((*((int *)(imagedata1+ii+5)))&0xff)==0xb1)
					        if(((*((int *)(imagedata1+ii+4090)))&0xff)==0x7a)
						      if(((*((int *)(imagedata1+ii+4091)))&0xff)==0xbf)
							   if(((*((int *)(imagedata1+ii+4092)))&0xff)==0x02)
								 if(((*((int *)(imagedata1+ii+4093)))&0xff)==0x4e)
				                     { 
			                             if(controlimage==0)
				                            AddBuffer(imagedata1+ii+6);
						                 ii+=4094;
						                 if(ii+4093>=stringlength+23456*frame1)
							               {
							                  stringlength=stringlength+23456*frame1-ii;
							                  memmove(imagedata,imagedata1+ii,stringlength);
							                  break;
										   }  
						                 continue;
		                             }
	             if(ii+4093>=stringlength+23456*frame1)
	               {
	                  stringlength=stringlength+23456*frame1-ii;
	                  memmove(imagedata,imagedata1+ii,stringlength);
	                  break;
				   }                     
               
				 ii++;	
			  
	     }
	    CmtReleaseLock(lock);
    }
    return 0;
}

static int SetupApplication (void)
{
    exiting=0;
    CmtNewThreadPool (MAX_THREADS, &poolHandle);
    return 0;
}


static int ShutdownApplication (void)
{  
    if (poolHandle != 0) {
    	exiting = 1;
        CmtDiscardThreadPool (poolHandle);
    } 
    return 0;
}   

/********************************************************************/ 
/*functions of UDP Interruption functions */

void CVICALLBACK WSACallback (WinMsgWParam wParam, WinMsgLParam lParam,
         void *callbackData)
{
  
  int test0,test,w,ii;
  int low,hight,pt_buf,pt_bufh,yc_buf,yc_bufh;
  unsigned char total;
  int q=0,t=0,fm=0;
  int p,frame; 
  char errortest[10]=" ";
     GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
	  if(datasource==0)
	     {
 	   /*flush the led when this function works */
 	     
		   /* acquire data from buffer */
          	test0=UDPRead(udpHandle,buf1, 23657*2+10,5);
          	if(test0==0)
          	  {
          	     if(ledstatus==0)
		          {
		            ledstatus=1;
		            newcount=0;
		            SetCtrlVal (panel_handle, SERIAL_LED_3, 0);
		          }
		        else
		          {
		            ledstatus=0;
		            newcount=0;
		            SetCtrlVal (panel_handle, SERIAL_LED_3, 1);
		          }  
          	  }
             frame=(*(int *)(buf1+4))&0xff;
             low=(*(int *)(buf1+8))&0xff;
             hight=(*(int *)(buf1+9))&0xff;
             hight=(hight<<8)|low;
          	 CmtGetLock (lock);  
             frame1=frame;
             for(p=0;p<frame1;p++)
             {
		     	memcpy(imagedata+stringlength+23456*p,buf1+10+hight*p,23456);	
             }
             CmtReleaseLock(lock);
                /* select image data from packet */  
             CmtScheduleThreadPoolFunction (poolHandle, MyThreadFunction, (void*)imagedata, NULL);       	 
// get the yc data
			
	  for(ii=23456;ii<23657;ii++)
	  {
	  if(((*((int *)(buf1+ii)))&0xff)==0xEB)
	  if(((*((int *)(buf1+ii+1)))&0xff)==0x90)
	  {
	  
	  pt_buf= ((*((int *)(buf1+ii+3)))&0xff);
	   switch(pt_buf)
                 {
                 	  case 'A':
                 	     yc_buf= ((*((int *)(buf1+ii+4)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+5)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     //yc_buf=|0x8000;
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     fy=yc_buf*0.1;
                 	     //fy=0xfff6;
                 	     yc_buf= ((*((int *)(buf1+ii+6)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+7)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     qx=yc_buf*0.1;
                 	     yc_buf= ((*((int *)(buf1+ii+8)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+9)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     hx=yc_buf*0.1;
                 	     //p++;
                 	     //printf("AAAA=%d     ",p);
                 	     break;
                 	  case 'B':
                 	     yc_buf= ((*((int *)(buf1+ii+4)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+5)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh= ((*((int *)(buf1+ii+6)))&0xff);
                 	     yc_buf=(yc_buf&0xffff)|((yc_bufh&0x00ff)<<16);
                 	     yc_bufh= ((*((int *)(buf1+ii+7)))&0xff);
                 	     yc_buf=(yc_buf&0xffffff)|((yc_bufh&0x00ff)<<24);
                 	     yc_bufh=yc_buf&0x80000000;
                 	     if(yc_bufh==0x80000000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffffffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     glon=yc_buf*0.000001;
                 	   //  glat=yc_buf*0.000001;
                 	     yc_buf= ((*((int *)(buf1+ii+8)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+9)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh= ((*((int *)(buf1+ii+10)))&0xff);
                 	     yc_buf=(yc_buf&0xffff)|((yc_bufh&0x00ff)<<16);
                 	     yc_bufh= ((*((int *)(buf1+ii+11)))&0xff);
                 	     yc_buf=(yc_buf&0xffffff)|((yc_bufh&0x00ff)<<24);
                 	     yc_bufh=yc_buf&0x80000000;
                 	     if(yc_bufh==0x80000000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffffffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     glat=yc_buf*0.000001;
                 	     //glon=yc_buf*0.000001;
                 	     yc_buf= ((*((int *)(buf1+ii+15)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+16)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     gd=yc_buf*0.5-gd3;
                 	     
                 	     
                 	     yc_buf= ((*((int *)(buf1+ii+13)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+14)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     gps_hx=yc_buf*0.1;
                 	     //q++;
                 	     //printf("BBBB=%d     ",q);
                 	     break;
                 	  case 'C':
                 	    // t++;
                 	    // printf("CCCC=%d     ",t);
                 	     break;
                 	  case 'D':
                 	    // fm++;
                 	    // printf("DDDD=%d     ",fm);
                 	     break;
                 }									

	  break;
	  }	
	  }		

	  if(frame>1)
	  {
	  for(ii=23456*2;ii<23657*2;ii++)
	  {
	  if(((*((int *)(buf1+ii)))&0xff)==0xEB)
	  if(((*((int *)(buf1+ii+1)))&0xff)==0x90)
	  {
	  
	  pt_buf= ((*((int *)(buf1+ii+3)))&0xff);
	   switch(pt_buf)
                 {
                 	  case 'A':
                 	     yc_buf= ((*((int *)(buf1+ii+4)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+5)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     fy=yc_buf*0.1;
                 	    // fy=0xfff6;
                 	     yc_buf= ((*((int *)(buf1+ii+6)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+7)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     qx=yc_buf*0.1;
                 	     yc_buf= ((*((int *)(buf1+ii+8)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+9)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     hx=yc_buf*0.1;
                 	     //p++;
                 	     //printf("AAAA=%d     ",p);
                 	     break;
                 	  case 'B':
                 	     yc_buf= ((*((int *)(buf1+ii+4)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+5)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh= ((*((int *)(buf1+ii+6)))&0xff);
                 	     yc_buf=(yc_buf&0xffff)|((yc_bufh&0x00ff)<<16);
                 	     yc_bufh= ((*((int *)(buf1+ii+7)))&0xff);
                 	     yc_buf=(yc_buf&0xffffff)|((yc_bufh&0x00ff)<<24);
                 	     yc_bufh=yc_buf&0x80000000;
                 	     if(yc_bufh==0x80000000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffffffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     glon=yc_buf*0.000001;
                 	     yc_buf= ((*((int *)(buf1+ii+8)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+9)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh= ((*((int *)(buf1+ii+10)))&0xff);
                 	     yc_buf=(yc_buf&0xffff)|((yc_bufh&0x00ff)<<16);
                 	     yc_bufh= ((*((int *)(buf1+ii+11)))&0xff);
                 	     yc_buf=(yc_buf&0xffffff)|((yc_bufh&0x00ff)<<24);
                 	     yc_bufh=yc_buf&0x80000000;
                 	     if(yc_bufh==0x80000000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffffffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     glat=yc_buf*0.000001;
                 	     yc_buf= ((*((int *)(buf1+ii+15)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+16)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     //yc_buf=fy&0x7fff;
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     gd=yc_buf*0.5-gd3;
                 	     
                 	     yc_buf= ((*((int *)(buf1+ii+13)))&0xff);
                 	     yc_bufh= ((*((int *)(buf1+ii+14)))&0xff);
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     gps_hx=yc_buf*0.1;
                 	     //q++;
                 	     //printf("BBBB=%d     ",q);
                 	     break;
                 	  case 'C':
                 	    // t++;
                 	    // printf("CCCC=%d     ",t);
                 	     break;
                 	  case 'D':
                 	    // fm++;
                 	    // printf("DDDD=%d     ",fm);
                 	     break;
                 }									

	  break;
	  }	
	  }		
	  }				
	  		
	  //get the platform data
	  for(ii=23456;ii<23657;ii++)
	  {
	  if(((*((int *)(buf1+ii)))&0xff)==0xAA)
	  if(((*((int *)(buf1+ii+1)))&0xff)==0x55)
	  {			        yc_buf= ((*((int *)(buf1+ii+15)))&0xff);
                 	    yc_bufh= ((*((int *)(buf1+ii+16)))&0xff);
                        yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);     
                  //      pt_buf= (ptyc[15]&0xff)|((ptyc[16]&0xff)<<8);       
	  			//		printf("******1%d",yc_buf);       
	  					f=yc_buf;     
	  			//		f=pt_buf;
	  					f=f*0.01;
	  					yc_buf= ((*((int *)(buf1+ii+7)))&0xff);
	  					yc_bufh= ((*((int *)(buf1+ii+6)))&0xff);
	  					yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
     		//			printf("******%d",yc_buf);
     					gdj=yc_buf/100.0-180.0;
	  				 	yc_buf= ((*((int *)(buf1+ii+5)))&0xff);
	  					yc_bufh= ((*((int *)(buf1+ii+4)))&0xff);
	  					yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
     					fwj=yc_buf/100.0;
		 			 	yc_buf= ((*((int *)(buf1+ii+11)))&0xff);
	     				yc_bufh= ((*((int *)(buf1+ii+12)))&0xff);
         				jg_gd=((yc_bufh&0x00ff)<<8)|(yc_buf&0xff);
         				yc_buf= ((*((int *)(buf1+ii+17)))&0xff);  
         				if(yc_buf==0)
         				ir_or_ccd=0;
         				else if(yc_buf==0x01)
         				      ir_or_ccd=1;

	  break;
	  }
	  }
	  	  //get the platform data
	  if(frame>1)
	  {
	  for(ii=23456*2;ii<23657*2;ii++)
	  {
	  if(((*((int *)(buf1+ii)))&0xff)==0xAA)
	  if(((*((int *)(buf1+ii+1)))&0xff)==0x55)
	  {
			//		    printf("&&&&&&&&&&&");  =======================================================================================================
					    yc_buf= ((*((int *)(buf1+ii+15)))&0xff);
                 	    yc_bufh= ((*((int *)(buf1+ii+16)))&0xff);
                        yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);     
                  //      pt_buf= (ptyc[15]&0xff)|((ptyc[16]&0xff)<<8);       
	  					f=yc_buf;     
	  			//		f=pt_buf;
	  					f=f*0.01;
	  					yc_buf= ((*((int *)(buf1+ii+7)))&0xff);
	  					yc_bufh= ((*((int *)(buf1+ii+6)))&0xff);
	  					yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
     				//	printf("####%d",yc_buf);  
     					gdj=yc_buf/100.0-180.0;
	  				 	yc_buf= ((*((int *)(buf1+ii+5)))&0xff);
	  					yc_bufh= ((*((int *)(buf1+ii+4)))&0xff);
	  					yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
     					fwj=yc_buf/100.0;
		 			 	yc_buf= ((*((int *)(buf1+ii+11)))&0xff);
	     				yc_bufh= ((*((int *)(buf1+ii+12)))&0xff);
         				jg_gd=((yc_bufh&0x00ff)<<8)|(yc_buf&0xff);
         				yc_buf= ((*((int *)(buf1+ii+17)))&0xff);  
         				if(yc_buf==0)
         				ir_or_ccd=0;
         				else if(yc_buf==0x01)
         				      ir_or_ccd=1;

	  break;
	  }
	  }
	  }							
         }
		
}

/********************************************************************/

void SetConfigParms (void)
{
    SetCtrlVal (config_handle, CONFIG_COMPORT, comport);
    SetCtrlVal (config_handle, CONFIG_BAUDRATE, baudrate);
    SetCtrlVal (config_handle, CONFIG_PARITY, parity);
    SetCtrlVal (config_handle, CONFIG_DATABITS, databits);
    SetCtrlVal (config_handle, CONFIG_STOPBITS, stopbits);
    SetCtrlVal (config_handle, CONFIG_INPUTQ, inputq);
    SetCtrlVal (config_handle, CONFIG_OUTPUTQ, outputq);
    SetCtrlVal (config_handle, CONFIG_CTSMODE, ctsmode);
    SetCtrlVal (config_handle, CONFIG_XMODE, xmode);
    SetCtrlIndex (config_handle, CONFIG_COMPORT, portindex);
}


/********************************************************************/

void GetConfigParms (void)
{
    GetCtrlVal (config_handle, CONFIG_COMPORT, &comport);
    GetCtrlVal (config_handle, CONFIG_BAUDRATE, &baudrate);
    GetCtrlVal (config_handle, CONFIG_PARITY, &parity);
    GetCtrlVal (config_handle, CONFIG_DATABITS, &databits);
    GetCtrlVal (config_handle, CONFIG_STOPBITS, &stopbits);
    GetCtrlVal (config_handle, CONFIG_INPUTQ, &inputq);
    GetCtrlVal (config_handle, CONFIG_OUTPUTQ, &outputq);
    GetCtrlIndex (config_handle, CONFIG_COMPORT, &portindex);
    #ifdef _NI_unix_
        devicename[0]=0;
    #else
        GetLabelFromIndex (config_handle, CONFIG_COMPORT, portindex,
                       devicename);
    #endif                   
}


int CVICALLBACK ConfigCallBack (int panel, int control, int event,
                                void *callbackData, int eventData1,
                                int eventData2)
{
    switch (event)
        {
        case EVENT_COMMIT:
            config_handle = LoadPanel (panel_handle, "serial.uir", CONFIG);
            InstallPopup (config_handle);

            if (config_flag)    /* Configuration done at least once.*/
                SetConfigParms ();
            else                /* 1st time.*/
                config_flag = 1;
            break;
        case EVENT_RIGHT_CLICK :
            break;
        }
    return(0);
}


int CVICALLBACK CloseConfigCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :

            port_open = 0;  /* initialize flag to 0 - unopened */
            GetConfigParms ();

            DisableBreakOnLibraryErrors ();
            RS232Error = OpenComConfig (comport, devicename, baudrate, parity, databits, stopbits, inputq, outputq);
            EnableBreakOnLibraryErrors ();

            if (RS232Error) DisplayRS232Error ();

            if (RS232Error == 0) {

                port_open = 1;

                GetCtrlVal (config_handle, CONFIG_XMODE, &xmode);
                SetXMode (comport, xmode);

                GetCtrlVal (config_handle, CONFIG_CTSMODE, &ctsmode);
                SetCTSMode (comport, ctsmode);

                GetCtrlVal (config_handle, CONFIG_TIMEOUT, &timeout);
                SetComTime (comport, timeout);

                EnablePanelControls (0); /* Enable: no errors */
                }
            else
                EnablePanelControls (1); /* Disable: errors found */

            DiscardPanel (config_handle);
            break;

        }
    return(0);
}


/********************************************************************/

void EnablePanelControls (int enable)
{
    ActivateSendControls (enable);
}

/********************************************************************/

/*  Activate or deactivate the Send controls.  For activate, enable = 0,
    for deactivate, enable = 1, since 0 is not dimmed and 1 is dimmed.
*/
 void ActivateSendControls (int enable)
{
    if (send_mode) {   
        }
    else {             
         }
}


/********************************************************************/

int CVICALLBACK ClearBoxCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :

            break;
    }
    return(0);
}


/********************************************************************/

int CVICALLBACK SendModeCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            ActivateSendControls (0);
            break;
    }
    return(0);
}


/********************************************************************/

int CVICALLBACK FlushInCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            FlushInQ (comport);
            MessagePopup ("RS232 Message", "Input queue flushed.");
            break;
    }

    return(0);
}


/********************************************************************/

int CVICALLBACK FlushOutQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            FlushOutQ (comport);
            MessagePopup ("RS232 Message", "Output queue flushed.");
            break;
    }

    return(0);
}


/********************************************************************/

int CVICALLBACK GetInQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            inqlen = GetInQLen (comport);
            Fmt (msg, "%s<Input queue length = %i", inqlen);
            MessagePopup ("RS232 Message", msg);
            break;
    }

    return(0);
}
														   

/********************************************************************/

int CVICALLBACK GetOutQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            outqlen = GetOutQLen (comport);
            Fmt (msg, "%s<Output queue length = %i", outqlen);
            MessagePopup ("RS232 Message", msg);
            break;
    }

    return(0);
}


/********************************************************************/

int CVICALLBACK ComStatusCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            com_status = GetComStat (comport);
            DisplayComStatus ();
            break;
    }

    return(0);
}


/********************************************************************/

void DisplayComStatus ()
{
    com_msg[0] = '\0';
    if (com_status & 0x0001)
        strcat(com_msg, "Input lost: Input queue"
               " filled and characters were lost.\n");
    if (com_status & 0x0002)
        strcat(com_msg, "Asynch error: Problem "
               "determining number of characters in input queue.\n");
    if (com_status & 0x0010)
        strcat(com_msg, "Parity error.\n");
    if (com_status & 0x0020)
        strcat(com_msg, "Overrun error: Received"
               " characters were lost.\n");
    if (com_status & 0x0040)
        strcat(com_msg, "Framing error: Stop bits were not received"
               " as expected.\n");
    if (com_status & 0x0080)
        strcat(com_msg, "Break: A break signal was detected.\n");
    if (com_status & 0x1000)
        strcat(com_msg, "Remote XOFF: An XOFF character was received."
               "\nIf XON/XOFF was enabled, no characters are removed"
               " from the output queue and sent to another device "
               "until that device sends an XON character.\n");
    if (com_status & 0x2000)
        strcat(com_msg, "Remote XON: An XON character was received."
               "\nTransmisson can resume.\n");
    if (com_status & 0x4000)
        strcat(com_msg, "Local XOFF: An XOFF character was sent to\n"
               " the other device.  If XON/XOFF was enabled, XOFF is\n"
               " transmitted when the input queue is 50%, 75%, and 90%\n"
               " full.\n");
    if (com_status & 0x8000)
        strcat(com_msg, "Local XON: An XON character was sent to\n"
               " the other device.  If XON/XOFF was enabled, XON is\n"
               " transmitted when the input queue empties after XOFF\n"
               " was sent.  XON tells the other device that it can \n"
               " resume sending data.\n");
    if (strlen(com_msg) == 0)
        strcat(com_msg, "No status bits are set.");
    MessagePopup ("RS232 Message", com_msg);
}


/********************************************************************/

int CVICALLBACK SendCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT :
            if (send_mode)
                SendAscii ();
            else
            RS232Error = ReturnRS232Err ();
            if (RS232Error) DisplayRS232Error ();
            break;
        }
    return(0);
}


/********************************************************************/

void SendByte (void)
{												  
int m,n,cmm;
int xfx,yfx;

if(kzzt==0)
	{
	send_data[8]=send_buf[8];
	send_data[9]=send_buf[9];
	send_data[10]=send_buf[10];   
	}
	else
	{
	spx=-(adx-516-20);//control_fact;///16;
	xfx=spx/32;
	czy=(ady-516-16);//control_fact;//16;
	yfx=-czy/32;
	if(control_fact==6)
	control_fact1=f/16;
	else if(control_fact==2)
	     control_fact1=f/12;
	     else control_fact1=control_fact;
	if(control_fact1!=0)
	{
	spx=spx/control_fact1;
	if(spx<0)
	spx=4096+spx;
	czy=czy/control_fact1;
	if(czy<0)
	czy=4096+czy;
	}
	if(pt_ctr_status==4)
		{
		if(xfx<0)
		xfx=256+xfx;
		send_buf[10]=(xfx&0xff);
		send_buf[9]=0x00;
		if(yfx<0)
		yfx=256+yfx;
		send_buf[8]=(yfx&0xff); 
		}
	else
		{
		send_buf[8]=spx&0xff;
		send_buf[9]= (((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
		send_buf[10]=(czy&0xff0)>>4; 
		}
	send_data[8]=send_buf[8];
	send_data[9]=send_buf[9];
	send_data[10]=send_buf[10];
	}
	send_data[11]=0;
	for(cmm=2;cmm<11;cmm++)
	send_data[11]=send_data[11]+send_data[cmm];
	send_data[11]=send_data[11]&0xff;

    if(command_bit1!=0)
    {
    send_data[11]=send_buf[11];
	send_data[12]=send_buf[12];
	send_data[13]=send_buf[13];
	send_data[14]=send_buf[14];
	send_data[15]=send_buf[15];
	send_data[16]=send_buf[16];
	send_data[17]=send_buf[17];
    command_bit1--;
    }

	if(command_bit==0)
	{
	send_data[0]=0xeb;
	send_data[1]=0x90;   
	send_data[2]=0x64;
	send_data[3]=0x00;
	send_data[4]=0x0a;   
	send_data[5]=0x00;   
	send_data[6]=0x00;//send_data[4]&0x3f;   
	send_data[7]=0x00;//send_buf[7];   
	send_data[8]=0;//send_buf[8];   
	send_data[9]=0x00;//send_buf[9];   
	send_data[10]=0;
	send_data[11]=0;
	send_data[12]=0;
	send_data[13]=0;
	send_data[14]=0;
	send_data[15]=0;
	send_data[16]=0;
	send_data[17]=0;
	send_buf[14]=0;
	send_buf[15]=0;
	send_buf[16]=0;
	send_buf[17]=0;
//	}
	}

	if(command_bit1==0)
	{
	send_data[11]=0;
	send_data[12]=0;
	send_data[13]=0;
	send_data[14]=0;
	send_data[15]=0;
	send_data[16]=0;
	send_data[17]=0;
	send_buf[14]=0;
	send_buf[15]=0;
	send_buf[16]=0;
	send_buf[17]=0;
	}
}


/********************************************************************/

void SendAscii (void)
{
    switch (send_term_index) {
        case 1:
            strcat(send_data, "\r");
            break;
        case 2:
            strcat(send_data, "\n");
            break;
    }
    stringsize = StringLength (send_data);
    bytes_sent = ComWrt (comport, send_data, stringsize);
}


/********************************************************************/

int CVICALLBACK ErrorCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            RS232Error = ReturnRS232Err ();
            DisplayRS232Error ();
            break;

        case EVENT_RIGHT_CLICK :
            break;

    }
    return(0);
}


/********************************************************************/

int CVICALLBACK ReadCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_COMMIT:
            read_data[0] = '\0';
            switch (read_term_index) {
                case 0:
                    read_term = 0;
                    break;
                case 1:
                    read_term = 13;
                    break;
                case 2:
                    read_term = 10;
                    break;
            }
            if (read_term)
                bytes_read = ComRdTerm (comport, read_data, read_cnt, read_term);
            else
                bytes_read = ComRd (comport, read_data, read_cnt);
            CopyString (tbox_read_data, 0, read_data, 0, bytes_read);
            RS232Error = ReturnRS232Err ();
            if (RS232Error) DisplayRS232Error ();
            break;

        case EVENT_RIGHT_CLICK :
            break;
    }
    return(0);
}

void ReadByte(void)
{
int nbyte;
char buf[4096];
int readbuf,strlen,yc_buf,yc_bufh,pt_buf,pt_bufh;

ptds_bj=1;
portstatus=GetInQLen (comport);
while(portstatus!=0)
	{
	readbuf= ComRdByte (comport); 
if(readbj==0)
{
    if(readbuf==0xeb)
	{
	readbj=1;ptyc[0]=0xeb;ri++;
	}
}	
else
	{
	ptyc[ri]=readbuf;
	if(ri<127)
	ri++;
	else {readbj=0;ri=0;}
	}

	portstatus=GetInQLen(comport); 
}
}


void yc_data_js(void)
{
int yc_buf,yc_bufh,pt_buf,pt_bufh;   
						if((ptyc[0]==0xeb)&&(ptyc[1]==0x90))
						{
						 yc_buf=ptyc[4];
                 	     yc_bufh= ptyc[5];
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     fy=yc_buf*0.1;
                 	     yc_buf= ptyc[6];
                 	     yc_bufh=ptyc[7];
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     qx=yc_buf*0.1;
                         hx=(ptyc[8]|(ptyc[9]<<8))*0.1+adjust_heading;
                         gps_hx=(ptyc[53]|(ptyc[54]<<8))*0.1; 
                         yc_buf= ptyc[18];
                 	     yc_bufh= ptyc[19];
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh=yc_buf&0x8000;
                 	     if(yc_bufh==0x8000)
                 	     {
                 	     yc_buf=0xffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     gd=yc_buf*0.5-gd3;
                 	     //GPS
                 	     yc_buf= ptyc[20];
                 	     yc_bufh= ptyc[21];
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh= ptyc[22];
                 	     yc_buf=(yc_buf&0xffff)|((yc_bufh&0x00ff)<<16);
                 	     yc_bufh= ptyc[23];
                 	     yc_buf=(yc_buf&0xffffff)|((yc_bufh&0x00ff)<<24);
                 	     yc_bufh=yc_buf&0x80000000;
                 	     if(yc_bufh==0x80000000)
                 	     {
                 	     yc_buf=0xffffffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     glon=yc_buf*0.000001;
                 	     yc_buf= ptyc[24];
                 	     yc_bufh= ptyc[25];
                 	     yc_buf=(yc_buf&0xff)|((yc_bufh&0x00ff)<<8);
                 	     yc_bufh= ptyc[26];
                 	     yc_buf=(yc_buf&0xffff)|((yc_bufh&0x00ff)<<16);
                 	     yc_bufh= ptyc[27];
                 	     yc_buf=(yc_buf&0xffffff)|((yc_bufh&0x00ff)<<24);
                 	     yc_bufh=yc_buf&0x80000000;
                 	     if(yc_bufh==0x80000000)
                 	     {
                 	     yc_buf=0xffffffff-yc_buf+1;
                 	     yc_buf=yc_buf*(-1);
                 	     }
                 	     glat=yc_buf*0.000001;
						//014 pintai
                        pt_buf= (ptyc[111]&0xff)|((ptyc[112]&0xff)<<8);
	  					f=pt_buf;
	  					f=f*0.01;
	  					pt_buf= ptyc[101];
	  					pt_bufh= ptyc[102];
	  					pt_buf=(pt_buf&0xff)|((pt_bufh&0x00ff)<<8);
     					gdj=pt_buf/100.0-180;
	  					pt_buf= ptyc[99];
	  					pt_bufh= ptyc[100];
	  					pt_buf=(pt_buf&0xff)|((pt_bufh&0x00ff)<<8);
     					fwj=pt_buf/100.0;
		 				pt_buf= ptyc[107];
	     				pt_bufh=ptyc[108];
         				jg_gd=((pt_bufh&0x00ff)<<8)|(pt_buf&0xff);
						}

}
/********************************************************************/

int CVICALLBACK InputQCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
    switch (event) {
        case EVENT_RIGHT_CLICK :
            DisplayHelp (InputqHelp);
            break;
    }
    return(0);
}


/********************************************************************/

int CVICALLBACK QuitCallBack(int panel, int control, int event, void *callbackData, int eventData1, int eventData2)
{
	int stopi;
    switch (event) {
        case EVENT_COMMIT :
			ptds_bj=0;
			Agree=0;
			startOK=0;
            if (port_open) {
                outqlen = GetOutQLen (comport);
                if (outqlen > 0) {
                    MessagePopup ("RS232 Message", "The output queue has\n"
                                    "data in it. Wait for device to receive\n"
                                    "the data or flush the queue.\n");
                    break;
                    }
                RS232Error = CloseCom (comport);
                if (RS232Error) DisplayRS232Error ();
            }
			   /* Free all allocated buffers. */
		   for (n=0; n<NbFrames; n++)
		      {
		      }
	     if(tableRS==1)
			{
				// Stop the IMAQ Loop
				SetCtrlAttribute (panel_handle, SERIAL_TIMER, ATTR_ENABLED, FALSE);
	
				// Stop the acquisition
				imgSessionStopAcquisition (Sid);
		
			}
             QuitUserInterface (0);
            break;

        case EVENT_RIGHT_CLICK :
            DisplayHelp (QuitHelp);
            break;

    }
    return(0);
}


/********************************************************************/

void DisplayRS232Error (void)
{
    char ErrorMessage[200];
    switch (RS232Error) {									   
        default :
            if (RS232Error < 0) {   /* Bug? ComWrtByte sets rs232error if sent a byte out? */
                Fmt (ErrorMessage, "%s<RS232 error number %i", RS232Error);
                MessagePopup ("RS232 Message", ErrorMessage);
                }
            break;

        case 0  :
            MessagePopup ("RS232 Message", "No errors.");
            break;

        case -2 :
            Fmt (ErrorMessage, "%s", "Invalid port number (must be in the range 1 to 8).");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;

        case -3 :
            Fmt (ErrorMessage, "%s", "No port is open.\n"
                 "Check COM Port setting in Configure.");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;
        case -99 :
            Fmt (ErrorMessage, "%s", "Timeout error.\n\n"
                 "Either increase timeout value,\n"
                 "       check COM Port setting, or\n"
                 "       check device.");

            MessagePopup ("RS232 Message", ErrorMessage);
            break;

    }
}


/********************************************************************/

void DisplayHelp(int HelpId)
{
    switch (HelpId) {
        case 1 :
            MessagePopup ("Quit Help",
                          "The Quit button closes the current RS232 COM port,\n"
                          "checks and displays any error messages, \n"
                          "and then exits this program.");
            break;
        case 2 :
            MessagePopup ("Input Queue Help",
                          "Specifies the size of the input queue for the selected port.\n"
                          "Default Value:  512\n"
                          "Valid Range:    28 to 65516");
            break;
    }
}


int CVICALLBACK Standby (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			send_buf[4]=0xe9;
			send_buf[5]=0x00;
			send_buf[6]=0x00;//;(send_buf[4]&0xe0)|0x04;		//switch1 pressed for at least one second & heading hold
			send_buf[7]=0x40;
			send_buf[8]=0x00;
			send_buf[9]=0x00;
			kzzt=0;  
			command_bit=2;
			ChangeData();
			break;
	}
	return 0;
}


int CVICALLBACK saveimage (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int systime;
	switch (event)
		{
		case EVENT_COMMIT:
		  if(controlimage==1)
		   {
		       systime = GetSystemDate (&month, &day, &year);
	           systime = GetSystemTime(&hours, &minutes, &seconds);
	           Fmt(filename1,"d:\\temp1\\%d%d%d%d%d%d.bmp",year,month,day,hours,minutes,seconds);
	           imaqMergeOverlay(image,image,NULL,0,NULL);
			   imaqWriteFile(image,filename1,NULL);
		   if(*mapbuf==1)
             {

                imaqReadFile (image, "d:\\temp\\index1.bmp", NULL, NULL); 
				Drawimage();
                imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	

             }
           else if(*mapbuf==2)
             {

                imaqReadFile (image, "d:\\temp\\index2.bmp", NULL, NULL); 
				Drawimage();
                imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
             }	
		   }	
			break;
		}
	return 0;
}



int CVICALLBACK showimage (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
FILE *fp;
int filehandle;
   int stopi0;
	switch (event)
		{
		case EVENT_COMMIT:
		 GetCtrlVal (panel_handle, SERIAL_TOGGLEBUTTON_2, &stopi0);
		 switch(stopi0)
		  {
		   case 1: 
		 
			 selstatus = FileSelectPopup ("d:\\temp1", "*.bmp", "", "Select an Image", VAL_SELECT_BUTTON, 0,
										  0, 1, 0, pathname);
		     if(selstatus>0)
		       {
		          imaqReadFile (image, pathname, NULL, NULL);
		          imaqDisplayImage (image, WINDOW_TO_USE, TRUE);
		          SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 0);
		          selstatus0=0;
		          controlimage=1;
	              flag=0;
	              testx[0]=0;
	              testx[1]=0;
	              testy[0]=0;
	              testy[1]=0;
	              imaqSetEventCallback (ExtractImage, TRUE);  
    	       }
		     else
		       {
		        
		          SetCtrlVal (panel_handle, SERIAL_TOGGLEBUTTON_2, 0);
		          SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 1);
		          selstatus0=1;
		          controlimage=0;
	              flag=1;
		       }   
			
			 break;
		   case 0:
			 SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 1);
			 selstatus0=1;
			 selstatus=0;
		     controlimage=0;
	         flag=1;
		     break;
		  }
	   }
	return 0;
}


void drawinitial(void)
{	
	SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,6000156);	//green 
	pointVal3 = MakePoint (0,130);
    pointVal4 = MakePoint (260,130);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);  
	pointVal3 = MakePoint (130,0);
    pointVal4 = MakePoint (130,260);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	
	pointVal3 = MakePoint (125,32);
    pointVal4 = MakePoint (135,32);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);  
	pointVal3 = MakePoint (125,65);
    pointVal4 = MakePoint (135,65);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	pointVal3 = MakePoint (125,97);
    pointVal4 = MakePoint (135,97);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);  
	pointVal3 = MakePoint (125,162);
    pointVal4 = MakePoint (135,162);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	pointVal3 = MakePoint (125,195);
    pointVal4 = MakePoint (135,195);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	pointVal3 = MakePoint (125,227);
    pointVal4 = MakePoint (135,227);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	
	pointVal3 = MakePoint (32,125);
    pointVal4 = MakePoint (32,135);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);  
	pointVal3 = MakePoint (65,125);
    pointVal4 = MakePoint (65,135);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	pointVal3 = MakePoint (97,125);
    pointVal4 = MakePoint (97,135);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);  
	pointVal3 = MakePoint (162,125);
    pointVal4 = MakePoint (162,135);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	pointVal3 = MakePoint (195,125);
    pointVal4 = MakePoint (195,135);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	pointVal3 = MakePoint (227,125);
    pointVal4 = MakePoint (227,135);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal3, pointVal4);
	
	circle.left = 65;
    circle.top = 65;
    circle.height = 130;
    circle.width = 130;
	CanvasDrawOval (panel_handle, SERIAL_CANVAS_2, circle,  VAL_DRAW_FRAME);
	circle.left =0;
    circle.top = 0;
    circle.height = 260;
    circle.width = 260;
	CanvasDrawOval (panel_handle, SERIAL_CANVAS_2, circle,  VAL_DRAW_FRAME);
	
	//画十字叉
	SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0xffffff);
	pointVal1 = MakePoint (linex-5,liney);
    pointVal2 = MakePoint (linex+5,liney);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
	pointVal1 = MakePoint (linex,liney-5);
    pointVal2 = MakePoint (linex,liney+5);
	CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
	//写方向
	SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_FILL_COLOR, VAL_BLACK);
	SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,6000156);
    CreateMetaFont("VAL_3SEG_META_FONT",VAL_APP_META_FONT,15,1,0,0,0);
	
	CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, "W", "VAL_3SEG_META_FONT",
						   MakePoint(7,130), VAL_CENTER);
	CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, "E", "VAL_3SEG_META_FONT",
						   MakePoint(253,130), VAL_CENTER);
	CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, "N", "VAL_3SEG_META_FONT",
						   MakePoint(130,7), VAL_CENTER);
	CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, "S", "VAL_3SEG_META_FONT",
						   MakePoint(130,253), VAL_CENTER);
	GetCtrlVal (panel_handle, SERIAL_NUMERIC_22, &mbnum); 
	CreateMetaFont("VAL_4SEG_META_FONT",VAL_APP_META_FONT,11,0,0,0,0); 
    Fmt(mbstring,"Target%d:",mbnum); 
    SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_COLOR, VAL_YELLOW); 
    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, mbstring, "VAL_4SEG_META_FONT",
						   MakePoint(3,10), VAL_CENTER_LEFT);
	
	scalenum=sqrt(((xs-yzmbx)*(xs-yzmbx)+(ys-yzmby)*(ys-yzmby))/(800*800));
	scalenum+=1;
	SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000); 
	CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
						   MakePoint(250,10), VAL_CENTER_RIGHT);
	drawcanvas(scalenum);							   
}



void Drawimage(void)
{
  Point start,end1,end,end2;
  float jiaodu;
  char mbstring[20];
            imaqGetMousePos (&position, WINDOW_TO_USE);
        //  图象上写图象收容范围
            if(controlimage==0)   //非图象冻结状态计算图象收容范围 
			{
			image_x=gd*4.8/f;
			image_y=gd*3.6/f;
			}
            else
            {
            image_x=dj_gd*4.8/dj_f;
			image_y=dj_gd*3.6/dj_f;
			}
			Fmt(mbstring,"%f[p0]",image_x);
			imaqOverlayRect(image,MakeRect(545,6,30,105),&(IMAQ_RGB_GRAY),0,NULL); 
			imaqOverlayText (image, MakePoint(10,550), mbstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			Fmt(mbstring,"%f[p0]",image_y);
			imaqOverlayText (image, MakePoint(65,550), mbstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			
			//图象上写目标坐标
			Fmt(mbstring,"%d",xt);
			imaqOverlayText (image, MakePoint(210,3), "target",&(IMAQ_RGB_GRAY), &options, NULL);
			imaqOverlayRect(image,MakeRect(0,3,40,190),&(IMAQ_RGB_GRAY),0,NULL); 
			imaqOverlayText (image, MakePoint(8,3), mbstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			Fmt(mbstring,"%d",yt);
			imaqOverlayText (image, MakePoint(115,3), mbstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			imaqOverlayText (image, MakePoint(8,23), targetstringx,&(IMAQ_RGB_GRAY), &options, NULL);	
			imaqOverlayText (image, MakePoint(115,23), targetstringy,&(IMAQ_RGB_GRAY), &options, NULL);					 
			//图象上写偏差坐标 				 
		
			Fmt(pcstring,"%d",pcx);
			imaqOverlayRect(image,MakeRect(0,260,40,190),&(IMAQ_RGB_GRAY),0,NULL); 
			imaqOverlayText (image, MakePoint(268,3), pcstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			Fmt(pcstring,"%d",pcy);
			imaqOverlayText (image, MakePoint(110+260,3), pcstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			imaqOverlayText (image, MakePoint(268,23), pcstringx,&(IMAQ_RGB_GRAY), &options, NULL);	
			imaqOverlayText (image, MakePoint(110+260,23), pcstringy,&(IMAQ_RGB_GRAY), &options, NULL);					 
			//写标记
			if(gd_get_status==0)
			imaqOverlayText (image, MakePoint(310,553), "PR.Alt",&(IMAQ_RGB_GRAY), &options, NULL);
			else imaqOverlayText (image, MakePoint(310,553), "Laser Alt",&(IMAQ_RGB_GRAY), &options, NULL);
			if(mborzd==1)
			imaqOverlayText (image, MakePoint(213,553), "target",
							 &(IMAQ_RGB_GRAY), &options, NULL);
			else imaqOverlayText (image, MakePoint(213,553), "Burst",
							 &(IMAQ_RGB_GRAY), &options, NULL);
			if(DWORCT==0)
			imaqOverlayText (image, MakePoint(513,553), "Mapping",
							 &(IMAQ_RGB_GRAY), &options, NULL);
			else imaqOverlayText (image, MakePoint(513,553), "Location",
							 &(IMAQ_RGB_GRAY), &options, NULL);
			//写高度，平台角度
			if(controlimage==0)   //非图象冻结状态写高度和平台角度 
			{
				Fmt(gdjdstring,"Alt:%f",dj_gd); 
				imaqOverlayText (image, MakePoint(10,400),gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
				Fmt(gdjdstring,"Roll:%f",qx); 
				imaqOverlayText (image, MakePoint(10,430), gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
				Fmt(gdjdstring,"Elevation:%f",gdj); 
				imaqOverlayText (image, MakePoint(10,460),gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
			}
            else
            {
            	Fmt(gdjdstring,"Alt:%f",dj_gd); 
            	imaqOverlayText (image, MakePoint(10,400), gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
				Fmt(gdjdstring,"Roll:%f",dj_qx[djnum]); 
				imaqOverlayText (image, MakePoint(10,430), gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
				Fmt(gdjdstring,"Elevation:%f",dj_gdj[djnum]); 
				imaqOverlayText (image, MakePoint(10,460), gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
			}
			//write ding wei piancha
			Fmt(gdjdstring,"pcx::%d",wucha_x); 
            imaqOverlayText (image, MakePoint(10,490), gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);
			Fmt(gdjdstring,"pcy:%d",wucha_y); 
			imaqOverlayText (image, MakePoint(10,520), gdjdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);		 
			//图象上写炸点坐标
 
			Fmt(zdstring,"%d",xz);
			imaqOverlayText (image, MakePoint(460,3), "Burst",
							 &(IMAQ_RGB_GRAY), &options, NULL);
			imaqOverlayRect(image,MakeRect(0,520,40,190),&(IMAQ_RGB_GRAY),0,NULL); 
			imaqOverlayText (image, MakePoint(525,3), zdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			Fmt(zdstring,"%d",yz);
			imaqOverlayText (image, MakePoint(620,3), zdstring,
							 &(IMAQ_RGB_GRAY), &options, NULL);	
			
			imaqOverlayText (image, MakePoint(525,23), zdstringx,&(IMAQ_RGB_GRAY), &options, NULL);	
			imaqOverlayText (image, MakePoint(620,23), zdstringy,&(IMAQ_RGB_GRAY), &options, NULL);					 
			
			//画指北标记
			if(controlimage==1)
			jiaodu=(dj_fwj[djnum]+dj_hx[djnum])*3.14/180.0;
			else
			jiaodu=(fwj+hx)*3.14/180.0;
			
			if(jiaodu>6.28)
			jiaodu=jiaodu-6.28;
			if(jiaodu<=1.57)
			{
			start.y=ym3;
			start.x=xm3;
			}
			else if(jiaodu<=3.14) 
			{
			start.y=ym3;
			start.x=xm3;
			}
			else if(jiaodu<=4.71)    
			{start.y=ym3;
			start.x=xm3;
			}
			else  if(jiaodu<=6.28)    
			{start.y=ym3;
			start.x=xm3;
			}
			else
			{
			jiaodu=jiaodu-6.28;
			start.y=ym3;
			start.x=xm3;
			}
			imaqOverlayText (image, start, "N",
							 &(IMAQ_RGB_GRAY), &options, NULL);	
		//画中心十字	
			imaqOverlayRect(image,MakeRect(centerY/2-2,centerX/2-35,4,25),&(IMAQ_RGB_GRAY0),0,NULL);
			imaqOverlayRect(image,MakeRect(centerY/2-2,centerX/2+10,4,25),&(IMAQ_RGB_GRAY0),0,NULL);
			imaqOverlayRect(image,MakeRect(centerY/2-35,centerX/2-2,25,4),&(IMAQ_RGB_GRAY0),0,NULL);
			imaqOverlayRect(image,MakeRect(centerY/2+10,centerX/2-2,25,4),&(IMAQ_RGB_GRAY0),0,NULL);
		//画屏幕坐标
			Fmt(zbstring,"Screen(%d, %d)",position.x,position.y);
			imaqOverlayText (image, MakePoint(5,287), zbstring,&(IMAQ_RGB_GRAY), &options, NULL);
            //画弹道线
            start.y=testy[2];
			start.x=testx[2];
			end.y=ym2;
			end.x=xm2;
}


int CVICALLBACK timeok(int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{

int exbuf,xpzs;
int x,y,x1,y1;
int filen,systime;
float zcbuf1,zcbuf11,zcbuf2,zcbuf22,zcbuf3,zcbuf33;
double x2buf,y2buf;
int du,shi,fen,miao;
float t1,t2,s1,s2,p1,p2;
float kk;
int mm;
int phandle,globalx,globaly,leftdown,rightdown;
int result;											                    
	
	switch (event) {
		case EVENT_TIMER_TICK:
		 if(flag)		//flat是否进入到画图
		  {
		    if(!selstatus0)
		       imaqBringWindowToTop (WINDOW_TO_USE);
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
			if(startOK==1)
			{
			if(sendbit==0)			 //每80ms通过串口发送数据一次
			{
			}
			}
			
		newcount+=1;
		if(newcount>=44&&datasource==0)
		  {
		    SetCtrlVal (panel_handle, SERIAL_LED_3, 0);
		  }
		else if(newcount>=44&&datasource==2)
		  {
		    SetCtrlVal (panel_handle, SERIAL_LED_2, 0);
		  }
    if(datasource==2)
	 {
	   if(controlimage==0)
	     {
		  imaqGetWindowSize(WINDOW_TO_USE,&centerX,&centerY); 
            yc_data_js();
		    if(portstatus!=0)
		      {
		         if(ledstatus1==0)
		          {
		            ledstatus1=1;
		            newcount=0;
		            SetCtrlVal (panel_handle, SERIAL_LED_2, 0);
		          }
		        else
		          {
		            ledstatus1=0;
		            newcount=0;
		            SetCtrlVal (panel_handle, SERIAL_LED_2, 1);
		          }  
		      }
		    imgGrab (Sid, (void **)&ImaqBuffer, FALSE);  //采集图像 
        	imgSessionSaveBufferEx(Sid,ImaqBuffer,"D:\\temp2\\22.bmp"); //将采集的图像存储到"D:\\temp2\\22.bmp"
			imaqReadFile (image, "D:\\temp2\\22.bmp", NULL, NULL);  //从"D:\\temp2\\22.bmp"打开图像
			Drawimage();
	    	imaqDisplayImage (image, WINDOW_TO_USE, TRUE); //显示图像
					   
		 }
	   else if(controlimage==1)
	     {
	      imaqSetEventCallback (ExtractImage, TRUE);
	      flag=0; 
 
	     }
			
	}
    else if(datasource==0)
	 {
		if(controlimage==0)	
		 {
		   imaqGetWindowSize(WINDOW_TO_USE,&centerX,&centerY);
/*		   if(*mapbuf==1)
    		{
    			 display_status=1;
    	         imaqReadFile (image, "d:\\temp\\index1.bmp", NULL, NULL); 
    			  Drawimage();
    		     imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
    		
    		}
          else if(*mapbuf==2)
    		{
    			  display_status=0;
    		     imaqReadFile (image, "d:\\temp\\index2.bmp", NULL, NULL); 
    			  Drawimage();
    		     imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
    		}	 							 	*/  
    	  }										
    	else
    	 {
    	 
    	  imaqSetEventCallback (ExtractImage, TRUE);
    	  flag=0;
    	 }	
      }
     } 
    else if(flag==0)
	    {
	       if(!selstatus0)
	         imaqBringWindowToTop (WINDOW_TO_USE);
	       if(valuei==1)
		        {
				    imaqClearOverlay(image,NULL);
				    if(selstatus==0)
				       Drawimage();
		            imaqGetMousePos (&position, WINDOW_TO_USE);
		            testx[1]=position.x;
			        testy[1]=position.y;
				    imaqOverlayLine(image,MakePoint(testx[0],testy[0]), MakePoint(testx[1],testy[1]),&IMAQ_RGB_GRAY0,NULL);
				    GetGlobalMouseState(&phandle,&globalx,&globaly,&leftdown,&rightdown,0);
				    if(rightdown==1)
				      {
				        imaqClearOverlay(image,NULL);
				        if(selstatus==0)
				           Drawimage();
				        testx[1]=testx[0];
				        testy[1]=testy[0];
				        valuei=0;
				        valuej=0;
				      } 
		        }
		     else
		        {
		          imaqClearOverlay(image,NULL);
				  imaqOverlayLine(image,MakePoint(testx[0],testy[0]), MakePoint(testx[1],testy[1]),&IMAQ_RGB_GRAY0,NULL);
				  if(DWORCT==1&&mborzd==1)
				     {
				        GetGlobalMouseState(&phandle,&globalx,&globaly,&leftdown,&rightdown,0);
				        if(rightdown==1)
				          {
				             imaqDrawLineOnImage (image, image,IMAQ_DRAW_VALUE, MakePoint(position1.x-5,position1.y),
				                         MakePoint(position1.x+5,position1.y),MakeColor(163,243,139));
				             imaqDrawLineOnImage (image, image,IMAQ_DRAW_VALUE, MakePoint(position1.x,position1.y-5),
				                         MakePoint(position1.x,position1.y+5),MakeColor(163,243,139));   
				          }
				     }    
		           if(selstatus==0)
		            { 
				      Drawimage();
				    }   
				   imaqDisplayImage (image, WINDOW_TO_USE, TRUE); 
		        }
		    
	    }    
    	 
    	   if(controlimage==0)
    	   	   {
    	   	        djnum=djnum1;
    	   	      if(djnum<20)
    	   	        {
                    }
                  else
                    {
                         djnum=0;
                         djnum1=0;
                    }
                   
                djnum1++;    
              } 
          else
              djnum=djnum;
            SetCtrlVal (panel_handle, SERIAL_NUMERICMETER_2, fy);
				//十字叉消影
			if(controlimage==0)   //非图象冻结状态 
			{
			testx[2]=360;
			testy[2]=288;
			testx[3]=360;
			testy[3]=288;
			}
			if(sendbit==0)
			{
				SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000);	//BLACK
			
			pointVal1 = MakePoint (linex-5,liney);//(randLeft, randTop+randHeight); 
            pointVal2 = MakePoint (linex+5,liney);//randLeft+randWidth, randTop);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			pointVal1 = MakePoint (linex,liney-5);//(randLeft, randTop+randHeight); 
            pointVal2 = MakePoint (linex,liney+5);//randLeft+randWidth, randTop);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			
			//目标，目标特征文字信息消影
			CreateMetaFont("VAL_4SEG_META_FONT",VAL_APP_META_FONT,11,0,0,0,0);
//			CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, mbstring, "VAL_4SEG_META_FONT",
//								   MakePoint(3,10), VAL_CENTER_LEFT);
			CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, Timestring, "VAL_4SEG_META_FONT",
											   MakePoint(3,250), VAL_CENTER_LEFT);
			CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, Datastring, "VAL_4SEG_META_FONT",
											   MakePoint(250,250), VAL_CENTER_RIGHT);					   
								   
		   //目标消影	
			pointVal1 = MakePoint (distancex-5,distancey);
            pointVal2 = MakePoint (distancex+5,distancey);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			pointVal1 = MakePoint (distancex,distancey-5);
            pointVal2 = MakePoint (distancex,distancey+5);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);					   
								   
			//飞机图标消影
			x=130-8*sin(fjhx+1.57);
			y=130-8*cos(fjhx+1.57);
			x1=130+8*sin(fjhx+1.57);
			y1=130+8*cos(fjhx+1.57);
			pointVal1 = MakePoint (x,y);
            pointVal2 = MakePoint (x1,y1);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			x=130-12*sin(fjhx);
			y=130-12*cos(fjhx);
			x1=130+12*sin(fjhx);
			y1=130+12*cos(fjhx);
			pointVal1 = MakePoint (x,y);
            pointVal2 = MakePoint (x1,y1);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			x=x1-3*sin(fjhx+1.57);
			y=y1-3*cos(fjhx+1.57);
			x1=x1+3*sin(fjhx+1.57);
			y1=y1+3*cos(fjhx+1.57);
			pointVal1 = MakePoint (x,y);
            pointVal2 = MakePoint (x1,y1);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			
			x=130+20*cos(pthxjiao);
			y=130+20*sin(pthxjiao);
			pointVal1 = MakePoint (130,130);
            pointVal2 = MakePoint (x,y);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
		
			//画线
			x=ady-515;
			linex=200-x*0.45;
			y=adx-515;
			liney=200-y*0.45;
			
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_22, &mbnum);
			//写目标特征和目标几
//			 SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_COLOR, VAL_YELLOW);

//			 Fmt(mbstring,"Target%d:",mbnum);
//			 CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, mbstring, "VAL_4SEG_META_FONT",
//								   MakePoint(3,10), VAL_CENTER_LEFT);
      
      		drawinitial();
								  
			//画飞机图标
			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,6750156);	//BLUE 
			fjhx=-hx*3.14/180;
			x=130-8*sin(fjhx+1.57);
			y=130-8*cos(fjhx+1.57);
			x1=130+8*sin(fjhx+1.57);
			y1=130+8*cos(fjhx+1.57);
			pointVal1 = MakePoint (x,y);//(randLeft, randTop+randHeight); 
            pointVal2 = MakePoint (x1,y1);//randLeft+randWidth, randTop);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			
			x=130-12*sin(fjhx);
			y=130-12*cos(fjhx);
			x1=130+12*sin(fjhx);
			y1=130+12*cos(fjhx);
			pointVal1 = MakePoint (x,y);
            pointVal2 = MakePoint (x1,y1);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			
			x=x1-3*sin(fjhx+1.57);
			y=y1-3*cos(fjhx+1.57);
			x1=x1+3*sin(fjhx+1.57);
			y1=y1+3*cos(fjhx+1.57);
			pointVal1 = MakePoint (x,y);
            pointVal2 = MakePoint (x1,y1);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			
			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,VAL_RED);	
			pthxjiao=(-90+fwj+hx)*3.14/180;
			x=130+20*cos(pthxjiao);
			y=130+20*sin(pthxjiao);
			pointVal1 = MakePoint (130,130);
            pointVal2 = MakePoint (x,y);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			
			//判断目标是否出现
            if(sqrt((xs-yzmbx1)*(xs-yzmbx1)+(ys-yzmby1)*(ys-yzmby1))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_1, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_1, 0);
            if(sqrt((xs-yzmbx2)*(xs-yzmbx2)+(ys-yzmby2)*(ys-yzmby2))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_2, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_2, 0);
			if(sqrt((xs-yzmbx3)*(xs-yzmbx3)+(ys-yzmby3)*(ys-yzmby3))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_3, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_3, 0);
			if(sqrt((xs-yzmbx4)*(xs-yzmbx4)+(ys-yzmby4)*(ys-yzmby4))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_4, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_4, 0);
			if(sqrt((xs-yzmbx5)*(xs-yzmbx5)+(ys-yzmby5)*(ys-yzmby5))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_5, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_5, 0);
			if(sqrt((xs-yzmbx6)*(xs-yzmbx6)+(ys-yzmby6)*(ys-yzmby6))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_6, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_6, 0);
			if(sqrt((xs-yzmbx7)*(xs-yzmbx7)+(ys-yzmby7)*(ys-yzmby7))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_7, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_7, 0);
			if(sqrt((xs-yzmbx8)*(xs-yzmbx8)+(ys-yzmby8)*(ys-yzmby8))<800*scalenum)
			  {
			    SetCtrlVal (panel_handle, SERIAL_YZMB_8, 1);
			  }
			else 
			  SetCtrlVal (panel_handle, SERIAL_YZMB_8, 0);
			  
			switch(mbnum)
			  {
			    case 1:
			       if(sqrt((xs-yzmbx1)*(xs-yzmbx1)+(ys-yzmby1)*(ys-yzmby1))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx1)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby1)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx1);
			           distancex=(ys-yzmby1);
			         }
			       break; 
			   case 2:
			         if(sqrt((xs-yzmbx2)*(xs-yzmbx2)+(ys-yzmby2)*(ys-yzmby2))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx2)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby2)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx2);
			           distancex=(ys-yzmby2);
			         }  
			       break; 
			  case 3:
			       if(sqrt((xs-yzmbx3)*(xs-yzmbx3)+(ys-yzmby3)*(ys-yzmby3))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx3)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby3)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx3);
			           distancex=(ys-yzmby3);
			         }  
			       break; 
			  case 4:
			       if(sqrt((xs-yzmbx4)*(xs-yzmbx4)+(ys-yzmby4)*(ys-yzmby4))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx4)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby4)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx4);
			           distancex=(ys-yzmby4);
			         }  
			       break; 
			  case 5:
			       if(sqrt((xs-yzmbx5)*(xs-yzmbx5)+(ys-yzmby5)*(ys-yzmby5))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx5)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby5)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx5);
			           distancex=(ys-yzmby5);
			         }  
			       break; 
			  case 6:
			       if(sqrt((xs-yzmbx6)*(xs-yzmbx6)+(ys-yzmby6)*(ys-yzmby6))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx6)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby6)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx6);
			           distancex=(ys-yzmby6);
			         }  
			       break; 
			  case 7:
			       if(sqrt((xs-yzmbx7)*(xs-yzmbx7)+(ys-yzmby7)*(ys-yzmby7))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx7)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby7)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx7);
			           distancex=(ys-yzmby7);
			         }  
			       break; 
			  case 8:
			       if(sqrt((xs-yzmbx8)*(xs-yzmbx8)+(ys-yzmby8)*(ys-yzmby8))<800*scalenum)
			         {
			           distancey=(((xs-yzmbx8)/scalenum)*130)/800+130;
			           distancex=(((ys-yzmby8)/scalenum)*130)/800+130;
			         }
			       else
			         {
			           distancey=(xs-yzmbx8);
			           distancex=(ys-yzmby8);
			         }  
			       break; 
			 
			  }
			  
	    	//画目标
			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,MakeColor(175,209,201));
		    pointVal1 = MakePoint (distancex-5,distancey);//(randLeft, randTop+randHeight); 
            pointVal2 = MakePoint (distancex+5,distancey);//randLeft+randWidth, randTop);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);
			pointVal1 = MakePoint (distancex,distancey-5);
            pointVal2 = MakePoint (distancex,distancey+5);
			CanvasDrawLine (panel_handle, SERIAL_CANVAS_2, pointVal1, pointVal2);	
			}
			systime = GetSystemTime(&hours, &minutes, &seconds);
			systime = GetSystemDate (&month, &day, &year);
			Fmt(Datastring,"%d/%d/%d",year,month,day);
			Fmt(Timestring,"%d:%d:%d",hours,minutes,seconds);
			SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_COLOR, VAL_YELLOW);
			CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, Timestring, "VAL_4SEG_META_FONT",
											   MakePoint(3,250), VAL_CENTER_LEFT);
			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,VAL_GREEN);
		    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, Datastring, "VAL_4SEG_META_FONT",
											   MakePoint(250,250), VAL_CENTER_RIGHT);
			
            GetGlobalMouseState (NULL, &globalX, &globalY, &leftBtn, &rightBtn,
                             &keyStates);

			//get yc data
			exbuf=ReadFromPhysicalMemory(0xd0500,keybuf1,1);
			key1=*keybuf1;

			for(x=0;x<32;x++)
				{
				exbuf=ReadFromPhysicalMemory((0xd0000+x),keybuf1,1);//
				yc1[x]=*keybuf1;
				yc1[x]=yc1[x]&0xff;
				}
			//receive lj fw data
			for(x=0;x<4;x++)
				{
				exbuf=ReadFromPhysicalMemory((0xd0110+x),keybuf1,1);//
				yc2[x]=*keybuf1;
				yc2[x]=yc2[x]&0xff;
				}
				fw=((yc2[2]&0x00ff)<<6)|((yc2[3]&0x00fc)>>2);
				fw=fw*0.021972656;
				for(x=0;x<4;x++)
				{
				exbuf=ReadFromPhysicalMemory((0xd0100+x),keybuf1,1);//
				yc3[x]=*keybuf1;
				yc3[x]=yc3[x]&0xff;
				}
				jl=((yc3[2]&0x00ff)<<8)|(yc3[3]&0x00ff);
				jl=jl*5/1000;
			if(yc1[2]==0x81)
				{
				yc3[10]=(yc1[21]&0xff)|((yc1[20]&0x001f)<<8);
				ffhh=yc3[10]&0x1000;
				yc3[11]=(yc1[19]&0xff)|((yc1[18]&0x001f)<<8);
				ffhh=yc3[11]&0x1000;
				}
				
			if(yc1[2]==0x82)
				{
				yc3[10]=(yc1[21]&0xff)|((yc1[20]&0x001f)<<8);
				yc3[11]=(yc1[19]&0xff)|((yc1[18]&0x001f)<<8);
				ffhh=yc3[11]&0x1000;
				f_table=((yc1[16]&0x000f)<<8)|((yc1[17]&0xff));
				f_table=f_table/4;
				}
				
			if(yc1[2]==0x83)
				{
				yc3[10]=(yc1[21]&0xff)|((yc1[20]&0x001f)<<8);
				yc3[11]=(yc1[19]&0xff)|((yc1[18]&0x001f)<<8);
				ffhh=yc3[11]&0x1000;
				}
				
			if(yc1[2]==0x84)
				{
				yc3[10]=(yc1[21]&0xff)|((yc1[20]&0x001f)<<8);
				yc3[11]=(yc1[19]&0xff)|((yc1[18]&0x001f)<<8);
				ffhh=yc3[11]&0x1000;
				}
Fmt(planestring," (%f , %f)",glat,glon);  
//写像机返回信息

			   xpzs=(ptyc[9]&0xff)|((ptyc[10]&0x0f)<<8);
//写录象机返回信息
				mbok=1;
				Fmt(filestring,"%s R=%f, MB:%d ,buf:%d,xs=%d  ys=%d  xt=%d  yt=%d pcx=%d pcy=%d h=%f  r=%f  Q=%f  w=%f  f=%f  pg=%f  pf=%f\n",
	               Timestring1,dj_jl[djnum],mbnum,bufnum,dj_xs[djnum],dj_ys[djnum],xt,yt,pcx,pcy,dj_gd1,dj_qx[djnum],dj_fy[djnum],dj_hx[djnum],dj_f,dj_gdj[djnum],dj_fwj[djnum]); 

if(ir_or_ccd==0)			  //CCD比例系数
{
blxs_x=0.005714;
blxs_y=0.005806;
}
else if(ir_or_ccd=1)	//红外比例系数
	{
	blxs_x=0.015;
	blxs_y=0.015;
	}
if(DWORCT==1)   //WORCT=1：定位
{
xm=(testx[2]-(360+15))*blxs_x;//0.00667;
ym=(testy[2]-(288+15))*blxs_y;//0.00625;
xd1=(testx[3]-(360+15))*blxs_x;//0.00667;
yd1=(testy[3]-(288+15))*blxs_y;//0.00625;
}
else 
{
xm=(testx[0]-(360+15))*blxs_x;//0.00667;//0.0167;
ym=(testy[0]-(288+15))*blxs_y;//00625;//0.0167;
xd1=(testx[1]-(360+15))*blxs_x;//00667;//0.0167;
yd1=(testy[1]-(288+15))*blxs_y;//0.00625;//0.0167;

}
fjfw=fw/180.0*3.1415926;

//xs=3897894;
//ys=19416096;
CoordinateShift1(xs,ys);
            du=BZ;
			shi=(BZ-du)*60;
			fen=((BZ-du)*60-shi)*60;
            Fmt(xsstring,"%d. %d. %d",du,shi,fen);	   //目标
			du=LZ;
			shi=(LZ-du)*60;
			fen=((LZ-du)*60-shi)*60;
			Fmt(xsstring,"%d. %d. %d",du,shi,fen);
if(hx_gpshx_get==0)
			dj_hx[djnum]=dj_chx;
			else if(hx_gpshx_get==1)
				    dj_hx[djnum]=dj_gpshx;
if(jg_gd_get==1)
			dj_gd=dj_qygd;
			else if(jg_gd_get==0)
				    dj_gd=dj_jggd;
if(controlimage==1)   
	{
	hx11=((dj_hx[djnum]))*3.14/180.0;
	fy1=dj_fy[djnum]*3.14/180; 
	qx1=dj_qx[djnum]*3.14/180;
	ptfwj=(dj_fwj[djnum])*3.14/180;
	ptgdj=(dj_gdj[djnum])*3.14/180;
	ff=dj_f;
	gd1=dj_gd;
	xs1=dj_xs[djnum];
	ys1=dj_ys[djnum];
	}
	else
	{
	hx11=(hx)*3.14/180.0;			 
	fy1=fy*3.14/180; 
	qx1=qx*3.14/180;
	ptfwj=(fwj)*3.14/180.0;
	ptgdj=gdj*3.14/180.0;
	ff=f;
	gd1=jg_gd;
	xs1=xs;
	ys1=ys;
	}

xm1=-ym;
ym1=xm;
xd=-yd1;
yd=xd1;
xm4=-(525-288)*blxs_x;//0.00625;
ym4=(665-360)*blxs_y;//0.00667;

a1=cos(hx11)*cos(fy1)*cos(ptfwj)*cos(ptgdj)
   +sin(ptfwj)*cos(ptgdj)*(cos(hx11)*sin(fy1)*sin(qx1)-sin(hx11)*cos(qx1))
   -sin(ptgdj)*(cos(hx11)*cos(qx1)*sin(fy1)+sin(hx11)*sin(qx1));
a2=-cos(hx11)*cos(fy1)*sin(ptfwj)+cos(ptfwj)*(cos(hx11)*sin(fy1)*sin(qx1)-cos(qx1)*sin(hx11));
a3=cos(ptfwj)*sin(ptgdj)*cos(hx11)*cos(fy1)
   +sin(ptgdj)*sin(ptfwj)*(cos(hx11)*sin(fy1)*sin(qx1)-cos(qx1)*sin(hx11))
   +cos(ptgdj)*(cos(hx11)*cos(qx1)*sin(fy1)+sin(hx11)*sin(qx1));
b1=cos(ptfwj)*cos(ptgdj)*sin(hx11)*cos(fy1)
   +sin(ptfwj)*cos(ptgdj)*(cos(hx11)*cos(qx1)+sin(hx11)*sin(qx1)*sin(fy1))
   -sin(ptgdj)*(sin(hx11)*cos(qx1)*sin(fy1)-cos(hx11)*sin(qx1));
b2=-sin(ptfwj)*cos(fy1)*sin(hx11)
   +cos(ptfwj)*(cos(hx11)*cos(qx1)+sin(hx11)*sin(fy1)*sin(qx1));
b3=cos(ptfwj)*sin(ptgdj)*cos(fy1)*sin(hx11)
   +sin(ptfwj)*sin(ptgdj)*(cos(hx11)*cos(qx1)+sin(hx11)*sin(qx1)*sin(fy1))
   +cos(ptgdj)*(sin(hx11)*cos(qx1)*sin(fy1)-cos(hx11)*sin(qx1));
c1=-cos(ptfwj)*cos(ptgdj)*sin(fy1)+sin(ptfwj)*cos(ptgdj)*cos(fy1)*sin(qx1)
   -sin(ptgdj)*cos(fy1)*cos(qx1);
c2=sin(ptfwj)*sin(fy1)+cos(ptfwj)*cos(fy1)*sin(qx1);
c3=-cos(ptfwj)*sin(ptgdj)*sin(fy1)
   +sin(ptfwj)*sin(ptgdj)*cos(fy1)*sin(qx1)+cos(ptgdj)*cos(fy1)*cos(qx1);

d0=c1*xm1+c2*ym1+c3*ff;
d00=c1*xd+c2*yd+c3*ff; 
d000=c1*xm4+c2*ym4+c3*ff;
pintaihao=1;   
if(pintaihao==1)
{
	if(d0!=0)
	{
	xt=xs1+gd1*(a1*xm1+a2*ym1+a3*ff)/d0;	
	yt=ys1+gd1*(b1*xm1+b2*ym1+b3*ff)/d0;
	xz=xs1+gd1*(a1*xd+a2*yd+a3*ff)/d00;
	yz=ys1+gd1*(b1*xd+b2*yd+b3*ff)/d00;
	ys4=xs1+gd1*(a1*xm4+a2*ym4+a3*ff)/d000;
	xs4=ys1+gd1*(b1*xm4+b2*ym4+b3*ff)/d000;
	}
}

//计算弹道线坐标
GetCtrlVal (panel_handle, SERIAL_CHECKBOX, &zb_mode);
GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &plane_x0);
GetCtrlVal (panel_handle, SERIAL_NUMERIC_17, &plane_y0);
if(zb_mode!=1)
{
CoordinateShift(plane_x0,plane_y0);
plane_x0=xs_1;
plane_y0=ys_1;
}
t1=(plane_x0-xs1)*c1-gd1*a1;
t2=(plane_y0-ys1)*c1-gd1*b1;
s1=(plane_x0-xs1)*c2-gd1*a2;
s2=(plane_y0-ys1)*c2-gd1*b2;
p1=gd1*a3*ff-c3*ff*(plane_x0-xs1);
p2=gd1*b3*ff-c3*ff*(plane_y0-ys1);
ym2=(p1*t2-p2*t1)/(s1*t2-s2*t1);
xm2=(p1*s2-p2*s1)/(t1*s2-t2*s1);

ym2=ym2/blxs_x;//0.00667;
ym2=-ym2+288;
xm2=xm2/blxs_y;//0.00625;
xm2=-xm2+360;

xs4=xs4+10;
t1=(xs4-xs1)*c1-gd1*a1;
t2=(ys4-ys1)*c1-gd1*b1;
s1=(xs4-xs1)*c2-gd1*a2;
s2=(ys4-ys1)*c2-gd1*b2;
p1=gd1*a3*ff-c3*ff*(xs4-xs1);
p2=gd1*b3*ff-c3*ff*(ys4-ys1);
ym3=(p1*t2-p2*t1)/(s1*t2-s2*t1);
xm3=(p1*s2-p2*s1)/(t1*s2-t2*s1);

ym3=ym3/blxs_x;//0.00667;
ym3=-ym3+288;
xm3=xm3/blxs_y;//0.00625;
xm3=-xm3+360;


pcx=xz-xt;
pcy=yz-yt;
wucha_x=yzmbx-xt;
wucha_y=yzmby-yt;
SetCtrlVal (panel_handle, SERIAL_NUMERIC_2, dj_hx[djnum]);
SetCtrlVal (panel_handle, SERIAL_NUMERIC_10,dj_gd);
Fmt(mbdisdata,"(%d ,%d)",xt,yt);
Fmt(pcdisdata,"(%d ,%d)",pcx,pcy);


			if(sendbit==0)
			{
			if(mbok==1)
			Fmt(string,"(%d , %d)",xt,yt);
			//显示直角坐标
			if(controlimage==1)
			{
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_15, xs1);//glat);	  //飞机x
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_16, ys1);//glon);   //飞机y
			}
			else
			{
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_15, xs);//glat);	  //飞机x
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_16, ys);//glon);   //飞机y
			CoordinateShift(glat,glon);
			xs=xs_1;
			ys=ys_1;
			du=glat;
			shi=(glat-du)*60;
			fen=((glat-du)*60-shi)*60;
			Fmt(planestringx,"%d. %d. %d",du,shi,fen);//,miao);			   //飞机
			SetCtrlVal (panel_handle, SERIAL_TEXTMSG_18, planestringx); 
			du=glon;
			shi=(glon-du)*60;
			fen=((glon-du)*60-shi)*60;
			Fmt(planestringy,"%d. %d. %d",du,shi,fen);
			SetCtrlVal (panel_handle, SERIAL_TEXTMSG_19, planestringy); 
			}
			//目标直角坐标转换成经纬度坐标
            CoordinateShift1(xt,yt);    
            du=BZ;
			shi=(BZ-du)*60;
			fen=((BZ-du)*60-shi)*60;
            Fmt(targetstringx,"%d. %d. %d",du,shi,fen);//,miao);			   //目标
			du=LZ;
			shi=(LZ-du)*60;
			fen=((LZ-du)*60-shi)*60;
			Fmt(targetstringy,"%d. %d. %d",du,shi,fen);
			//炸点直角坐标转换成经纬度坐标  
			CoordinateShift1(xz,yz);    
            du=BZ;
			shi=(BZ-du)*60;
			fen=((BZ-du)*60-shi)*60;
            Fmt(zdstringx,"%d. %d. %d",du,shi,fen);//,miao);			   //目标
			du=LZ;
			shi=(LZ-du)*60;
			fen=((LZ-du)*60-shi)*60;
			Fmt(zdstringy,"%d. %d. %d",du,shi,fen);
            //偏差直角坐标转换成经纬度坐标  
            CoordinateShift1(xt,yt);    
            BZ1=BZ;
            LZ1=LZ;
            CoordinateShift1(xz,yz); 
            if(BZ1>BZ)
            {
            BZ=BZ1-BZ;
            LZ=LZ1-LZ;
            }
            else
            {
            BZ=BZ-BZ1;
            LZ=LZ-LZ1;
            }
            du=BZ;
			shi=(BZ-du)*60;
			fen=((BZ-du)*60-shi)*60;
            Fmt(pcstringx,"%d. %d. %d",du,shi,fen);//,miao);			   //偏差
			du=LZ;
			shi=(LZ-du)*60;
			fen=((LZ-du)*60-shi)*60;
			Fmt(pcstringy,"%d. %d. %d",du,shi,fen);
			if(controlimage==0)  
			{

			SetCtrlVal (panel_handle, SERIAL_NUMERIC, gd);//0.0);	  //高度
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_19, jg_gd);//0.0);	  //激光高度 
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_3, jl);//3.3);   //距离
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_4,hx);//4.4);   //航向
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_6,fy);//fy);//6.6);   //俯仰
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_5,qx);//qx);//5.5);  // 倾斜
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_7, f);	   //焦距
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_8, gdj);//BZ);//);	   //高低角
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_9, fwj);//LZ);//fwj);	   //方位角
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_23, gps_hx);//
			}
			else
			{
			SetCtrlVal (panel_handle, SERIAL_NUMERIC, dj_qygd);//0.0);	  //高度
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_19, dj_jggd);//0.0);	  //激光高度 
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_3, jl);//3.3);   //距离
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_4,dj_hx[djnum]);//4.4);   //航向
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_6,dj_fy[djnum]);//fy);//6.6);   //俯仰
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_5,dj_qx[djnum]);//qx);//5.5);  // 倾斜
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_7, dj_f);	   //焦距
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_8, dj_gdj[djnum]);//BZ);//);	   //高低角
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_9, dj_fwj[djnum]);//LZ);//fwj);	   //方位角
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_23, dj_gpshx);//
			}
		 }
	   
	 break;
	}
	return 0;
}

int CVICALLBACK StartSend (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			Initilation();  
			startOK=1;
			break;
	}
	return 0;
}

void ChangeData(void)
{
int cm;

	send_data[0]=0xeb;
	send_data[1]=0x90;  
	send_data[2]=0x64;
	send_data[3]=0x00;
	send_data[4]=0xea;
	send_data[5]=send_buf[5];   
	send_data[6]=send_buf[6];   
	send_data[7]=send_buf[7];   
	send_data[8]=send_buf[8];
	send_data[9]=send_buf[9]; 
	send_data[10]=send_buf[10];
	send_data[11]=send_buf[11]; 
	send_data[12]=send_buf[12]; 
}



int CVICALLBACK ScanQuit (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE, &x);
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE_2, &y);
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE_3, &z);
			if(zt==2)
			{
			send_buf[8]=0;
			send_buf[9]=0;
			send_buf[10]=0;
			}
			command_bit=2;
			ChangeData();
			DiscardPanel (scan_handle);
			break;
	}
	return 0;
}

int CVICALLBACK QuitCage (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			if(zt==2)
			{
			send_buf[8]=0;
			send_buf[9]=0;
			send_buf[10]=0;
			}
			command_bit=2;
			ChangeData();
			DiscardPanel (cage_handle);
			break;
	}
	return 0;
}

int CVICALLBACK QuitTrack (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{

	switch (event) {
		case EVENT_COMMIT:
			GetCtrlVal (adjust_handle, TRACK_NUMERICSLIDE, &adjust_heading); 
			DiscardPanel (adjust_handle);
			break;
	}
	return 0;
}

int CVICALLBACK Xshift (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			GetCtrlVal (adjust_handle, TRACK_NUMERICSLIDE, &adjust_heading);
			break;
	}
	return 0;
}



int CVICALLBACK BearAngle (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:

			kzzt=0; 
			GetCtrlVal (cage_handle, CAGE_NUMERICSLIDE, &spx);
			GetCtrlVal (cage_handle, CAGE_NUMERICSLIDE_2, &czy);
			if(pt_ctr_status==2)		  //位置配置
			{
				wzpzx_buf=spx;
				wzpzy_buf=czy;
				spx=spx;
				czy=czy-90;
				if(spx<0)
				spx=4096+spx;
				if(czy<0)
				czy=4096+czy;
				send_buf[4]=0xe9;
				send_buf[5]=0x00;
				send_buf[6]=0x11;  ;
				send_buf[7]=0x00;
				send_buf[8]=spx&0xff;
				send_buf[9]= (((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
				send_buf[10]=(czy&0xff0)>>4; 
				command_bit=0;
				ChangeData();
			}
			if(pt_ctr_status==3)		  //惯性/保持
			{
				spx=spx*50;
				if(spx<0)
				spx=4096+spx;
				czy=czy*50;
				if(czy<0)
				czy=4096+czy;
				send_buf[8]=spx&0xff;
				send_buf[9]= (((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
				send_buf[10]=(czy&0xff0)>>4; 
				command_bit=2;
				ChangeData();
			}
			if(pt_ctr_status==4)		  //搜索/跟踪
			{
				spx=spx/8;
				if(spx<0)
				spx=256+spx;
				czy=czy/8;
				if(czy<0)
				czy=256+czy;
				send_buf[8]=spx&0xff;
				send_buf[9]=0x00;
				send_buf[10]=czy&0xff; 
				command_bit=2;
				ChangeData();
			}
			
			break;
	}
	return 0;
}

int CVICALLBACK DepressAngle (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event) {
		case EVENT_COMMIT:
			kzzt=0; 
			GetCtrlVal (cage_handle, CAGE_NUMERICSLIDE, &spx);
			GetCtrlVal (cage_handle, CAGE_NUMERICSLIDE_2, &czy);
			if(pt_ctr_status==2)		 //位置配置
			{
				wzpzx_buf=spx;
				wzpzy_buf=czy;
				spx=spx;
				czy=czy-90;
				if(czy<0)
				czy=4096+czy;
				if(spx<0)
				spx=4096+spx;
				send_buf[4]=0xe9;
				send_buf[5]=0x00;
				send_buf[6]=0x11;  ;
				send_buf[7]=0x00;
				send_buf[8]=spx&0xff;
				send_buf[9]=(((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
				send_buf[10]=(czy&0xff0)>>4;
				command_bit=0;
				ChangeData();

			}
			if(pt_ctr_status==3)		  //惯性/保持
			{
				spx=spx*50;
				if(spx<0)
				spx=4096+spx;
				czy=czy*50;
				if(czy<0)
				czy=4096+czy;
				send_buf[8]=spx&0xff;
				send_buf[9]= (((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
				send_buf[10]=(czy&0xff0)>>4; 
				command_bit=2;
				ChangeData();
			}
			if(pt_ctr_status==4)		  //搜索/跟踪
			{
				spx=spx/8;
				if(spx<0)
				spx=256+spx;
				czy=czy/8;
				if(czy<0)
				czy=256+czy;
				send_buf[8]=spx&0xff;
				send_buf[9]=0x00;
				send_buf[10]=czy&0xff; 
				command_bit=2;
				ChangeData();
			}
			
			break;
	}
	return 0;
}

void StartRS(void)
{
                     imgInterfaceOpen ("img0", &Iid);
			         imgSessionOpen (Iid, &Sid);
	
			         // Check that the Acquisition window is not smaller than the Canvas
			         imgGetAttribute (Sid, IMG_ATTR_ROI_WIDTH, &AcqWinWidth);
			         imgGetAttribute (Sid, IMG_ATTR_ROI_HEIGHT, &AcqWinHeight);
			
			         if(AcqWinWidth>720)
				       AcqWinWidth = 720;
			         if(AcqWinHeight>576)
				       AcqWinHeight = 576;
				
			        // Set the ROI to the size of the Canvas so that it will fit nicely
			        imgSetAttribute (Sid, IMG_ATTR_ROI_WIDTH, AcqWinWidth);
			        imgSetAttribute (Sid, IMG_ATTR_ROI_HEIGHT, AcqWinHeight);
			        imgSetAttribute (Sid, IMG_ATTR_ROWPIXELS, AcqWinWidth);
			
			        // Set the plot Flag depending on the bitDepth
			        imgGetAttribute (Sid, IMG_ATTR_BITSPERPIXEL, &bitDepth); 

			        switch(bitDepth)
			         {
				        case 10:
					      PlotFlag = IMGPLOT_MONO_10;
					      break;
				        case 12:
					      PlotFlag = IMGPLOT_MONO_12;   
					      break;
				        case 14:
					      PlotFlag = IMGPLOT_MONO_14;   
				       	  break;
				        case 16:
					      PlotFlag = IMGPLOT_MONO_16;   
					      break;
				        case 24:
				        case 32:
					      PlotFlag = IMGPLOT_COLOR_RGB32;   
					      break;
				        default:
				 	      PlotFlag = IMGPLOT_MONO_8;   
					      break;
			        }
			
			      // Setup and launch the grap operation
			       imgGrabSetup (Sid, TRUE);

                  // Let NI-IMAQ manage the memory
			      ImaqBuffer = NULL;
}


int CVICALLBACK StopImage (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int stopi;
	switch (event)
		{
		case EVENT_COMMIT:
		 GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
		 if(datasource==0)
		  {
			GetCtrlVal (panel_handle, SERIAL_TOGGLEBUTTON, &stopi); 
			switch(stopi){
						 case 1://冻结图像
						selstatus0=0;
						controlimage=1;
						testx[0]=0;
	                    testx[1]=0;
	                    testy[0]=0;
	                    testy[1]=0;
						SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 0);
                        SetCtrlAttribute (panel_handle, SERIAL_TOGGLEBUTTON_2, ATTR_DIMMED, 1);
						 dj_jl[djnum]=jl;
						 dj_xs[djnum]=xs;
						 dj_ys[djnum]=ys;
						 dj_hx[djnum]=hx;
						 dj_fy[djnum]=fy;
						 dj_qx[djnum]=qx;
						 dj_fwj[djnum]=fwj;
						 dj_gdj[djnum]=gdj;
						 dj_f=f;
						 dj_gpshx=gps_hx;
						dj_qygd=gd;
						dj_jggd=jg_gd;
						dj_gd=jg_gd;//gd;//jg_gd;
						dj_chx=hx;
						if(active_ccd_ir==0x00)   //红外焦距
						{
						}
						else if(active_ccd_ir==0x04)  //电视焦距
							{
							}
						 Fmt(Datastring1,"%d/%d/%d",year,month,day);
						 Fmt(Timestring1,"%d:%d:%d",hours,minutes,seconds);
						 mb_zd_shift=0;  //=1：画目标 =2：画炸点
						 command_bit=0; 
						 
						 if(*mapbuf==1)
    	                   {
    			
    	                      imaqReadFile (image, "d:\\temp\\index1.bmp", NULL, NULL); 
    			
    		                  imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
    			
    		               }
                         else if(*mapbuf==2)
    		               {
    			
    		                  imaqReadFile (image, "d:\\temp\\index2.bmp", NULL, NULL); 
    			
    		                  imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
    		               }  
						 
						 break;
						 case 0:	//重放图象
						 selstatus0=1;
						 controlimage=0;
						 flag=1;
						 SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 1);
						 SetCtrlAttribute (panel_handle, SERIAL_TOGGLEBUTTON_2, ATTR_DIMMED, 0);
						 break;
						 }
		  }
	     else if(datasource==2)
	        {
	           GetCtrlVal (panel_handle, SERIAL_TOGGLEBUTTON, &stopi);
	           switch(stopi)
	             {
	                case 1://冻结图像
	                  selstatus0=0;
	                  controlimage=1;
	                  testx[0]=0;
	                  testx[1]=0;
	                  testy[0]=0;
	                  testy[1]=0;
	                  SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 0);
	                  SetCtrlAttribute (panel_handle, SERIAL_TOGGLEBUTTON_2, ATTR_DIMMED, 1);
	                  if(tableRS==1)
	                    {
	                       imgSessionStopAcquisition (Sid);
			               imgClose (Sid, TRUE);
			               imgClose (Iid, TRUE);
			               tableRS=0;
			            }
			             dj_jl[djnum]=jl;
						 dj_xs[djnum]=xs;
						 dj_ys[djnum]=ys;
						 dj_hx[djnum]=hx;
						 dj_fy[djnum]=fy;
						 dj_qx[djnum]=qx;
						 dj_fwj[djnum]=fwj;
						 dj_gdj[djnum]=gdj;
						 dj_f=f;
						 dj_gpshx=gps_hx; 
						 dj_qygd=gd;
						 dj_jggd=jg_gd;
						 dj_gd=jg_gd;//gd;
						 dj_chx=hx;//jg_gd;
				
			         break;
	                case 0://重放图像
	                 selstatus0=1;
	                 controlimage=0;
	                 flag=1;
	                 SetCtrlAttribute (panel_handle, SERIAL_COMMANDBUTTON_5, ATTR_DIMMED, 1);
	                 SetCtrlAttribute (panel_handle, SERIAL_TOGGLEBUTTON_2, ATTR_DIMMED, 0);
	                 if(tableRS==0)
	                   {
	                      StartRS();
	                      tableRS=1;
	                   }
	                 
	              break;   
	             }
	        }
	     break;
		}
	return 0;
}


int CVICALLBACK ExitScope (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON,&lx_lx);
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_2,&lx_pf);
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_3,&lx_kj);
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_4,&lx_kt);
			lxj_backinf_display=0;
			DiscardPanel (scope_handle);   
			break;
		}
	return 0;
}

int CVICALLBACK Lx (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON,&lx_lx);
			if(lx_lx==0)
			{
			send_buf[14]=0xd9;
            send_buf[15]=0xd9;
            send_buf[16]=0xef;
            send_buf[17]=0xef;
			}else if(lx_lx==1)
				{
				send_buf[14]=0xd9;
                send_buf[15]=0xd9;
                send_buf[16]=0xee;
                send_buf[17]=0xee;
				}
			command_bit1=8;     
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_4,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_2,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_3,0);
			
			break;
		}
	return 0;
}

int CVICALLBACK Bf (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_2,&lx_pf);
			if(lx_pf==0)
			{
			send_buf[14]=0xd9;
            send_buf[15]=0xd9;
            send_buf[16]=0xef;
            send_buf[17]=0xef;
			}else if(lx_pf==1)
				{
				send_buf[14]=0xd9;
                send_buf[15]=0xd9;
                send_buf[16]=0xed;
                send_buf[17]=0xed;
				}
			command_bit1=8;     
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_4,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_3,0);
			
			break;
		}
	return 0;
}

int CVICALLBACK Kj (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_3,&lx_kj);  
			if(lx_kj==0)
			{
			send_buf[14]=0xd9;
            send_buf[15]=0xd9;
            send_buf[16]=0xef;
            send_buf[17]=0xef;
			}else if(lx_kj==1)
				{
//				printf("%d",lx_lx);
				send_buf[14]=0xd9;
                send_buf[15]=0xd9;
                send_buf[16]=0xfa;
                send_buf[17]=0xfa;
				}
			command_bit1=8;     
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_2,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_4,0);
			
			break;
		}
	return 0;
}

int CVICALLBACK Kt (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_4,&lx_kt);   
			if(lx_kt==0)
			{
			send_buf[14]=0xd9;
            send_buf[15]=0xd9;
            send_buf[16]=0xef;
            send_buf[17]=0xef;
			}else if(lx_kt==1)
				{
//				printf("%d",lx_lx);
				send_buf[14]=0xd9;
                send_buf[15]=0xd9;
                send_buf[16]=0xfb;
                send_buf[17]=0xfb;
				}
			command_bit1=8;     
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_2,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_3,0);
			
			break;
		}
	return 0;
}

int CVICALLBACK PTYC (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			ptycok=1;
			ptds_bj=1;
			break;
		}
	return 0;
}


int CVICALLBACK TrackCorner (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			kzzt=0; 
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE, &spx);
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE_3, &czy);
			if(pt_ctr_status==1)		   //扫描配置
			{
				if((spx!=0)&&(czy!=0))
				{
				send_buf[8]=spx&0xff;
				send_buf[9]= (((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
				send_buf[10]=(czy&0xff0)>>4; 
				command_bit=2;
				ChangeData();
				}
			}
			break;
		}
	return 0;
}

int CVICALLBACK TrackSpeed (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			kzzt=0; 
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE, &spx);
			GetCtrlVal (scan_handle, SCAN_NUMERICSLIDE_3, &czy);
			if(pt_ctr_status==1)		   //扫描配置
			{
				if((spx!=0)&&(czy!=0))
				{
				send_buf[8]=spx&0xff;
				send_buf[9]=(((spx&0xf00)>>8)|((czy&0x0f)<<4))&0x00ff;
				send_buf[10]=(czy&0xff0)>>4;
				command_bit=2;
				ChangeData();
				}
			}
			break;
		}
	return 0;
}

int CVICALLBACK GD_ql (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			cage_handle = LoadPanel (panel_handle, "serial.uir",HRESET);        
            InstallPopup (cage_handle);
			
			break;
		}
	return 0;
}


void CoordinateShift(double jdj,double wdj)		//经纬度坐标转换为高斯平面
{
int n1;
double t,h2,B,L,e12,V2,V0,C,V,N,M,R,P2,C0,C1,C2,C3,n,l,m,x0b,y11;
e12=0.006738525414683497; 
C=6399698.9017827111;
C0=6367558.49687;
C1=32005.7801;
C2=133.9213;
C3=0.7032;
B=jdj*3.1415926/180;
L=wdj*3.1415926/180;
t=tan(B);
h2=e12*cos(B)*cos(B);
V2=1+h2;
V=sqrt(V2);
V0=sqrt(1.0+e12);
N=C/V;
M=C/(V*V*V);
R=C/(V*V);
P2=206264.8062471;
n1=L*P2/(6*3600); 
n=1+n1;
l=L-(6*n-3)*3600/P2;
m=l*cos(B);
x0b=C0*B-cos(B)*(C1*sin(B)+C2*sin(B)*sin(B)*sin(B)+C3*sin(B)*sin(B)*sin(B)*sin(B)*sin(B));
xs_1=x0b+0.5*N*t*m*m+1.0/24.0*(5-t*t+9*h2+4*h2*h2)*N*t*m*m*m*m+1.0/720.0*(61-58*t*t+t*t*t*t)*N*t*m*m*m*m*m*m;
y11=N*m+1.0/6.0*(1-t*t+h2)*N*m*m*m+1.0/120.0*(5-18*t*t+t*t*t*t+14*h2-58*h2*t*t)*N*m*m*m*m*m;
ys_1=pow(10,6)*n+500000+y11;
}


void CoordinateShift1(long xsexx,long ysexx)		//为高斯平坐标转换为经纬度
{
int n1;
double t,h2,Bf,e12,V2,V0,C,V,N,M,R,P2,C0,C1,C2,C3,n,l,m,x0b,y11;
double k0,k1,k2,k3,k4;
double y1,E,xsex,ysex;

xsex=xsexx;//3830045.654;
ysex=ysexx;//19418279.470;

k0=0.157046046122*0.000001;
k1=0.005051773902;
k2=0.00002983868;
k3=0.0000002415;
k4=0.0000000022;
e12=0.006738525414683497; 
C=6399698.9017827111;
C0=6367558.49687;
C1=32005.7801;
C2=133.9213;
C3=0.7032;
n1=ysex*0.000001;
y1=ysex-n1*1000000.0-500000;
E=k0*xsex;
Bf=E+cos(E)*(k1*sin(E)-k2*pow(sin(E),3.0)+k3*pow(sin(E),5.0)-k4*(sin(E),7.0));
t=tan(Bf);
h2=e12*cos(Bf)*cos(Bf);
V2=1+h2;
V=sqrt(V2);
N=C/V;
P2=206264.8062471;
BZ=Bf-1.0/2.0*(V2*t)*pow((y1/N),2.0)+1.0/24.0*(5+3*t*t+h2-9*h2*t*t)*(V2*t)*pow((y1/N),4.0)-1.0/720.0*(61+90*t*t+45*pow(t,4.0))*(V2*t)*pow((y1/N),6.0);
l=(1.0/cos(Bf))*(y1/N)-1.0/6.0*(1+2*t*t+h2)*(1.0/cos(Bf))*pow((y1/N),3.0)+1.0/120.0*(5+28*t*t+24*pow(t,4.0)+6*h2+8*h2*t*t)*(1.0/cos(Bf))*pow((y1/N),4.0);
LZ=3600.0/P2*(6*n1-3)+l;

BZ=BZ*180.0/3.14159265358975;
LZ=LZ*180.0/3.14159265358975;

		
}

int CVICALLBACK HresetQuit (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (cage_handle, HRESET_NUMERIC, &gd3);
			DiscardPanel (cage_handle); 
			break;
		}
	return 0;
}

int CVICALLBACK GetHvalue (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			gd2=gd;
			if(gd2<0)
			gd3=-gd2;
			else gd3=gd2;
			SetCtrlVal (cage_handle, HRESET_NUMERIC, gd2);    
			break;
		}
	return 0;
}

int CVICALLBACK Yzmb (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			yzmb_handle = LoadPanel (panel_handle, "serial.uir",YZMB);        
            InstallPopup (yzmb_handle);
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC, yzmbx1);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_2, yzmby1); 
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_3, yzmbx2);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_4, yzmby2); 
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_5, yzmbx3);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_6, yzmby3); 
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_7, yzmbx4);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_8, yzmby4); 
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_9, yzmbx5);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_10, yzmby5); 
			
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_11, yzmbx6);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_12, yzmby6); 
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_13, yzmbx7);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_14, yzmby7); 
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_15, yzmbx8);  
			SetCtrlVal (yzmb_handle, YZMB_NUMERIC_16, yzmby8);
			break;
		}
	return 0;
}

void initYzmb(void)
{
char linebuffer[40]={0}; 
char tempbuffer[20]={0};
int fp1;
int totalnum=1;
int zbx,zby;
int xb;
char ch=' ';
int i=0;
int yzmbselect;   
		fp1 = OpenFile ("target.txt", VAL_READ_ONLY, VAL_OPEN_AS_IS, VAL_BINARY);
		if(fp1!=-1)
		{
			while(totalnum!=-2)
			{
				i++;
				memset(linebuffer,0,40);
				memset(tempbuffer,0,20);
				totalnum=ReadLine(fp1,linebuffer,40);
				xb=strcspn(linebuffer,&ch);
				memcpy(tempbuffer,linebuffer,xb);
				zbx=atoi(tempbuffer);
				memset(tempbuffer,0,20);
				memcpy(tempbuffer,linebuffer+xb,strlen(linebuffer)-xb);
				while(*tempbuffer==' ')
				{
					memmove(tempbuffer,tempbuffer+1,strlen(tempbuffer)-1);
					tempbuffer[strlen(tempbuffer)-1]=0;
				}
				zby=atoi(tempbuffer);
				switch(i)
				{
					case 1:
					yzmbx1=zbx;
					yzmby1=zby;
					break;
					case 2:
					yzmbx2=zbx;
					yzmby2=zby;
					break;
					case 3:
					yzmbx3=zbx;
					yzmby3=zby;
					break;
					case 4:
					yzmbx4=zbx;
					yzmby4=zby;
					break;
					case 5:
					yzmbx5=zbx;
					yzmby5=zby;
					break;
					case 6:
					yzmbx6=zbx;
					yzmby6=zby;
					break;
					case 7:
					yzmbx7=zbx;
					yzmby7=zby;
					break;
					case 8:
					yzmbx8=zbx;
					yzmby8=zby;
					break;
					default:
					break;
				}
				
			}
		    CloseFile(fp1);
		}
	    GetCtrlVal (panel_handle, SERIAL_NUMERIC_22, &yzmbselect);
		switch(yzmbselect)
		{
			case 1:
			yzmbx=yzmbx1;
			yzmby=yzmby1;
			break;
			case 2:
			yzmbx=yzmbx2;
			yzmby=yzmby2;
			break;
			case 3:
			yzmbx=yzmbx3;
			yzmby=yzmby3;
			break;
			case 4:
			yzmbx=yzmbx4;
			yzmby=yzmby4;
			break;
			case 5:
			yzmbx=yzmbx5;
			yzmby=yzmby5;
			break;
			case 6:
			yzmbx=yzmbx6;
			yzmby=yzmby6;
			break;
			case 7:
			yzmbx=yzmbx7;
			yzmby=yzmby7;
			break;
			case 8:
			yzmbx=yzmbx8;
			yzmby=yzmby8;
			break;

			default:
			break;
		}

}

int CVICALLBACK YzmbQuit (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
char linebuffer[40]={0}; 
char xxxyyy;
int yzmbselect; 
int fp1;
char *ch=" ";
char *hh="\n";


	switch (event)
		{
		case EVENT_COMMIT:
		
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC, &yzmbx1);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_2, &yzmby1); 
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_3, &yzmbx2);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_4, &yzmby2); 
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_5, &yzmbx3);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_6, &yzmby3); 
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_7, &yzmbx4);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_8, &yzmby4); 
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_9, &yzmbx5);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_10, &yzmby5); 
			
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_11, &yzmbx6);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_12, &yzmby6);
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_13, &yzmbx7);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_14, &yzmby7);
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_15, &yzmbx8);  
			GetCtrlVal (yzmb_handle, YZMB_NUMERIC_16, &yzmby8);	
		
		    GetCtrlVal (panel_handle, SERIAL_NUMERIC_22, &yzmbselect);
			switch(yzmbselect)
			{
				case 1:
				yzmbx=yzmbx1;
				yzmby=yzmby1;
				break;
				case 2:
				yzmbx=yzmbx2;
				yzmby=yzmby2;
				break;
				case 3:
				yzmbx=yzmbx3;
				yzmby=yzmby3;
				break;
				case 4:
				yzmbx=yzmbx4;
				yzmby=yzmby4;
				break;
				case 5:
				yzmbx=yzmbx5;
				yzmby=yzmby5;
				break;
				case 6:
				yzmbx=yzmbx6;
				yzmby=yzmby6;
				break;
				case 7:
				yzmbx=yzmbx7;
				yzmby=yzmby7;
				break;
				case 8:
				yzmbx=yzmbx8;
				yzmby=yzmby8;
				break;

				default:
				break;
			}
			fp1 = OpenFile ("target.txt", VAL_READ_WRITE, VAL_TRUNCATE, VAL_ASCII);
			if(fp1!=-1)
			{
				Fmt(linebuffer,"%d  %d\n",yzmbx1,yzmby1);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx2,yzmby2);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx3,yzmby3);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx4,yzmby4);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx5,yzmby5);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx6,yzmby6);
				WriteFile (fp1, linebuffer,strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx7,yzmby7);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				memset(linebuffer,0,40);  
				Fmt(linebuffer,"%d  %d\n",yzmbx8,yzmby8);
				WriteFile (fp1, linebuffer, strlen(linebuffer));
				CloseFile (fp1);
			}
			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000);
		    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
								   MakePoint(250,10), VAL_CENTER_RIGHT);
	        scalenum=sqrt(((xs-yzmbx)*(xs-yzmbx)+(ys-yzmby)*(ys-yzmby))/(800*800));
			scalenum+=1;
    		CreateMetaFont("VAL_4SEG_META_FONT",VAL_APP_META_FONT,11,0,0,0,0);
    	    SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_FILL_COLOR, VAL_BLACK); 
    	    drawcanvas(scalenum);
			DiscardPanel (yzmb_handle);    
			break;
		}
	return 0;
}


int CVICALLBACK QuitCameraPanel (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON, &xj_open20_status);  
			GetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON_2, &xj_open60_status);  
			DiscardPanel (cam_handle);  
			break;
		}
	return 0;
}

int CVICALLBACK XJ_TIMER_SET (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int timeset;
	switch (event)
		{
		case EVENT_COMMIT:
			timeset = GetSystemTime(&hours, &minutes, &seconds);
			timeset = GetSystemDate (&month, &day, &year);
			send_buf[14]=0xd0;
            send_buf[15]=0xd0;
            send_buf[16]=(year-2000)&0xff;
            send_buf[17]=(year-2000)&0xff;
			time_set_status=1;
			command_bit1=8;    
			break;
		}
	return 0;
}

int CVICALLBACK XJ_OPEN20 (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int xj_status;
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON, &xj_status);
		    if(xj_status==1)
		    {
			send_buf[14]=0xd6;
            send_buf[15]=0xd6;
            send_buf[16]=0x93;
            send_buf[17]=0x93;
			SetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON_2, 0);    
			command_bit1=8;    
			}
			else if(xj_status==0)
				{
				send_buf[14]=0xd6;
            	send_buf[15]=0xd6;
            	send_buf[16]=0x95;
            	send_buf[17]=0x95;
				SetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON, 0);
				SetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON_2, 0); 
				command_bit1=8;    
				}
			
			break;
		}
	return 0;
}

int CVICALLBACK XJ_OPEN60 (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int xj_status1;
	switch (event)
		{
		case EVENT_COMMIT:
			
			GetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON_2, &xj_status1); 
			if(xj_status1==1)
			{
			send_buf[14]=0xd6;
            send_buf[15]=0xd6;
            send_buf[16]=0x6c;
            send_buf[17]=0x6c;
			SetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON, 0);   
			command_bit1=8;    
			}
			else if(xj_status1==0)
			{
			send_buf[14]=0xd6;
            send_buf[15]=0xd6;
            send_buf[16]=0x95;
            send_buf[17]=0x95;
			SetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON, 0);
			SetCtrlVal (cam_handle, CAMERA_TOGGLEBUTTON_2, 0); 
			command_bit1=8;    
			}
			break;
		}
	return 0;
}

int CVICALLBACK XJ_CLOSE (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int xj_high_jz; 
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (cam_handle, CAMERA_NUMERIC, &xj_high_jz); 
			xj_high_jz=xj_high_jz/100;
			send_buf[14]=0xd7;
            send_buf[15]=0xd7;
            send_buf[16]=xj_high_jz&0xff;
            send_buf[17]=xj_high_jz&0xff;
			command_bit1=8;    
			break;
		}
	return 0;
}

int CVICALLBACK XJ_HIGH_SET (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{

	switch (event)
		{
		case EVENT_COMMIT:
			 
			break;
		}
	return 0;
}

int CVICALLBACK JLY_HD_CLEAR (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			send_buf[14]=0xd9;
            send_buf[15]=0xd9;
            send_buf[16]=0xfc;
            send_buf[17]=0xfc;
			command_bit1=8;    
			break;
		}
	return 0;
}

int CVICALLBACK JLY_CLOSE (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			send_buf[14]=0xd9;
            send_buf[15]=0xd9;
            send_buf[16]=0xfd;
            send_buf[17]=0xfd;
			command_bit1=8;    
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_4,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_2,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON_3,0);
			SetCtrlVal (scope_handle, SCOPE_TOGGLEBUTTON,0);
			break;
		}
	return 0;
}

int CVICALLBACK JLY_FRAME_NUMBER (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int frame_number;
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_NUMERIC, &frame_number); 
			send_buf[14]=0xdb;
            send_buf[15]=0xdb;
            send_buf[16]=frame_number&0xff;
            send_buf[17]=frame_number&0xff;
			command_bit1=8;
			break;
		}
	return 0;
}

int CVICALLBACK JLY_COMPRESS_FACT (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int compress_fact;
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (scope_handle, SCOPE_NUMERIC_2, &compress_fact); 
			send_buf[14]=0xda;
            send_buf[15]=0xda;
            send_buf[16]=compress_fact&0xff;
            send_buf[17]=compress_fact&0xff;
			command_bit1=8;    
			break;
		}
	return 0;
}



int CVICALLBACK SELECT_HIGH (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_5, &gd_get_status);  
			if(gd_get_status==0)
			jg_gd_get=0;
			else jg_gd_get=1;
			if(jg_gd_get==1)
			dj_gd=dj_qygd;
			else if(jg_gd_get==0)
				    dj_gd=dj_jggd;
			break;
		}
	return 0;
}

void Initilation(void)
{
int filen,systime;
	// 初始化文件
	systime = GetSystemDate (&month, &day, &year);
	systime = GetSystemTime(&hours, &minutes, &seconds);
	Fmt(filename,"d:\\%d%d%d%d%d.txt",year,month,day,minutes,seconds);
	filehandle = OpenFile (filename, VAL_READ_WRITE, VAL_TRUNCATE, VAL_ASCII); 
	Fmt(filestring,"%s  %s  %s\n",Datastring,Timestring,"目标定位数据");
//	filen= WriteFile (filehandle, filestring, stringlength);
}


int CVICALLBACK SelYZMB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
int yzmbselect;
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_22, &yzmbselect);
				switch(yzmbselect)
				{
				case 1:
				yzmbx=yzmbx1;
				yzmby=yzmby1;
				break;
				case 2:
				yzmbx=yzmbx2;
				yzmby=yzmby2;
				break;
				case 3:
				yzmbx=yzmbx3;
				yzmby=yzmby3;
				break;
				case 4:
				yzmbx=yzmbx4;
				yzmby=yzmby4;
				break;
				case 5:
				yzmbx=yzmbx5;
				yzmby=yzmby5;
				break;
				case 6:
				yzmbx=yzmbx6;
				yzmby=yzmby6;
				break;
				case 7:
				yzmbx=yzmbx7;
				yzmby=yzmby7;
				break;
				case 8:
				yzmbx=yzmbx8;
				yzmby=yzmby8;
				break;
				default:
				break;
				}

			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000);
			CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, mbstring, "VAL_4SEG_META_FONT",
								   MakePoint(3,10), VAL_CENTER_LEFT);
		    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
								   MakePoint(250,10), VAL_CENTER_RIGHT);
	        scalenum=sqrt(((xs-yzmbx)*(xs-yzmbx)+(ys-yzmby)*(ys-yzmby))/(800*800));
			scalenum+=1;
    		CreateMetaFont("VAL_4SEG_META_FONT",VAL_APP_META_FONT,11,0,0,0,0);
    	    SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_FILL_COLOR, VAL_BLACK);
    	    drawcanvas(scalenum);
    	    Fmt(mbstring,"Target%d:",yzmbselect); 
    	    SetCtrlAttribute (panel_handle, SERIAL_CANVAS_2, ATTR_PEN_COLOR, VAL_YELLOW); 
    	    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, mbstring, "VAL_4SEG_META_FONT",
								   MakePoint(3,10), VAL_CENTER_LEFT);
			break;
		}
	return 0;
}




int CVICALLBACK Stop_Grape (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:  
			break;
		}
	return 0;
}


int CVICALLBACK Start_Grape (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			break;
		}
	return 0;
}



int CVICALLBACK Record_Image (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
		
			break;
		}
	return 0;
}

int CVICALLBACK Plane_X0 (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			if(zb_mode==1)
				GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &zjx);
				else if(zb_mode==2) GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &jwdx);
			
			break;
		}
	return 0;
}

int CVICALLBACK Plane_Y0 (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
				if(zb_mode==1)
				GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &zjy);
				else if(zb_mode==2) GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &jwdy);
			break;
		}
	return 0;
}

int CVICALLBACK Focus_select (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{

	switch (event)
		{
		case EVENT_COMMIT:
			break;
		}
	return 0;
}



/*放大范围，每次放大0.8公里*/
int CVICALLBACK magnify (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
		   //范围信息消影
		    SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000);
		    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
								   MakePoint(250,10), VAL_CENTER_RIGHT);
			scalenum+=1;
			drawcanvas(scalenum);
			break;
		}
	return 0;
}


void drawcanvas(int scalenum1)
{
	juliz=2*(scalenum1)*100*4/1000;
	julil=2*(scalenum1)*100*4%1000;
	Fmt(scalestring,"%d.%d KM ",juliz,julil);
	SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,VAL_GREEN);
	//写范围信息
	CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
						   MakePoint(250,10), VAL_CENTER_RIGHT);
}


/*缩小范围，每次缩小0.8公里*/
int CVICALLBACK dwindle (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
		   //范围信息消影
		    SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000);
		    CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
								   MakePoint(250,10), VAL_CENTER_RIGHT);
			scalenum-=1;
			if(scalenum<1)
			    scalenum=1;
			drawcanvas(scalenum);
			break;
		}
	return 0;
}


/*重置范围至0.8公里*/
int CVICALLBACK reset (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
		   //范围信息消影
			SetCtrlAttribute(panel_handle,SERIAL_CANVAS_2,ATTR_PEN_COLOR,0000000);
			CanvasDrawTextAtPoint (panel_handle, SERIAL_CANVAS_2, scalestring, "VAL_4SEG_META_FONT",
								   MakePoint(250,10), VAL_CENTER_RIGHT);
			scalenum=1;	
			drawcanvas(scalenum);
			break;
		}
	return 0;
}



/*通过vision事件触发在图像上进行标记*/

void IMAQ_CALLBACK ExtractImage (WindowEventType event, int windowNumber, Tool tool, Rect rect)
{
//	Point position1;
	int width,heighth;
	switch (event)
		{
		   case IMAQ_CLICK_EVENT:
		    //  printf("**1**");  
		      if(controlimage==1)		//controlimage=1:冻结 =0：重放
		        {
		           flag=0;
		           
		           GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_6, &DWORCT);
		           GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH, &mborzd);
			       imaqGetMousePos (&position1, WINDOW_TO_USE);
			       
		       if(DWORCT==1&&mborzd==1)		  //DWORCT=1：定位 =0：测图  mborzd=1:目标 =0：炸点 
		            { 
		          		imaqOverlayLine(image,MakePoint(position1.x-5,position1.y),MakePoint(position1.x+5,position1.y),&IMAQ_RGB_APP0,NULL);
				        imaqOverlayLine(image,MakePoint(position1.x,position1.y-5),MakePoint(position1.x,position1.y+5),&IMAQ_RGB_APP0,NULL);
					  imaqDisplayImage (image, WINDOW_TO_USE, TRUE);
					  testx[2]=position1.x;
                      testy[2]=position1.y;
				    }
			       else if(DWORCT==1&&mborzd==0)
			        {
					  imaqDrawLineOnImage (image, image,IMAQ_DRAW_VALUE, MakePoint(position1.x-5,position1.y),
				                           MakePoint(position1.x+5,position1.y),MakeColor(163,139,243));
				      imaqDrawLineOnImage (image, image,IMAQ_DRAW_VALUE, MakePoint(position1.x,position1.y-5),
				                           MakePoint(position1.x,position1.y+5),MakeColor(163,139,243));				   
					  imaqDisplayImage (image, WINDOW_TO_USE, TRUE);
					  testx[3]=position1.x;
                      testy[3]=position1.y;
			        }
			       else if(DWORCT==0)
			        {
			           if(valuej==0)
			             {
			                imaqClearOverlay(image,NULL);
			                if(selstatus==0)
			                  Drawimage();
			                valuei=1;
			                testx[0]=position1.x;
			                testy[0]=position1.y;
			                valuej++;
			             }
			           else 
			             {
			               valuej=0;
			               valuei=0;
			             }
				       imaqDisplayImage (image, WINDOW_TO_USE, TRUE);
			        }   
		
		        }
		     break;
		
		 case IMAQ_DOUBLE_CLICK_EVENT:
			break;
		}
	return;
}

int CVICALLBACK ZB_SHIFT (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:

			GetCtrlVal (panel_handle, SERIAL_CHECKBOX, &zb_mode);
			if(zb_mode==1)
			{
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX, 1);
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX_2, 0); 
			zb_mode=1;
			}
			else {
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX, 0);
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX_2, 1); 
			zb_mode=2;
			}
			if(zb_mode==1)
			{
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &plane_x0);
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_17, &plane_y0);
			CoordinateShift(plane_x0,plane_y0);//glat0,glon0);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_18, xs_1);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_17, ys_1);
			}
			else if(zb_mode==2)
			{
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &plane_x0);
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_17, &plane_y0);
			CoordinateShift1(plane_x0,plane_y0);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_18, BZ);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_17, LZ);
			}
			break;
			
		}
	return 0;
}

int CVICALLBACK ZB_SHIFT1 (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_CHECKBOX_2, &zb_mode);
			if(zb_mode==1)
			{
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX, 0);
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX_2, 1); 
			zb_mode=2;
			}
			else {
			 SetCtrlVal (panel_handle, SERIAL_CHECKBOX, 1);
			SetCtrlVal (panel_handle, SERIAL_CHECKBOX_2, 0); 
			zb_mode=1;
			}
			if(zb_mode==1)
			{
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &plane_x0);
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_17, &plane_y0);
			CoordinateShift(plane_x0,plane_y0);//glat0,glon0);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_18, xs);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_17, ys);
			}
			else if(zb_mode==2)
			{
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_18, &plane_x0);
			GetCtrlVal (panel_handle, SERIAL_NUMERIC_17, &plane_y0);
			CoordinateShift1(plane_x0,plane_y0);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_18, BZ);
			SetCtrlVal (panel_handle, SERIAL_NUMERIC_17, LZ);
			}
			break;
			
			
		}
	return 0;
}

int CVICALLBACK Sel_MBORZD (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
  
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH, &mborzd);
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_6, &DWORCT);
			if(DWORCT==1)
			  {
			    if(mborzd==1)
			      imaqSetToolColor(&IMAQ_RGB_APP0);
			    if(mborzd==0)
			      imaqSetToolColor(&IMAQ_RGB_APP1);  
			  }
			break;
		}
	return 0;
}

int CVICALLBACK Sel_DWORCT (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_6, &DWORCT);
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH, &mborzd);
			   if(DWORCT==1)
			     {
			        imaqSetCurrentTool(IMAQ_POINT_TOOL);
			        if(mborzd==1)
			          imaqSetToolColor(&IMAQ_RGB_APP0);
			        if(mborzd==0)
			          imaqSetToolColor(&IMAQ_RGB_APP1);
			        valuei=0;
			     }
			   else if(DWORCT==0)
			     {
			        imaqSetCurrentTool(IMAQ_POINT_TOOL);
					imaqSetToolColor(&IMAQ_RGB_TRANSPARENT);
			     }     
			break;
		}
	return 0;
}

int CVICALLBACK sourcedata (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
		    if(datasource==0)
		      {
		        SetCtrlVal (panel_handle, SERIAL_LED_2, 0);
		        SetCtrlVal (panel_handle, SERIAL_LED_3, 1);
		        newcount=0;
		        if(tableRS==1)
                    {
                       imgSessionStopAcquisition (Sid);
		               imgClose (Sid, TRUE);
		               imgClose (Iid, TRUE);
		               tableRS=0;
		            }
		        
		        exiting=0;
		        CmtScheduleThreadPoolFunction (poolHandle,ImagePlay,NULL, NULL); 
	            WSAAsyncSelect (udpHandle, hwnd, wmsg, FD_READ);
		      }
		    else
		      {
		        SetCtrlVal (panel_handle, SERIAL_LED_2, 1);
		        SetCtrlVal (panel_handle, SERIAL_LED_3, 0);
		        newcount=0;
		        if(tableRS==0)
	                   {
	                      exiting=1; 
	                      StartRS();
	                      tableRS=1;
	                   }
		      }
			break;
		}
	return 0;
}

int CVICALLBACK ClearExtractimage (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int stopi;
	switch (event)
		{
		case EVENT_COMMIT:
		    GetCtrlVal (panel_handle, SERIAL_TOGGLEBUTTON, &stopi);
			if(selstatus==1&&stopi==0)
			  {
			     imaqReadFile (image, pathname, NULL, NULL);
		        imaqDisplayImage (image, WINDOW_TO_USE, TRUE);
			  }
		    else
		      {  
		         
		          if(stopi==1)
		            {
			           GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_2, &datasource);
			             if(datasource==2)
			                {
			                	imaqReadFile (image, "D:\\temp2\\22.bmp", NULL, NULL);
			                	Drawimage();
		                        imaqDisplayImage (image, WINDOW_TO_USE, TRUE);
			                }
			             else
			                {
			                   if(*mapbuf==1)
    		                     {
    			
    	                            imaqReadFile (image, "d:\\temp\\index1.bmp", NULL, NULL); 
    								Drawimage();
    		                        imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
    		
    		                     }
                               else if(*mapbuf==2)
    		                     {
    			
    		                        imaqReadFile (image, "d:\\temp\\index2.bmp", NULL, NULL); 
    								Drawimage();
    		                        imaqDisplayImage (image, WINDOW_TO_USE, TRUE);	
			                     }				  
			                }
		            }
			  }  
			break;
		}
	return 0;
}

int CVICALLBACK ADJUST (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
		adjust_handle = LoadPanel (panel_handle, "serial.uir",TRACK);        
        InstallPopup (adjust_handle);
		SetCtrlVal (adjust_handle, TRACK_NUMERICSLIDE, adjust_heading); 
			break;
			
		}
	return 0;
}

int CVICALLBACK SELECT_HX (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
		{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_BINARYSWITCH_7, &hx_get_status);  
			if(hx_get_status==0)
			hx_gpshx_get=0;
			else hx_gpshx_get=1;
			if(hx_gpshx_get==0)
			dj_hx[djnum]=dj_chx;
			else if(hx_gpshx_get==1)
				    dj_hx[djnum]=dj_gpshx;
			break;
		}
	return 0;
}

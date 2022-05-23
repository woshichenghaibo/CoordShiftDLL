/*
�������ΪGEO����γ������ΪGPS
  ������꣨Geodetic Coordinate��:��ز������Բο�������Ϊ��׼������ꡣ�����P��λ���ô�ؾ���L�����γ��B�ʹ�ظ�H��ʾ�������ڲο���������ʱ�����ô�ؾ��Ⱥʹ��γ�ȱ�ʾ����ؾ�����ͨ���õ�Ĵ������������ʼ���������֮��ļнǣ����γ����ͨ���õ�ķ���������ļнǣ���ظ��ǵ�����ط��ߵ��ο�������ľ��롣 
*/

#include "stdafx.h"
#include "TPalgVC.h"
#include <CMATH>
#include <math.h>
#include <stdlib.h>
#include <stdio.h> 
#include <string.h>
#include <windows.h>

BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
		case DLL_THREAD_ATTACH:
		case DLL_THREAD_DETACH:
		case DLL_PROCESS_DETACH:
			break;
    }
    return TRUE;
}


double PI=3.14159265358979323846264338327950288;

TPALGVC_API int BL2XY(double arr[4],double B, double L,  double x,  double y, double a, double f, int beltWidth, bool assumedCoord)
{

            int beltNum=0;                           //ͶӰ�ִ��Ĵ���
			double ee = (2 * f - 1) / f / f;       //��һƫ���ʵ�ƽ��
            double ee2 = ee / (1 - ee);            //�ڶ�ƫ���ʵ�ƽ��
            double rB, tB, m;
            beltNum = (int)ceil((L - (beltWidth == 3 ? 1.5 : 0)) / beltWidth);
            if (beltWidth == 3 && beltNum * 3 == L - 1.5) beltNum += 1;
            L -= beltNum * beltWidth - (beltWidth == 6 ? 3 : 0);

            rB = B *  PI / 180;
            tB =tan(rB);
            m =cos(rB) * L *  PI / 180;
            double N = a / sqrt(1 - ee *  sin(rB) *  sin(rB));
            double it2 = ee2 *  pow( cos(rB), 2);
            x = m * m / 2 + (5 - tB * tB + 9 * it2 + 4 * it2 * it2) *  pow(m, 4) / 24 + (61 - 58 * tB * tB +  pow(tB, 4)) *  pow(m, 6) / 720;
            x = MeridianLength(B, a, f) + N * tB * x;
            y = N * (m + (1 - tB * tB + it2) *  pow(m, 3) / 6 + (5 - 18 * tB * tB +  pow(tB, 4) + 14 * it2 - 58 * tB * tB * it2) *  pow(m, 5) / 120);
        
            //����ɼٶ����꣬ƽ��500km��ǰ��Ӵ���
            if (assumedCoord) y += 500000 + beltNum * 1000000;
			arr[0]=B;
			arr[1]=L;
			arr[2]=x;
			arr[3]=y;
	return 1;

}
TPALGVC_API int XY2BL(double arr[4],double x, double y,  double B,  double L, double a, double f, int beltWidth)  //��˹ƽ��ת��Ϊ��γ������
{
            //���Ϊ�ٶ����꣬ת��Ϊ��Ȼ����
	int beltNum = 0;
	if (y > 1000000)
      {
            beltNum = (int) ceil(y / 1000000) - 1;
            y -= 1000000 * beltNum + 500000;
      }
            //���γ���뾭��
    if (y > 1000000)
      {
                //throw new Exception("coordinates type error,natural one please!");				
	        return 0;
      }
            double ee = (2 * f - 1) / f / f;       //��һƫ���ʵ�ƽ��
            double ee2 = ee / (1 - ee);            //�ڶ�ƫ���ʵ�ƽ��
            double cA, cB, cC, cD, cE;
            cA = 1 + 3 * ee / 4 + 45 * ee * ee / 64 + 175 *  pow(ee, 3) / 256 + 11025 *  pow(ee, 4) / 16384;
            cB = 3 * ee / 4 + 15 * ee * ee / 16 + 525 *  pow(ee, 3) / 512 + 2205 *  pow(ee, 4) / 2048;
            cC = 15 * ee * ee / 64 + 105 *  pow(ee, 3) / 256 + 2205 *  pow(ee, 4) / 4096;
            cD = 35 *  pow(ee, 3) / 512 + 315 *  pow(ee, 4) / 2048;
            cE = 315 *  pow(ee, 4) / 131072;
            double Bf = x / (a * (1 - ee) * cA);
            do
            {
                B = Bf;
                Bf = (x + a * (1 - ee) * (cB *  sin(2 * Bf) / 2 - cC *  sin(4 * Bf) / 4 + cD *  sin(6 * Bf) / 6) - cE *  sin(8 * Bf) / 8) / (a * (1 - ee) * cA);
            }
            while ( abs(B - Bf) > 0.00000000001);
            double N = a /  sqrt(1 - ee *  pow( sin(Bf), 2));
            double V2 = 1 + ee2 *  pow( cos(Bf), 2);
            double it2 = ee2 *  pow( cos(Bf), 2);
            double tB2 =  pow( tan(Bf), 2);
            B = Bf - V2 *  tan(Bf) / 2 * ( pow(y / N, 2) - (5 + 3 * tB2 + it2 - 9 * it2 * tB2) *  pow(y / N, 4) / 12 + (61 + 90 * tB2 + 45 * tB2 * tB2) *  pow(y / N, 6) / 360);
            L = (y / N - (1 + 2 * tB2 + it2) *  pow(y / N, 3) / 6 + (5 + 28 * tB2 + 24 * tB2 * tB2 + 6 * it2 + 8 * it2 * tB2) *  pow(y / N, 5) / 120) /  cos(Bf);
            B = B * 180 /  PI;
            L = L * 180 /  PI;
            //��⾭��
            L += beltWidth * beltNum - ((beltWidth == 6) ? 3 : 0);
			arr[0]=x;
			arr[1]=y;
			arr[2]=B;
			arr[3]=L;
	return 1;
}

TPALGVC_API double MeridianLength(double B, 
								  double a, 
								  double f)
{
            double ee = (2 * f - 1) / f / f; //��һƫ���ʵ�ƽ�� 
            double rB = B *  PI / 180; //����ת��Ϊ����
            //�����߻�����ʽ��ϵ��
            double cA, cB, cC, cD, cE;
            cA = 1 + 3 * ee / 4 + 45 *  pow(ee, 2) / 64 + 175 *  pow(ee, 3) / 256 + 11025 *  pow(ee, 4) / 16384;
            cB = 3 * ee / 4 + 15 *  pow(ee, 2) / 16 + 525 *  pow(ee, 3) / 512 + 2205 *  pow(ee, 4) / 2048;
            cC = 15 *  pow(ee, 2) / 64 + 105 *  pow(ee, 3) / 256 + 2205 *  pow(ee, 4) / 4096;
            cD = 35 *  pow(ee, 3) / 512 + 315 *  pow(ee, 4) / 2048;
            cE = 315 *  pow(ee, 4) / 131072;
            //�����߻���
	return a * (1 - ee) * (cA * rB - cB *  sin(2 * rB) / 2 + cC *  sin(4 * rB) / 4 - cD *  sin(6 * rB) / 6 + cE *  sin(8 * rB) / 8);
}
#include "stm32f37x.h"
#include <stdio.h>
#include "sdadc.h"
#include <math.h>
#include "comp.h"
#include "matrixkey.h"
#include "stmdataprocess.h"
#include "arm_math.h" 
#include "userVAinteraction.h"
#include "timer.h"
#include "flash_data.h"
#include "CalA.h"

#define TEST_LENGTH_SAMPLES 2048
 
 
////���ڶ��У�����������										
//extern float Cal_A_tab[][2];

////���ڶ��У�����������										
//extern float Cal_A_gain[][2];

const float sdadc2_sample100a[1024]={//ģ��ǯͷ����
23.978,
17.8682,
12.5654,
1.1528,
-6.1098,
-17.0613,
-26.9753,
-35.5059,
-40.8087,
-47.8407,
-57.6395,
-64.902,
-68.9368,
-84.3842,
-89.2259,
-91.8773,
-98.6787,
-105.5955,
-111.0136,
-118.7373,
-122.5415,
-126.8068,
-128.6513,
-134.4152,
-138.5652,
-129.5735,
-149.8626,
-146.1737,
-144.9056,
-145.0209,
-146.0584,
-143.407,
-145.2514,
-142.4847,
-140.6403,
-133.6082,
-133.9541,
-131.9944,
-119.3137,
-131.6485,
-118.3914,
-111.1289,
-105.0191,
-99.8315,
-90.6092,
-88.4189,
-78.1591,
-71.934,
-59.4839,
-43.806,
-40.1171,
-17.0613,
-27.4364,
-30.8947,
-17.5224,
-6.5709,
3.2278,
6.2251,
17.753,
22.0183,
34.0073,
45.7657,
52.2213,
56.2561,
73.7785,
66.1701,
78.2744,
86.6897,
92.9148,
97.8718,
124.3859,
126.1151,
130.7263,
135.7986,
141.1014,
145.9431,
144.0986,
149.0556,
154.4737,
147.9028,
152.8598,
154.3585,
152.6293,
152.2834,
150.2084,
146.4042,
143.6375,
137.2972,
136.9513,
131.6485,
127.2679,
124.2707,
112.9733,
108.1316,
104.9038,
98.5635,
91.5315,
81.9633,
73.6632,
65.3631,
60.1756,
53.0283,
42.4226,
35.5059,
17.5224,
-0.4611,
-3.8042,
-10.6057,
-19.5974,
-31.1253,
-39.1948,
-39.8865,
-47.0338,
-53.0283,
-58.3311,
-66.6312,
-76.7757,
-75.7382,
-96.3732,
-98.3329,
-101.4454,
-113.4344,
-120.5817,
-123.1179,
-131.8791,
-133.493,
-138.3347,
-137.0666,
-140.2944,
-144.7903,
-136.0291,
-154.4737,
-147.557,
-144.7903,
-144.7903,
-134.761,
-129.5735,
-130.7263,
-129.9193,
-125.8846,
-119.1984,
-116.2011,
-114.1261,
-100.4079,
-107.9011,
-96.8343,
-87.7272,
-80.2341,
-75.3924,
-65.1326,
-61.7895,
-51.5297,
-58.7922,
-45.6504,
-37.3504,
-14.2946,
5.3028,
-2.5361,
-5.8792,
3.2278,
12.6807,
23.5169,
33.5462,
38.9643,
46.1116,
53.2589,
66.1701,
73.2021,
76.891,
91.0703,
85.9981,
94.9898,
104.6732,
124.6165,
128.8818,
133.0319,
136.2597,
137.8736,
142.2542,
145.482,
150.3237,
149.0556,
137.643,
146.4042,
133.2624,
137.8736,
137.2972,
135.7986,
134.2999,
130.9568,
126.3457,
121.7345,
119.6595,
117.815,
112.0511,
105.3649,
104.2121,
86.4592,
84.2689,
79.0813,
72.0493,
63.8645,
56.4867,
47.2644,
38.7337,
34.3531,
26.86,
17.5224,
3.6889,
5.6487,
-21.3266,
-23.5169,
-28.4739,
-38.7337,
-46.9185,
-52.2213,
-64.7867,
-73.0868,
-78.5049,
-81.3869,
-89.8023,
-100.9843,
-99.2551,
-118.7373,
-118.5067,
-112.5122,
-117.815,
-124.7318,
-125.7693,
-133.6082,
-135.2222,
-138.3347,
-135.7986,
-135.7986,
-141.5625,
-135.3374,
-140.525,
-138.5652,
-135.9138,
-132.8013,
-129.5735,
-125.8846,
-132.4555,
-130.0346,
-123.0026,
-114.3567,
-108.5927,
-103.9816,
-96.4884,
-92.4537,
-90.6092,
-80.4647,
-71.0118,
-63.5187,
-54.8728,
-38.5032,
-34.1226,
-23.7475,
-15.3321,
-13.6029,
0.6917,
2.4209,
25.8225,
21.096,
31.3559,
38.7337,
50.4922,
57.87,
65.8243,
72.1646,
79.5424,
86.4592,
95.6815,
100.9843,
105.826,
116.2011,
107.9011,
117.4692,
119.1984,
122.8873,
125.654,
130.0346,
128.8818,
132.1096,
130.9568,
137.8736,
132.3402,
131.0721,
143.1764,
124.2707,
126.3457,
126.1151,
125.654,
139.8333,
136.9513,
129.804,
124.2707,
118.7373,
115.9706,
111.2441,
101.676,
98.4482,
71.7035,
68.2451,
61.5589,
54.6422,
44.0365,
36.6587,
22.1336,
16.3696,
15.2168,
9.107,
2.075,
-12.4501,
-14.4099,
-24.9002,
-40.0018,
-43.1143,
-50.838,
-58.677,
-66.8618,
-74.7007,
-84.1536,
-90.6092,
-96.0273,
-106.2872,
-114.5872,
-122.1956,
-122.3109,
-134.9916,
-135.7986,
-137.1819,
-139.6028,
-141.6778,
-142.8306,
-146.8653,
-149.2862,
-146.2889,
-143.5222,
-144.0986,
-146.6348,
-133.7235,
-146.9806,
-139.4875,
-132.8013,
-126.3457,
-123.579,
-118.622,
-120.0053,
-112.9733,
-104.4427,
-94.9898,
-88.5342,
-85.6522,
-69.859,
-74.5854,
-61.9048,
-53.2589,
-44.3824,
-37.5809,
-26.1683,
-19.3669,
-8.9918,
-1.7292,
10.1445,
17.5224,
22.5947,
45.6504,
34.5837,
49.6852,
60.1756,
68.9368,
75.1618,
86.2286,
89.687,
96.9496,
101.2149,
110.6677,
115.74,
116.4317,
126.8068,
121.9651,
129.5735,
134.6458,
138.3347,
139.0264,
141.9083,
138.7958,
139.6028,
140.1792,
142.9458,
152.9751,
148.7098,
149.632,
145.9431,
137.1819,
135.7986,
134.6458,
129.4582,
125.0776,
117.815,
109.515,
102.4829,
98.1023,
94.2981,
84.3842,
75.1618,
74.4702,
49.8005,
44.3824,
23.7475,
16.8307,
5.0723,
0,
-12.9112,
-18.2141,
-27.0905,
-32.5087,
-42.0768,
-53.9505,
-50.1463,
-77.698,
-78.3897,
-84.1536,
-91.0703,
-79.773,
-85.4217,
-96.3732,
-101.4454,
-105.2496,
-107.6705,
-114.1261,
-121.7345,
-118.2762,
-131.8791,
-130.7263,
-131.6485,
-131.418,
-134.1846,
-134.1846,
-135.1069,
-134.4152,
-139.718,
-132.1096,
-130.7263,
-130.7263,
-125.4235,
-125.4235,
-122.5415,
-116.2011,
-109.7455,
-104.9038,
-98.4482,
-94.0676,
-87.612,
-80.6952,
-71.2424,
-62.9423,
-56.6019,
-59.7145,
-39.8865,
-42.6532,
-30.3184,
-17.9835,
-11.2973,
-5.0723,
5.1876,
12.6807,
23.7475,
31.2406,
42.6532,
47.7255,
53.1436,
71.1271,
64.0951,
75.1618,
93.1454,
100.6385,
108.0163,
112.6275,
115.9706,
121.9651,
126.3457,
133.493,
137.8736,
137.9888,
149.8626,
137.1819,
144.0986,
147.3264,
149.7473,
147.6723,
150.9001,
139.0264,
136.1444,
132.1096,
135.3374,
132.8013,
124.9623,
128.4207,
111.59,
108.5927,
107.9011,
104.4427,
95.2204,
90.9551,
81.2716,
74.9313,
67.3229,
62.5964,
56.0255,
42.7685,
38.3879,
22.7099,
14.7557,
11.5279,
4.0348,
-7.7237,
-14.6404,
-25.1308,
-34.699,
-40.6935,
-48.6477,
-54.8728,
-68.2451,
-73.4327,
-78.0438,
-92.2231,
-96.1426,
-98.9093,
-105.3649,
-113.4344,
-118.9678,
-124.2707,
-128.4207,
-129.5735,
-132.1096,
-131.418,
-134.1846,
-130.0346,
-140.4097,
-138.7958,
-138.1041,
-138.1041,
-137.643,
-134.2999,
-136.4902,
-133.9541,
-130.7263,
-123.3484,
-120.8123,
-120.4665,
-103.2899,
-116.8928,
-113.3192,
-103.0593,
-95.912,
-90.2634,
-81.7327,
-75.5077,
-69.2826,
-62.0201,
-41.0393,
-30.3184,
-27.6669,
-11.5279,
-12.9112,
-1.7292,
5.7639,
17.6377,
27.2058,
34.8142,
39.7712,
50.4922,
57.4089,
70.7813,
77.1216,
81.6175,
97.7565,
89.4564,
100.6385,
109.515,
117.4692,
120.4665,
128.4207,
130.8416,
134.4152,
136.7208,
130.611,
132.1096,
132.3402,
138.3347,
132.3402,
134.1846,
136.0291,
137.4125,
134.4152,
134.6458,
129.6888,
126.3457,
122.4262,
122.4262,
119.4289,
111.3594,
108.0163,
117.5845,
106.2872,
103.4052,
98.4482,
90.2634,
82.0786,
74.816,
65.0173,
57.1783,
51.5297,
29.5114,
33.4309,
23.2863,
21.7877,
-0.6917,
-2.5361,
-10.0293,
-21.5572,
-31.01,
-37.3504,
-49.1088,
-54.1811,
-64.5562,
-66.6312,
-75.623,
-85.0758,
-82.5397,
-105.5955,
-106.0566,
-108.8233,
-112.1664,
-118.7373,
-122.1956,
-128.1901,
-138.5652,
-139.3722,
-140.4097,
-142.0236,
-145.9431,
-140.0639,
-150.3237,
-147.0959,
-143.407,
-141.7931,
-140.4097,
-135.568,
-136.8361,
-132.3402,
-129.804,
-120.3512,
-117.0081,
-115.0483,
-98.1023,
-111.3594,
-99.1399,
-87.0356,
-80.2341,
-74.0091,
-55.795,
-46.3421,
-48.4171,
-38.7337,
-26.6294,
-18.9057,
-15.7932,
3.4584,
-1.1528,
9.2223,
18.9057,
29.1656,
34.9295,
48.4171,
51.8755,
60.5214,
67.2076,
79.8883,
84.2689,
89.2259,
107.9011,
94.7593,
105.826,
123.579,
131.1874,
134.1846,
138.45,
139.8333,
142.2542,
143.7528,
150.0931,
148.8251,
146.2889,
149.8626,
142.2542,
142.9458,
143.8681,
143.6375,
139.4875,
138.5652,
132.5707,
127.6137,
122.8873,
121.0429,
116.6623,
105.9413,
99.9468,
89.4564,
74.9313,
70.8965,
65.9395,
57.6395,
49.1088,
40.1171,
30.2031,
36.4281,
32.5087,
21.7877,
9.914,
-0.807,
-3.4584,
-18.9057,
-27.0905,
-30.6642,
-40.5782,
-48.0713,
-57.87,
-68.9368,
-73.0868,
-80.9258,
-93.1454,
-103.5205,
-111.1289,
-105.0191,
-128.8818,
-129.1124,
-130.0346,
-133.2624,
-138.3347,
-139.6028,
-143.8681,
-145.7125,
-145.0209,
-144.3292,
-145.1361,
-147.7876,
-137.1819,
-149.4015,
-145.482,
-140.8708,
-136.4902,
-134.6458,
-129.6888,
-131.1874,
-124.0401,
-120.8123,
-108.8233,
-104.3274,
-100.5232,
-84.6147,
-92.4537,
-80.6952,
-69.6285,
-59.7145,
-55.5644,
-45.881,
-38.3879,
-29.5114,
-21.6724,
-9.7987,
-0.807,
4.957,
18.6752,
20.9808,
32.5087,
40.8087,
50.838,
57.87,
66.8618,
71.0118,
79.0813,
85.3064,
96.1426,
101.676,
106.4024,
114.2414,
115.9706,
117.6998,
125.1929,
131.1874,
132.5707,
136.4902,
137.643,
137.1819,
136.9513,
142.7153,
144.4445,
141.4472,
139.2569,
141.1014,
133.6082,
130.7263,
129.5735,
127.2679,
122.0804,
118.622,
112.7428,
105.826,
100.9843,
97.8718,
88.9953,
80.0036,
83.4619,
61.0978,
57.0631,
51.645,
46.688,
42.4226,
33.2003,
25.9378,
12.4501,
3.9195,
-4.3806,
-5.7639,
-17.5224,
-25.1308,
-41.5004,
-44.9588,
-53.6047,
-60.752,
-69.6285,
-76.5452,
-84.8453,
-104.4427,
-109.9761,
-114.0108,
-120.8123,
-127.9596,
-122.3109,
-142.4847,
-141.3319,
-142.0236,
-143.7528,
-147.7876,
-147.0959,
-150.439,
-151.4765,
-151.8223,
-148.2487,
-147.0959,
-134.6458,
-126.1151,
-135.7986,
-128.4207,
-123.1179,
-117.4692,
-114.3567,
-107.9011,
-105.3649,
-99.0246,
-94.2981,
-82.4244,
-76.7757,
-71.2424,
-62.4812,
-56.2561,
-50.4922,
-37.1198,
-25.8225,
-18.2141,
-8.3001,
0.6917,
3.1125,
11.5279,
23.2863,
33.6614,
42.6532,
50.031,
62.7117,
61.5589,
73.5479,
82.8855,
90.0328,
95.912,
99.601,
104.4427,
109.515,
115.74,
124.2707,
127.9596,
130.9568,
143.6375,
134.1846,
140.6403,
144.3292,
146.5195,
148.7098,
149.4015,
147.9028,
145.482,
148.5945,
134.4152,
130.4957,
127.2679,
130.3804,
117.5845,
114.5872,
113.665,
109.1691,
101.676,
98.2176,
90.8398,
83.6925,
76.6605,
73.3174,
64.7867,
54.5269,
57.2936,
44.3824,
43.5754,
29.5114,
21.903,
10.6057,
1.4986,
-6.6862,
-20.2891,
-24.5544,
-31.1253,
-38.9643,
-51.4144,
-51.645,
-74.4702,
-75.8535,
-81.6175,
-87.0356,
-96.3732,
-93.1454,
-103.4052,
-107.4399,
-114.472,
-113.665,
-118.0456,
-126.5762,
-123.579,
-138.45,
-136.1444,
-136.0291,
-136.4902,
-144.2139,
-142.9458,
-147.557,
-145.7125,
-144.5598,
-139.718,
-138.5652,
-138.1041,
-132.1096,
-135.2222,
-128.4207,
-121.8498,
-114.8178,
-112.0511,
-103.8663,
-99.4857,
-92.2231,
-87.0356,
-75.2771,
-68.0146,
-62.9423,
-50.6074,
-50.9533,
-44.2671,
-35.967,
-22.8252,
-19.3669,
-3.9195,
3.6889,
5.7639,
19.3669,
28.3586,
36.6587,
44.6129,
51.645,
64.4409,
63.8645,
72.3952,
82.3091,
106.5177,
111.7053,
121.0429,
123.1179,
128.7665,
131.8791,
140.2944,
143.6375,
136.2597,
145.0209,
128.9971,
135.7986,
139.2569,
141.2167,
139.718,
140.4097,
136.7208,
142.8306,
140.6403,
144.3292,
138.5652,
132.1096,
136.4902,
116.3164,
116.4317,
114.0108,
110.2066,
101.4454,
97.0648,
86.4592,
79.3119,
70.3201,
59.2534,
51.5297,
};
	
 
 extern defSysValue SysValue ;//ϵͳ����ʱ����Ҫ��������
 extern defFlashCal SaveData;	//������flash�еı�Ҫ����
 
extern const u16 length;

extern u16 count_for_Standby;//���߼���  ��������AC�Զ�����ʧЧʱ ��ʱ1�봥��һ��AC����
uint16_t count_for_ac=0;
/* ------------------------------------------------------------------- 
* External Input and Output buffer Declarations for FFT Bin Example 
* ------------------------------------------------------------------- */ 
 
//float Input[TEST_LENGTH_SAMPLES]={0};//�ƶ���RAMsave����
	//float Output[TEST_LENGTH_SAMPLES/2];
	float phase_angle[TEST_LENGTH_SAMPLES/4];
	float THD[TEST_LENGTH_SAMPLES/4];

union RAMsavedef RAMsave;

#define Test_Fs 10240
#define PI2 (6.283185307179586476925286766559)
//  
/* ------------------------------------------------------------------ 
* Global variables for FFT Bin Example 
* ------------------------------------------------------------------- */ 
uint16_t fftSize = TEST_LENGTH_SAMPLES/2; 
uint8_t ifftFlag = 0; 
uint8_t doBitReverse = 1;

uint32_t refIndex = 213, testIndex = 0; 
//static float32_t maxValue=0; 
unsigned int iiflag=0;
/*******************************������FFT�ĳ�ʼ��*************************************/


/*key*/
u8 key=0,page=0;

/******************����***********************/
uint16_t t;
uint8_t count=0;
uint8_t fft_count=0;
uint8_t addcount=5;//��ֵ�˲�����
uint16_t datasize=1024;
u16 inrush_current_100ms_count=0;
/******************��־***********************/

uint8_t pause_flag=0;//Һ����ʾ��ͣ��־��0-����ͣ��1-��ͣ
uint8_t af_flag=0;//��Ƶ��ʾ��־,1-��ѹ��2-������3-����
uint8_t r_or_f_flag=0;//THDѡ��ģʽ��0-THD%r,1-THD%f
u8 adjust_flag=0;
u8 inrush_trigger_flag=0;
/******************SDADC�ɼ��Ͳ����Ƶ��ɱ�־***********************/
uint8_t collect_finished=0;//SDADC������־
uint8_t voltage_capture_finished=0 , current_capture_finished=0;//������⣬�����Ƶ��־

/******************��ѹ�����������ʴ洢����***********************/
float SDADC1_value[1024]={0},SDADC2_value[1024]={0};
float POWER_value[1024]={0};

/******************��Ҫ�������***********************/
float voltage_effective=0,current_effective=0;//��Ч��ѹ����Ч����
float inrush_current_effective_100ms=0,inrush_current_effective_100ms_sum;//100ms��ӿ����
float maxv_value,maxi_value,minv_value,mini_value;//�����С|��ѹ������
float voltage_cf=0,current_cf=0;//��������[CF]
float apparent_power=0,active_power=0,reactive_power=0;//���ڹ��ʡ��й����ʡ��޹�����
float kWh,kVAh,kVarh,KgCO2;
float power_factor=0,d_power_factor=0;//��������[PF]��λ�ƹ�������[DPF]
float THD_r_voltage=0,THD_f_voltage=0,THD_r_current=0,THD_f_current=0,THD_r_power=0,THD_f_power=0;//��г�������ʣ����������Чֵ������+г��������[THD%r]/��Ի�����Чֵ[THD%f]

///******************�����г��ֵ��м���***********************/
//float voltage_sum=0,current_sum=0,power_sum=0;//����Vrms��Arms���й�����
//float voltage_temp=0,voltage_effective_before=0;
//float current_temp=0,current_effective_before=0;
//float power_temp=0,active_power_before=0;

float maxv=0,maxi=0,minv=0,mini=0;//���������Сֵ

float voltage_peak_value=0,current_peak_value=0;//�����ֵ->CF

float voltage_foudamental_effective=0,current_foudamental_effective=0,power_foudamental_effective=0;//���������Чֵ->THD%r��THD%f
//20160607lea �ܵ�г����������г����ѹ�ľ�����ֵ��������ѹ�����ܵ�ѹ��
float THDV2sum=0,THDA2sum=0,THDP2sum=0;//����г����ƽ���͡�

float voltage_foudamental_phase=0,current_foudamental_phase=0,power_foudamental_phase=0;//���������λ->DPF

float voltage_fundamental_frequency = 0 , current_fundamental_frequency = 0;
float voltage_effective_sum=0,current_effective_sum=0,active_power_sum=0;//����ۼ���ͣ��Ա��ֵ����

float voltage_mean,current_mean;
float voltage_mean_temp ,current_mean_temp;//2060616 �������ڵ�ѹ����ƽ��ֵ���˲�

u8 timer_1s_blink;
u16 powertimercounter;
float kWh_sum,kVAh_sum,kVarh_sum,KgCO2_sum;
uint8_t VadENA=0;

float testAdjV=0;

float Current_multiplication=2.0f/0.6f;
void dealwith_information(void)
{
	/******************�����г��ֵ��м���***********************/
	float voltage_sum=0,current_sum=0,power_sum=0;//����Vrms��Arms���й�����
	float voltage_temp=0,voltage_effective_before=0;
	float current_temp=0,current_effective_before=0;
	float power_temp=0,active_power_before=0;
	float voltage_mean_sum,current_mean_sum;//�����ѹ������ƽ��ֵDC
	
	float v_temp,a_temp;
	
	char    chardata[32];
	uint16_t   	Tloop=0;
	float temp_mean=0;

	/*********************************************
	*	�ɼ�������־������������Ӧ����
	*********************************************/
	if(collect_finished == 1)
	{
		collect_finished = 0;
		
		//������������ ��ȡ�������1024����
		get_formed1024();	
		
		for(t = 0;t < datasize;t++)
		{//������������		
			if(RotaryKeyValue==KEY_VALUE_6)//V+A ��λ  
			{
				if((funcstatus ==state0)||(funcstatus ==state2))//AC V+A    AC\DC
				{
					if(rangenum ==0)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_6VD_zero)*SaveData.Value.cal_6VD_gain);
					}
					else if(rangenum ==1)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_60VD_zero)*SaveData.Value.cal_60VD_gain);
					}
					else if(rangenum ==2)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_600VD_zero)*SaveData.Value.cal_600VD_gain);
					}	
				}
				else if(funcstatus ==state1)//DC
				{					
					if(rangenum ==0)
					{
						if(Sysflag.Calonce==1)//У׼ģʽ
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
							
							if(temp_mean<2000)//���
							{
								SaveData.Value.cal_6VD_zero=temp_mean;
								sprintf(chardata, "set 6VD_zero %.4f", SaveData.Value.cal_6VD_zero);
								printf(chardata); updata_flash();
							}							
							else//����
							{
								SaveData.Value.cal_6VD_gain=5.0f/(temp_mean-SaveData.Value.cal_6VD_zero);
								sprintf(chardata, "set 6VD_gain %.4f", SaveData.Value.cal_6VD_gain);
								printf(chardata); 
								updata_flash();
							}
						}					
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_6VD_zero)*SaveData.Value.cal_6VD_gain);	//						
					}
					else if(rangenum ==1)
					{
						if(Sysflag.Calonce==1)//У׼ģʽ
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
									
							if(temp_mean<2000)//���
							{
								SaveData.Value.cal_60VD_zero=temp_mean;
								sprintf(chardata, "set 60VD_zero %.4f", SaveData.Value.cal_60VD_zero);
								printf(chardata); updata_flash();
							}							
							else//����
							{
								SaveData.Value.cal_60VD_gain=50.0f/(temp_mean-SaveData.Value.cal_60VD_zero);
								sprintf(chardata, "set 60VD_gain %.4f", SaveData.Value.cal_60VD_gain);
								printf(chardata); 
								updata_flash();
							}					
						}
						
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_60VD_zero)*SaveData.Value.cal_60VD_gain);							
					}
					else if(rangenum ==2)
					{
						if(Sysflag.Calonce==1)//У׼ģʽ
						{
							Sysflag.Calonce	=0;
							temp_mean=0;
							for(Tloop=0;Tloop<datasize;Tloop++)
							{
								temp_mean+=(int16_t)(RAMsave.K4_tab.InjectedConvData[Tloop]&0xFFFF);
							}
							temp_mean=temp_mean/datasize;					
									
							if(temp_mean<2000)//���
							{
								SaveData.Value.cal_600VD_zero=temp_mean;
								sprintf(chardata, "set 600VD_zero %.4f", SaveData.Value.cal_600VD_zero);
								printf(chardata);updata_flash(); 
							}							
							else//����
							{
								SaveData.Value.cal_600VD_gain=500.0f/(temp_mean-SaveData.Value.cal_600VD_zero);
								sprintf(chardata, "set 600VD_gain %.4f", SaveData.Value.cal_600VD_gain);
								printf(chardata); 
								updata_flash();
							}					
						}
						
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_600VD_zero)*SaveData.Value.cal_600VD_gain);							
					}	
				}											
			}
			else if(RotaryKeyValue==KEY_VALUE_7)// W��λ
			{
				if(rangenum ==0)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_6VD_zero)*SaveData.Value.cal_6VD_gain);
					}
					else if(rangenum ==1)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_60VD_zero)*SaveData.Value.cal_60VD_gain);
					}
					else if(rangenum ==2)
					{
						SDADC1_value[t]=(float)((SDADC1_value[t]-SaveData.Value.cal_600VD_zero)*SaveData.Value.cal_600VD_gain);
					}	
			}
			
			
			//У����������    �������û��ǯͷʱģ���һ������
			SDADC2_value[t]=/*sdadc2_sample100a[t];*/(SDADC2_value[t]-SaveData.Value.cal_A1_zero)*SaveData.Value.cal_A1_gain;
			
			if(SaveData.Value.cal_adjv==1)//�Ƿ���д����ʱ����ֵУ�������޸����� ȷ����������ʼ������ĿǰΪ1600A,��Ҫ��С
			{//1  ��������ģʽ���Դ�������ж��У׼��
				//SDADC2_value[t]+=Adj_V(SDADC2_value[t]); 
				SDADC2_value[t] = Adj_Nline(SDADC2_value[t]);
			}

			
			//�����ֵ//����Сֵ
			if(t == 0)
			{
				maxv = SDADC1_value[t];minv = SDADC1_value[t];maxi = SDADC2_value[t];mini = SDADC2_value[t];
			}
			if(maxv < SDADC1_value[t])	maxv = SDADC1_value[t];
			if(minv > SDADC1_value[t])	minv = SDADC1_value[t];
			
			if(maxi < SDADC2_value[t]) 	maxi = SDADC2_value[t];
			if(mini > SDADC2_value[t])	mini = SDADC2_value[t];
			
			if((RotaryKeyValue == KEY_VALUE_6) && (longparamstatus != state0))
			{//������Inrush����
				if(inrush_trigger_flag == 0)
				{//δ����ʱ���ж��ں��������´���
					if(((rangestatus == state0) && (maxi >= 10)) || ((rangestatus == state1) && (maxi >= 100)))//1A@600A || 10A@2000A
					{//�����(maxi >= 1)��(maxi >= 10)�е�1��10������ֵ��Ҫת����maxi��Ӧ-32768~32767
						inrush_current_100ms_count=0;
						inrush_current_effective_100ms_sum=0;
						inrush_trigger_flag=1;
					}
				}
				else if(inrush_trigger_flag == 1)
				{//�Ѿ�����,����100ms(50Hzʱ,100ms��5�����ڣ�����Ƶ��Ϊ5120Hzʱ����5*5120/50=512����)
					if(inrush_current_100ms_count < 512)
					{
						inrush_current_effective_100ms_sum += SDADC2_value[t] * SDADC2_value[t];//SDADC2_value[t];
						inrush_current_100ms_count++;
						
						printf("%5f\r\n",SDADC2_value[t]); //20160615 test ���� ����ӿ�������ݴ���
					}
					else if(inrush_current_100ms_count == 512)
					{
						inrush_current_100ms_count=0;
						inrush_current_effective_100ms = sqrt(inrush_current_effective_100ms_sum / 512);
						inrush_current_effective_100ms_sum=0;
						inrush_trigger_flag = 2;
					}
				}
				else if(inrush_trigger_flag == 2)//20160531lea�ɹ���ȡ��ӿ������Ĳ���
				{
					
				}
			}
			
			
			
			//������㹦��˲ʱֵ ��˲ʱֵ�ľ�ֵ�õ��й�����
			POWER_value[t] = SDADC1_value[t] * SDADC2_value[t];
			
			//��ѹ��������������Чֵ���㣬����һ���֡�����ƽ���ͣ�V/A������ͣ�W��
			
			voltage_sum += SDADC1_value[t] * SDADC1_value[t];//* SDADC_CAL_COEF2;//�������ʱ�������ѹ
			current_sum += SDADC2_value[t] * SDADC2_value[t];//* SDADC_CAL_COEF2;//�������ʱ���������
																																		//20160419 lea  ��������ǯͷ����  
			power_sum += SDADC1_value[t] * SDADC2_value[t];//���й�����//ƽ������
			
			voltage_mean_sum += SDADC1_value[t] ;//* SDADC_CAL_COEF;
			current_mean_sum += SDADC2_value[t] ;//* SDADC_CAL_COEF;
		}
		//��ѹ������ƽ��ֵ��DC
		voltage_mean = voltage_mean_sum / datasize;
		current_mean = current_mean_sum / datasize;
		
		

		
		//��ѹ��������������Чֵ���㣬������һ���֡��������������V/A���������W��
		voltage_temp = sqrt(voltage_sum / datasize);
		current_temp = sqrt(current_sum / datasize);//20160419lea ����ֵ��ȡ 
		power_temp = power_sum / datasize;
/************************************************************************************************************************/		
		SysValue.curr_now=current_mean;//Ŀǰֻ�����ڴ��ݸ�У׼���֣�ȫ����Ȼʹ��֮ǰ�ı������߼�
				
//		printf("v,c:%.4f,%.4f",voltage_temp,current_temp);
//		printf("vm,cm:%.4f,%.4f\r\n",voltage_mean,current_mean);
/*************************************************************************************************************************/
		//�ۼ�addcount�Σ����ں���addcount�ε�������ƽ��ֵ
		voltage_effective_sum += voltage_temp;
		current_effective_sum += current_temp;
		active_power_sum += power_temp;
	
		if(SaveData.Value.cal_adjv==0)//
		{//0 У׼ģʽ�µ���У����ʾ 
			current_mean = Adj_Nline(current_mean);
		}
		
		count++;
		//����addcount���Ѿ��õ�������ֵ�˲���ĵ�ѹ�͵�����Чֵ���й�����
		if(count == addcount)
		{
			//addcount�εľ�ֵ�����õ���ѹ��������������Чֵ
			voltage_effective=voltage_effective_sum / addcount;
			current_effective=current_effective_sum / addcount;
			active_power=active_power_sum / addcount;//ƽ����� �й�����ֵ
			
			//printf("ve,ce:%.4f,%.4f\r\n",voltage_effective,current_effective);
			
			if(((RotaryKeyValue==KEY_VALUE_6) && ((paramstatus == state1) || (paramstatus == state2))) || ((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0)))
				fft_count++;
			
			if(((RotaryKeyValue==KEY_VALUE_6) && (peakstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_6) && (paramstatus == state3)))
			{//VA��Peak����VA����CF��
				//�����Сֵ����
//				maxv_value = (float)maxv ;
//				minv_value = (float)minv ;
//				maxi_value = (float)maxi ;
//				mini_value = (float)mini ;
				if(maxv > maxv_value)	maxv_value = maxv;
				if(minv < minv_value)	minv_value = minv;
				if(maxi > maxi_value)	maxi_value = maxi;
				if(mini < mini_value)	mini_value = mini;
				
				//�����ֵ����ҪΪ�˽������ѹ�������Ĳ�������
				if(maxv_value > fabs(minv_value))//��ֵ(2014-10-31ע�ͣ�ֻ������������������ֵ)
					voltage_peak_value = maxv_value;
				else	voltage_peak_value = fabs(minv_value);
				if(maxi_value > fabs(mini_value))
					current_peak_value = maxi_value;
				else	current_peak_value = fabs(mini_value);
			}
			if((RotaryKeyValue==KEY_VALUE_6) && (paramstatus == state3))
			{//VA����CF��
				if(voltage_effective == 0)	voltage_cf = 0;
				else	voltage_cf = voltage_peak_value / voltage_effective;//��������(CF)
				if(current_effective == 0)	current_cf = 0;
				else	current_cf = current_peak_value / current_effective;
			}
			
			if(RotaryKeyValue==KEY_VALUE_7)
			{//�������ڹ��ʡ��޹����ʡ�����������PF��
				apparent_power = voltage_effective * current_effective;//���ڹ���(VA)
				
				if(apparent_power <= active_power)
					reactive_power = 0;
				else
					reactive_power = sqrt(apparent_power * apparent_power - active_power * active_power);//�޹�����(Var)
			}
			
			
			if((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0) && (paramstatus == state0))
			{//W����PF��
				power_factor = active_power / apparent_power;//��������(PF)
				if(power_factor > 1)	power_factor = 1;
			}
			count=0;
			voltage_effective_sum = 0;
			current_effective_sum = 0;
			active_power_sum = 0;
		}
			
		if((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus != state0))
		{//W���������
//			if(powertimer==1)//�ſ�ʼ�������ʱ������������
//			{
//				kWh=0;kVAh=0;kVarh=0;
//			}
			powertimercounter++;//
			/*kWh,kVAh,kVarh,KgCO2;*/
			kWh_sum += active_power;// * (float)powertimer/3600/1000;
			kVAh_sum += apparent_power;// * (float)powertimer/3600/1000;
			kVarh_sum += reactive_power;// * (float)powertimer/3600/1000;
			if(powertimer_1s_flag == 1)//1���ӱ�־����
			{
				powertimer_1s_flag=0;
				timer_1s_blink=1;
				kWh += kWh_sum / powertimercounter/3600/1000;
				kVAh += kVAh_sum / powertimercounter/3600/1000;
				kVarh += kVarh_sum / powertimercounter/3600/1000;
				
				powertimercounter=0;
				kWh_sum=0;kVAh_sum=0;kVarh_sum=0;
			}
		}
			
		maxv=0;maxi=0;minv=0;mini=0;
		voltage_sum=0;current_sum=0;power_sum=0;
		voltage_mean_sum=0;current_mean_sum=0;//�޸�һ������֮ǰ������current_mean_sum=0;�޸�һ��Ϊvoltage_mean_sum=0;
		
		/*********************************************
		*	FFT�������,ֻ����ťVA+W������
		*********************************************/
		if(((RotaryKeyValue==KEY_VALUE_6) && ((paramstatus == state1) || (paramstatus == state2))) || ((RotaryKeyValue==KEY_VALUE_7) && (longparamstatus == state0)))
		{//VA����THD%r��THD%f����W����Param��
			if(fft_count>=1)						//ÿ��fft_count�λ������㣬����һ��FFT
			{
				FFT();//��Ƶ
				
				if((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1))
				{
					//����λ�ƹ���������DPF��
					d_power_factor= cos((current_foudamental_phase- voltage_foudamental_phase)*PI/180);
				}
				
				//��ѹ��г��������(THD%r%f)
				if(voltage_effective< voltage_foudamental_effective)
					voltage_effective= voltage_foudamental_effective;
				
				if(voltage_effective == 0)	THD_r_voltage=0;
				//else	THD_r_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_effective* voltage_effective));		
				else THD_r_voltage=sqrt(THDV2sum)/voltage_effective;
				
				if(voltage_foudamental_effective == 0)	THD_f_voltage=0;
				//else	THD_f_voltage= sqrt((voltage_effective* voltage_effective- voltage_foudamental_effective* voltage_foudamental_effective)/(voltage_foudamental_effective* voltage_foudamental_effective));
				else THD_f_voltage=sqrt(THDV2sum)/voltage_foudamental_effective;
				
				//������г��������(THD%r%f)
				if(current_effective< current_foudamental_effective)
					current_effective= current_foudamental_effective;
				
				if(current_effective == 0)	THD_r_current=0;
				else	THD_r_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_effective* current_effective));
				
				if(current_foudamental_effective == 0)	THD_f_current=0;
				else	THD_f_current= sqrt((current_effective* current_effective- current_foudamental_effective* current_foudamental_effective)/(current_foudamental_effective* current_foudamental_effective));
				
				//������г��������(THD%r%f)
				if(active_power< power_foudamental_effective)
					active_power= power_foudamental_effective;
				
				if(active_power == 0)	THD_r_power=0;
				else	THD_r_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(active_power* active_power));
				
				if(power_foudamental_effective == 0)	THD_f_power=0;
				else	THD_f_power= sqrt((active_power* active_power- power_foudamental_effective* power_foudamental_effective)/(power_foudamental_effective* power_foudamental_effective));
				
				//����
				fft_count = 0;
				voltage_effective_sum = 0;
				current_effective_sum = 0;
				active_power_sum = 0;
			}
		}
		
		if(funcstatus ==state0)//ac
		{
			VadENA=0;//SDADC��ʹ�ܱ�־���ݵ��Ƚ����жϣ�����㿪ʼADC�ɼ�
			if(phasestatus != state0)
			{
				TIM_Cmd(TIM19, ENABLE);
			}
			//TIM_Cmd(TIM19, ENABLE);//test  ����ʱ��ACֱ�Ӳ���
		}
		else if((funcstatus ==state1)||(funcstatus ==state2))//ACDC V+A   
		{
			TIM_Cmd(TIM19, ENABLE);
		}
		//VadENA=0;//SDADC��ʹ�ܱ�־���ݵ��Ƚ����жϣ�����㿪ʼADC�ɼ�
		//TIM_Cmd(TIM19, ENABLE);
		
		count_for_ac = count_for_Standby;
	}
	//��ʱ����һ��AC����������ʱ����    2Sһ��   ������ֵ��addcount�˲���������ֵ���»�Ƚ�����
	if(count_for_Standby-count_for_ac>=2)
	{
		TIM_Cmd(TIM19, ENABLE);
	}
}


//2015-10-30JEFF����ѹ����������ʱ��ʱ����Ϊstm32�����õ��ĵ�ѹֵ
void FFT(void)
{
  unsigned int ii=0;
	uint8_t fundamental_flag=0;
	float temp_frequency;
	
	
	
  arm_status status;
  arm_cfft_radix4_instance_f32 S;  
	
     
  status = ARM_MATH_SUCCESS;
   
  /* Initialize the CFFT/CIFFT module */  
  status = arm_cfft_radix4_init_f32(&S, fftSize,ifftFlag, doBitReverse);
	
/*****************************************��ѹFFT********************************************************/
	if(((RotaryKeyValue==KEY_VALUE_6) && (paramstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1)))//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = ((float)SDADC1_value[ii]);//
			RAMsave.Input[2*ii+1] = 0;
		}	 
		arm_cfft_radix4_f32(&S,RAMsave.Input);//FFT����
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);//�����ֵ	 
		//���ڶ�����һ���������ڣ��˴�output ��ռ��input����ĺ���һ���֣���Ӱ�����ǰ��ʹ�г���ļ��㡣
		
//		printf("---------------------��ѹ-----------------------------\r\n");
//		printf("��ѹƵ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		THDV2sum=0;
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
					//printf("ֱ����ѹ ,  V=%2.5f V\r\n", RAMsave.K4_tab.Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//��ֵ
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//���
					
					if(fabs(temp_frequency-50)<0.5 /*|| fabs(temp_frequency-60)<0.5*/)//ȷ����Ƶ��Ϊ��Ƶ
					{
						fundamental_flag= ii;//������iiֵ����fundamental_flag����������
						voltage_foudamental_phase= phase_angle[ii];
						voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
						THDV2sum -= RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];//����������ѹ��ֵ
					}
//					else if(fabs(temp_frequency-60)<0.5)//�жϻ�Ƶ�ǲ���60HZ
//					{
//						RAMsave.K4_tab.Output[ii]/1.41421356f>voltage_foudamental_effective;
//						voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
//					}
					//�����г�������Ļ�����
					if(r_or_f_flag==0)
					{
						if(voltage_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ voltage_effective;//THD%r
					}
					else
					{
						if(voltage_foudamental_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ voltage_foudamental_effective;//THD%f
					}
					THDV2sum+=RAMsave.K4_tab.Output[ii]*RAMsave.K4_tab.Output[ii];
					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("V=%2.5f V  ,  ", RAMsave.K4_tab.Output[ii]);
//					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
//					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}

/*******************************************����FFT******************************************************/	
	if(((RotaryKeyValue==KEY_VALUE_6) && (paramstatus != state0)) || ((RotaryKeyValue==KEY_VALUE_7) && (paramstatus == state1)))//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = 0.0f+ (float)SDADC2_value[ii];					//�ֶ������һ����ֵΪ10��ֱ������
			RAMsave.Input[2*ii+1] = 0;
		}
		arm_cfft_radix4_f32(&S,RAMsave.Input);
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);
		
		
//		printf("---------------------����-----------------------------\r\n");
//		
//		printf("����Ƶ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
					//printf("ֱ������ ,  I=%2.5f A\r\n", RAMsave.K4_tab.Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//��ֵ
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//���
					
					if(fabs(temp_frequency-50)<0.5f /*|| fabs(temp_frequency-60)<0.5f*/)//ȷ����Ƶ��Ϊ��Ƶ
					{
						fundamental_flag= ii;
						current_foudamental_phase= phase_angle[ii];
						current_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
					}
//					else if(fabs(temp_frequency-60)<0.5)//�жϻ�Ƶ�ǲ���60HZ
//					{
//						RAMsave.K4_tab.Output[ii]/1.41421356f>voltage_foudamental_effective;
//						voltage_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
//					}
					//�����г�������Ļ�����
					if(r_or_f_flag==0)
					{
						if(current_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ current_effective;//THD%r
					}
					else
					{
						if(current_foudamental_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ current_foudamental_effective;//THD%f
					}
					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("I=%2.5f A  ,  ", Output[ii]);
//					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
//					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}
/************************************************����FFT*************************************************/		
	if(RotaryKeyValue==KEY_VALUE_7)//if(pause_flag==0 && af_flag==0)
	{
		for(ii=0;ii<fftSize;ii++)
		{
			RAMsave.Input[2*ii] = 0.0f+ (float)POWER_value[ii];					//�ֶ������һ����ֵΪ10��ֱ������
			RAMsave.Input[2*ii+1] = 0;
		}
		arm_cfft_radix4_f32(&S,RAMsave.Input);
		arm_cmplx_mag_f32(RAMsave.Input, RAMsave.K4_tab.Output, fftSize);
		
//		printf("---------------------����-----------------------------\r\n");

//		printf("����Ƶ�ʷֱ���-Fhz=%2.2f \r\n", (float32_t)SAMPLING_FREQ/fftSize);
		for(ii=0;ii<fftSize/2;ii++)
		{
			if(RAMsave.K4_tab.Output[ii]/fftSize>0.000001f)
			{
				if(ii==0)
				{
//					printf("ֱ��(�ֶ����10W)  ,  Pw=%2.5f W\r\n", Output[ii]/fftSize);
				}
				else
				{
					temp_frequency= (float32_t)ii*SAMPLING_FREQ/fftSize;//Ƶ��
					RAMsave.K4_tab.Output[ii]=RAMsave.K4_tab.Output[ii]*2/fftSize;//��ֵ
					phase_angle[ii]= atan2(RAMsave.Input[2*ii],RAMsave.Input[2*ii+1])* 360/PI2;//���
					
					if(fabs(temp_frequency-100)<0.5f /*|| fabs(temp_frequency-120)<0.5f*/)//ȷ����Ƶ��Ϊ��Ƶ
					{
						fundamental_flag= ii;
						power_foudamental_phase= phase_angle[ii];
						power_foudamental_effective= RAMsave.K4_tab.Output[ii]/1.41421356f;
					}
					
					//�����г�������Ļ�����
					if(r_or_f_flag==0)
					{
						if(active_power==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ active_power;//THD%r
					}
					else
					{
						if(power_foudamental_effective==0)	THD[ii]=0;
						else	THD[ii]= RAMsave.K4_tab.Output[ii]/1.41421356f/ power_foudamental_effective;//THD%f
					}
					
//					printf("%d----Fhz=%2.2f  ,  ",ii, temp_frequency);
//					printf("Pw=%2.5f W  ,  ", Output[ii]);
//					printf("Phase=%2.5f ��  ��  ", phase_angle[ii]);
//					printf("THD=%2.5f(�ٷ���) \r\n\r\n", THD[ii]*100);
				}
			}
		}
	}
}
/************************************************************************************************
*���������������������Ӧ�ĵ�ѹֵ
*�����������޺󼴿�ʼ����
*������ʽΪ y=a1*exp(x/t1)+y0;
* 0.00137*exp(x/(206.147))+1.5688
*/
float Adj_V(float sdadc_value)
{
	float new_value=0;
	
	if(sdadc_value>1500.0f)
	{
		//SaveData.Value.cal_A_a1=0.000640519;
		//SaveData.Value.cal_A_t1=1484.56402;
		//SaveData.Value.cal_A_y0=64.17008;
		
		//new_value=0.00064*exp((sdadc_value/1484.56402))+64.17008;
		new_value=SaveData.Value.cal_A_a1*exp((sdadc_value/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
		return new_value;
	}
	else if((sdadc_value<-1500.0f))
	{
		sdadc_value*=-1;
		new_value=SaveData.Value.cal_A_a1*exp((sdadc_value/SaveData.Value.cal_A_t1))+SaveData.Value.cal_A_y0;
		return new_value*-1;
	}
	else 
	{
		return 0;
	}			
}


/*********************************************/
//����������
//ʹ��SDADC�������
/***********************************************/
uint8_t get_formed1024(void)
{
	int16_t tt=0,loop=0;
	int16_t temp[5]={0};//��������������ơ�

	if(funcstatus ==state0)//ac ֱ�ӽ����ɹ����Ƚϵõ���1024����
	{
		for(loop = 0;loop < 1024;loop++)//��ȡ1024�������ڼ���
		{
			SDADC1_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop]&0xFFFF);
			SDADC2_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop]>>16);	
		}
	}
	else if((funcstatus ==state1)||(funcstatus ==state2))//ACDC V+A   ��ȡ1024��
	{
		for(tt=1;tt<256;tt++)
		{
			temp[0] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt-1]&0xFFFF;
			temp[1] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt]&0xFFFF;
			temp[2] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt+1]&0xFFFF;
			temp[3] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt+2]&0xFFFF;
			temp[4] =  (int16_t)RAMsave.K4_tab.InjectedConvData[tt+3]&0xFFFF;
			if(temp[1]>0)
			{
				if((temp[1]>temp[0])&&(temp[2]>temp[1])&&(temp[2]<temp[3])&&(temp[3]<temp[4]))
				{
					tt--;
					break;
				}
			}	
			else
			{				
			}	
		}
		
		for(loop = 0;loop < 1024;loop++)//��ȡ1024�������ڼ���
		{
			SDADC1_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop+tt]&0xFFFF);
			SDADC2_value[loop] = (int16_t)(RAMsave.K4_tab.InjectedConvData[loop+tt]>>16);	
		}
	}
	return 1;

}



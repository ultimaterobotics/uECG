#include "imu_processor.h"
#include "urf_timer.h"

uint32_t last_imu_time = 0;
uint32_t cur_max_time = 0;
uint32_t prev_max_time = 0;

float zz_max = 0;
uint32_t total_steps = 8;
float step_time_avg = 0;
uint32_t step_candidate = 0;

void process_imu_data(sBMI160 *bmi_data)
{
	uint32_t ms = millis();
	if(last_imu_time + 10 > ms) return;
	last_imu_time = ms;
	
	float ax = bmi_data->aX;
	float ay = bmi_data->aY;
	float az = bmi_data->aZ;
	float zz = ax*ax + ay*ay + az*az - 9.8f*9.8f;
	int max_dt = ms - cur_max_time;
	if(zz > zz_max && zz > 10)
	{
		zz_max = zz;
		cur_max_time = ms;
//		bmi_data->step_cnt = max_dt;
		if(max_dt > 200 && max_dt < 2000)
		{
			float step_time = cur_max_time - prev_max_time;
			prev_max_time = cur_max_time;
			step_time_avg *= 0.8;
			step_time_avg += 0.2*step_time;
			if(step_time > step_time_avg * 0.7 && step_time < step_time_avg * 1.3)
			{
				total_steps++;
			}
			
			bmi_data->step_cnt = total_steps;
		}
	}
	if(max_dt < 200)
		zz_max *= 0.98;
	else if(max_dt < 500)
		zz_max *= 0.99;
	else
		zz_max *= 0.995;
}
#include "interaction.h"
#include <stdlib.h>
#include <math.h>
#include <climits>

#define LINE_LENGTH 1023

CInteraction::CInteraction(double scale) : m_scale(scale), m_length(0)
{
}

CInteraction::~CInteraction()
{
}

bool CInteraction::Load(FILE *f)
{
	char line[LINE_LENGTH+1];
	bool comment = false;
	int idx=0;
	char *ptr = NULL;
	unsigned long timestamp, prev_timestamp;
	unsigned long x, y, prev_x, prev_y;
	measurement *meas;

	rewind(f);
	int c = fgetc(f);
	prev_x = 0;
	prev_y = 0;
	memset(line, 0, LINE_LENGTH+1);

	while (c != EOF)
	{
		if (c == '\n')
		{
			line[idx++] = 0;
			
			if (!comment)
			{
				meas = new measurement;
				memset(meas, 0, sizeof(measurement));

				// process previous line
				ptr = line;
				timestamp = strtol(ptr, &ptr, 10);
				meas->timestamp = timestamp;
				ptr++;
				if (ptr)
				{
					x = strtol(ptr, &ptr, 10);
					meas->player.x = x;
					ptr++;
				}
				if (ptr)
				{
					y = strtol(ptr, &ptr, 10);
					meas->player.y = y;
					ptr++;
				}
				if (ptr)
				{
					x = strtol(ptr, &ptr, 10);
					meas->robot.x = x;
					ptr++;
				}
				if (ptr)
				{
					y = strtol(ptr, &ptr, 10);
					meas->robot.y = y;
					ptr++;
				}
				if (ptr)
				{
					x = strtol(ptr, &ptr, 10);
					meas->ball.x = x;
					ptr++;
				}
				if (ptr)
				{
					y = strtol(ptr, &ptr, 10);
					meas->ball.y = y;
					ptr++;
				}

				//printf("Adding meas: timestamp=%lu p.x=%f p.y=%f r.x=%f r.y=%f b.x=%f b.y=%f\n",
				//	meas->timestamp, meas->player.x, meas->player.y, meas->robot.x, meas->robot.y, meas->ball.x, meas->ball.y);
				m_measurements.push_back(meas);
			}
			idx = 0;
			comment = false;
		}
		else if (c == '#')
		{
			comment = true;
		}

		if (!comment)
		{
			if (idx == LINE_LENGTH)
			{
				printf("line too long!\n");
				continue;
			}
			line[idx++] = c;
		}
		c = fgetc(f);
	}

	// interactions always start at 0, even if the first measurement isn't exactly at 0
	//m_length = timestamp/1000000; // in milliseconds
	m_length = m_measurements.size(); // number of samples

	return true;
}

void CInteraction::get_sample(double phase, double sample[])
{
	//uint64_t elapsed_time_ns = phase * m_length * 1000000;
	//printf("Elapsed time: %lu out of %lu\n", elapsed_time_ns, m_length);
	uint64_t sample_index;

	if (phase < 0)
	{
		printf("%s: phase too small\n", __func__);
		phase = 0;
	}
	if (phase > 1)
	{
		printf("%s: phase too large\n", __func__);
		phase = 1;
	}
	
	sample_index = phase * m_length;

	// look up data at that timestamp - linear search for now, can be improved
	measurement *current_sample = NULL;
	for (measurement_list::iterator i=m_measurements.begin(); i != m_measurements.end(); i++)
	{
		current_sample = *i;
		//printf("Looking at timestamp %lu\n", current_sample->timestamp);
		//if (current_sample->timestamp >= elapsed_time_ns)
		//	break; 
		if (!sample_index--)
			break;
	}

	//sample[STATE_VAR_PHASE] = phase;//(double)current_sample->timestamp / (double)m_length / (double)1000000;
	//sample[STATE_VAR_PHASE_VEL] = 0.01;
	sample[STATE_VAR_BALL_X] = current_sample->ball.x;
	sample[STATE_VAR_BALL_Y] = current_sample->ball.y;
	sample[STATE_VAR_ROBOT_X] = current_sample->robot.x;
}


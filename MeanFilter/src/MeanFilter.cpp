#include "MeanFilter.h"
float MeanFilter::input(float inVal)
{
	int i;
	float sum = 0.0f;
	float result = 0.0f;
	for (i = 1 ; i<HISTORY_NUM; i++)
	{
		buffer_[i - 1] = buffer_[i];
	}
	buffer_[HISTORY_NUM - 1] = inVal;
	for (i = 0 ; i < HISTORY_NUM; i++)
	{
		sum += buffer_[i];
	}
	result = sum / HISTORY_NUM;
	return result;
}
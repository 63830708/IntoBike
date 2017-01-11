#ifndef MEAN_FILTER_H_
#define MEAN_FILTER_H_
#define HISTORY_NUM 10 
class MeanFilter
{
	public:
	float input(float inVal);
	private:
	float buffer_[HISTORY_NUM] =
	{
		0
	}
	;
}
;
#endif
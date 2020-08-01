/*
 * SensorFilter.cpp
 *
 *  Created on: Mar 2, 2018
 *      Author: spam180
 */

#include <SensorFilter.h>

#include <numeric>

SensorFilter::SensorFilter(unsigned nvalues) : mSize(nvalues)
{
}

void SensorFilter::SetSize(unsigned values)
{
	mSize = values;
}

void SensorFilter::Clear() {
	mData.clear();
}

void SensorFilter::AddValue(double val)
{
	mData.push_back(val);
	while (mData.size() > mSize)
		mData.pop_front();
}

double SensorFilter::GetAverage()
{
	double sum = std::accumulate(mData.begin(), mData.end(), 0.0);
	return sum / mData.size();
}


double SensorFilter::GetCurAvg()
{
	return curTxAvg;
}
double SensorFilter::ExpMovingAvg(double newVal, double curAvg, double alpha)
{
	curAvg = curTxAvg;
	// Defaulted alpha to 0.5. Need to test and tune to see what to set this to.
	// This equation should work for a frame of up to about 30 values.
	// This should set a private variable somewhere.
	curTxAvg = (alpha * newVal) + (1.0 - alpha) * curAvg;
	return curTxAvg;
}
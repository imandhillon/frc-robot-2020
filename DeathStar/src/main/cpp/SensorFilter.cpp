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

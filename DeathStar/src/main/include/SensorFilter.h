/*
 * SensorFilter.h
 *
 *  Created on: Mar 2, 2018
 *      Author: spam180
 */

#ifndef SRC_SENSORFILTER_H_
#define SRC_SENSORFILTER_H_

#include "deque"

class SensorFilter {
	std::deque<double>	mData;
	unsigned			mSize;

public:
	SensorFilter(unsigned values = 5);

	void Clear();
	void SetSize(unsigned values);
	void AddValue(double val);
	double GetAverage();

};

#endif /* SRC_SENSORFILTER_H_ */

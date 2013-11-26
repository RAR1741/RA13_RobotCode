#include "LookupTable.h"
#include <iostream>
#include <fstream>
#include <algorithm>

bool PairLess(std::pair<double,double> x1, std::pair<double,double> x2) {
	return x1.first < x2.first;
}

LookupTable::LookupTable(double below_min, double above_max)
{
	m_below_min = below_min;
	m_above_max = above_max;
}

void LookupTable::LoadFromFile(const char * filename)
{
	Clear();
	std::ifstream infile(filename, std::ios_base::in);
	
	if (!infile.is_open()) {
		std::cerr << "Error loading file, nothing loaded." << std::endl;
		return;
	}
	
	float x,y;
	while (infile >> x >> y) {
		AddEntry(x,y);
	}
	
	return;
}

void LookupTable::AddEntry(double x, double y)
{
	for (unsigned int i = 0; i < m_yvals.size(); ++i)
	{
		// If this is really close to an existing x value
		if (SortaEqual(m_yvals[i].first, x)) {
			// Don't insert value, that would make divide-by-zero
			// likely
			return;
		}
	}
	
	//TODO: Rewrite this, this is stupid. Should be able to use insertion
	// in order and cut down on the overhead of a true sort.
	m_yvals.push_back(std::pair<double,double>(x,y));
	std::sort(m_yvals.begin(), m_yvals.end(), PairLess);
}

double LookupTable::MinX()
{
	if (m_yvals.size() == 0) {
		return 0;
	} else 
		return m_yvals.front().first;
}

double LookupTable::MaxX()
{
	if (m_yvals.size() == 0) {
		return 0;
	} else
		return m_yvals.back().first;

}

double LookupTable::Lookup(double x)
{
	if (x < MinX()) return m_below_min;
	if (x > MaxX()) return m_above_max;

	double x1,y1,x2,y2;

	for (unsigned int i = 0; i < m_yvals.size() - 1; ++i) {
		if (m_yvals[i+1].first > x) {
			x1 = m_yvals[i].first;
			x2 = m_yvals[i+1].first;
			y1 = m_yvals[i].second;
			y2 = m_yvals[i+1].second;
			break;
		}
	}

	// Linearly interpolate between the two values
	double y = (y2 - y1) * ((x - x1) / (x2 - x1)) + y1;

	return y;
//TODO: Neural Network
}

void LookupTable::Clear()
{
	m_yvals.clear();
}

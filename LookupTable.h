#ifndef LOOKUP_TABLE_H__
#define LOOKUP_TABLE_H__

#include <cmath>
#include <vector>

// Class to implement simple table lookups
class LookupTable {
public:
	// Creates a lookup table.
	// below_min is what to return if to the left of minimum x
	// above_max is what to return if to the right of maximum x
	LookupTable(double below_min, double above_max);
	
	// Add a new point to the graph
	void AddEntry(double x, double y);

	// Lookup value of line at x
	double Lookup(double x);

	// Minimum x
	double MinX();
	
	// Maximum x
	double MaxX();
	
	// Read in from a text file.
	// format is 
	// 1 2
	// 1.5 9
	// -1 5
	void LoadFromFile(const char *filename);
	
	// Clear out list.
	void Clear();

private:
	bool SortaEqual(double x1, double x2) { return (::fabs(x1 - x2) < 1e-6); }
	double m_below_min, m_above_max;
	std::vector< std::pair<double,double> > m_yvals;
};

#endif

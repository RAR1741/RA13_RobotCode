#include "Target.h"
#include <cmath>
#include <cstdio>

Target::Target()
	: m_x(0), m_y(0), m_center(false), m_distance(0), m_is_valid(false)
{
}

Target::~Target()
{
	
}

bool Target::Parse(std::string input) {
	try {
		float x,y,z,c;
		
		int result = ::sscanf(input.c_str(), "%f,%f,%f,%f,", &x, &y, &z, &c);
		if (result <= 0 || result == EOF)
		{
			m_is_valid = false;
		} else {
			m_x = x;
			m_y = y;
			m_distance = z;
			m_center = (::fabs(c - 1) < 1e-4 );
			m_is_valid = true;
		}
	} catch (std::exception ex) {
		m_is_valid = false;
	}
	
	return m_is_valid;
}

#ifndef TARGET_H__
#define TARGET_H__

#include <string>
class Target {
public:
	Target();
	~Target();
	
	bool Parse(std::string in);
	
	float X() const { return m_x; }
	float Y() const { return m_y; }
	float Distance() const { return m_distance; }
	
	bool IsCenter() const { return m_center; }
	
	bool IsValid() const { return m_is_valid; }
	bool IsInvalid() const { return !IsValid(); }
	
	// Mark target as invalid
	void Invalidate() { m_is_valid = false; }
private:
	float m_x;
	float m_y;
	bool m_center;
	float m_distance;
	bool m_is_valid;
};

#endif

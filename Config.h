#ifndef CONFIG_H__
#define CONFIG_H__
#include <string>
#include <map>

class Config {
public:
	static bool LoadFromFile(std::string filename);

	static double GetSetting(std::string name, double reasonable_default);
	
	static void Dump();
private:
	Config();
	
	static bool Parse(std::string filename);
	static void SetSetting(std::string name, double value);
	static bool HasKey(std::string name);
	static bool ValidKeyChar(char ch);
	static std::string ConvertToLower(std::string str);
	static std::map<std::string, double> m_settings;
};

#endif

#include "Config.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <fstream>

std::map<std::string, double> Config::m_settings;
Config::Config()
{

}

void Config::Dump()
{
	std::cout << "DUMP" << std::endl;
	for (
			std::map<std::string,double>::const_iterator itr = m_settings.begin();
			itr != m_settings.end();
			++itr)
	{
		std::cout << itr->first << ": " << itr->second << std::endl;
	}
	std::cout << "END DUMP" << std::endl;
}
bool Config::LoadFromFile(std::string filename)
{
	return Parse(filename);
}

double Config::GetSetting(std::string name, double reasonable_default)
{
	double retval = reasonable_default;
	name = ConvertToLower(name);
	
	if (HasKey(name)) {
		std::map<std::string,double>::const_iterator itr = m_settings.find(name);
		retval = (*itr).second;
	}

	return retval;
}

void Config::SetSetting(std::string name, double value)
{
	name = ConvertToLower(name);
	if (HasKey(name)) {
		std::map<std::string,double>::const_iterator itr = m_settings.find(name);
		// We can't get erase to work right now, so we'll use the first 
		// occurance
		//m_settings.erase(itr);
		m_settings[name] = value;
	} else
	{
		m_settings[name] = value;	
	}
}

std::string Config::ConvertToLower(std::string str)
{
	std::transform(str.begin(), str.end(), str.begin(), tolower);
	return str;
}

bool Config::HasKey(std::string name)
{
	name = ConvertToLower(name);
	return (m_settings.find(name) != m_settings.end());
}

inline bool Config::ValidKeyChar(char ch)
{
	return (isalpha(ch) || isdigit(ch) || (ch == '_'));
}

bool Config::Parse(std::string filename)
{
	std::ifstream infile; 
	
	infile.open(filename.c_str(), std::ios_base::in);

	if (!infile.is_open()) {
		std::cerr << "Could not open file!" << std::endl;
		return false;
	}

	std::string line = "";
	int line_number = 0;
	while (!infile.eof()) {
		++line_number;
		std::getline(infile,line);

		std::string name, value_str;
		int state = 0;
		bool done = false;
		bool invalid = false;
		
		// 
		// Process each line to eliminate comments.
		for (unsigned int i = 0; i < line.length() && !done; ++i)
		{
			switch(state) {
			case 0: // Ignoring initial whitespace,
				    // looking for either comment
				    // or valid key name
				//std::cout<<"Entering state 0 ......"<<std::endl;
				if (isspace(line[i])) {
					continue;
				} else if (line[i] == '#') {
					state = 3;
				} else {
					name += line[i];
					state = 1;
				}
				break;
			case 1:
				// Currently reading key name,
				// looking for equals sign
				//std::cout<<"Entering state 1 ......"<<std::endl;
				if (isspace(line[i])) {
					// Ignore for now
				} else if (line[i] == '=') {
					state = 2;
				} else if (! ValidKeyChar(line[i])) {
					state = 3;
					invalid = true;
				} else {
					name += line[i];
				}
				break;
			case 2:
				// Just read equals sign, reading 
				// value string
				//std::cout<<"Entering state 2 ......"<<std::endl;
				if (isspace(line[i])) {
					// Ignore!

				} else if (line[i] == '#') {
					state = 3;
				} else {
					value_str += line[i];
				}
				break;
			case 3:
				//std::cout<<"Entering state 3 ......"<<std::endl;
				done = true;
				continue; // Ignore anything after the 
				break;
			}
		}

		if (!invalid) {
			name = ConvertToLower(name);
			double val = ::atof(value_str.c_str());
			//std::cout<<"Name: "<<name<<std::endl<<"Value: "<<val<<std::endl;
			SetSetting(name, val);
		} else {
			std::cerr << "Invalid string on line " << line_number << ": " << line << std::endl;
		}
	}

	return true;
}




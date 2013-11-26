#ifndef TARGET_SERVER_H__
#define TARGET_SERVER_H__
#include "WPILib.h"
#include "Task.h"
#include "Target.h"
#include <semLib.h>
#include <string>


class TargetServer : public ErrorBase
{
public:
	TargetServer();
	~TargetServer();
		
	unsigned int Release();
	void Start();
	void Stop();
	
	std::string GetLatestPacket();
	Target GetLatestTarget();
	
	
private:
	Task m_serverTask;
	SEM_ID m_newTargetSem;
	bool m_stopServer;
	std::string m_latest_packet;
	
	// main worker 
	static int s_ServerTask(TargetServer *thisPtr);
	
	int ServerTask();
	int StartServerTask();
	
};
#endif

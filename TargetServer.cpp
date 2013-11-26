#include "TargetServer.h"
#include <errnoLib.h>
#include <hostLib.h>
#include <inetLib.h>
#include <sockLib.h>
#include "Task.h"
#include "Timer.h"
#include "WPIErrors.h"
#include <vxWorks.h>
#include <taskLibCommon.h>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <string>
#include "Synchronized.h"

using std::cout;
using std::endl;
using std::istringstream;
using std::string;

static const int BUFLEN = 1024;
static const int SERVER_PORT_NUM = 1130;

#define ACK "ACK"

TargetServer::TargetServer()
: m_serverTask("RATargetServer", (FUNCPTR)s_ServerTask)
	, m_newTargetSem (NULL)
	, m_stopServer (false)
	, m_latest_packet("")
{
	StartServerTask();
}


TargetServer::~TargetServer()
{
	Stop();
	ClearError();
}


int TargetServer::StartServerTask()
{
	cout << "Starting target server..." << endl;
	if (StatusIsFatal())
	{
		return -1;
	}
	int id = 0;
	m_stopServer = false;
	// Check for prior copy of running task
	int oldId = taskNameToId((char*)m_serverTask.GetName());
	if(oldId != ERROR)
	{
		// TODO: Report error. You are in a bad state.
		taskDelete(oldId);
	}

	// spawn target server task
	// this is done to ensure that the task is spawned with the
	// floating point context save parameter.
	bool started = m_serverTask.Start((int)this);

	id = m_serverTask.GetID();

	if (!started)
	{
		wpi_setWPIError(TaskError);
		return id;
	}
	taskDelay(1);
	cout << "Done: id " << id << endl;
	return id;
}


void TargetServer::Start()
{
	StartServerTask();
}

void TargetServer::Stop()
{
	m_stopServer = true;
}


int TargetServer::s_ServerTask(TargetServer *thisPtr)
{
	return thisPtr->ServerTask();
}

/**
 * @brief Most of the work done by the server. Get UDP packets containing
 * target string and save the latest one.
 */
int TargetServer::ServerTask()
{
    struct sockaddr_in  serverAddr;    /* server's socket address */ 
    struct sockaddr_in  clientAddr;    /* client's socket address */ 
    char                clientRequest[128]; /* request/Message from client */ 
    int                 sockAddrSize;  /* size of socket address structure */ 
    int                 sFd;           /* socket file descriptor */ 
//    char                inetAddr[INET_ADDR_LEN]; 
                                       /* buffer for client's inet addr */ 
 
    /* set up the local address */ 
 
    sockAddrSize = sizeof (struct sockaddr_in); 
    bzero ((char *) &serverAddr, sockAddrSize); 
    serverAddr.sin_len = (u_char) sockAddrSize; 
    serverAddr.sin_family = AF_INET; 
    serverAddr.sin_port = htons (SERVER_PORT_NUM); 
    serverAddr.sin_addr.s_addr = htonl (INADDR_ANY); 
 
    /* create a UDP-based socket */ 
    printf("Socket call...");
    if ((sFd = socket (AF_INET, SOCK_DGRAM, 0)) == ERROR) 
        { 
        perror ("socket"); 
        return (ERROR); 
        } 
    printf("Success!\n");
    /* bind socket to local address */ 
    printf("Binding...");
    if (bind (sFd, (struct sockaddr *) &serverAddr, sockAddrSize) == ERROR) 
        { 
        perror ("bind"); 
        close (sFd); 
        return (ERROR); 
        } 
    int smallRcvBufferSize = 256; // two d-grams
    // hopefully this works
    setsockopt(sFd, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<char*>(& smallRcvBufferSize), sizeof(smallRcvBufferSize));
    printf("Done!\n");
    /* read data from a socket and satisfy requests */
    
    /* read data from a socket and satisfy requests */ 
     
        FOREVER 
            { 
    		for (unsigned int i = 0; i < sizeof(clientRequest); ++i) {
    			clientRequest[i] = 0;
    		}
    	//printf("Waiting for a packet...\n");
            if (recvfrom (sFd, (char *) &clientRequest, sizeof (clientRequest), 0, 
                (struct sockaddr *) &clientAddr, &sockAddrSize) == ERROR) 
            { 
                perror ("recvfrom"); 
                close (sFd); 
                return (ERROR); 
            }
//            cout << clientRequest << endl;
            std::string temp(clientRequest);
            // Leave me alone for a while while I copy this here string.
            /*if (semTake(m_newTargetSem, 200) != OK) {
            	continue; // Try again later.
            }*/
            m_latest_packet = temp;
            taskDelay(100);
            //semGive(m_newTargetSem);
            // Done.
 
            } 
}
Target TargetServer::GetLatestTarget() {
	Target t;
	t.Parse(this->GetLatestPacket());
	return t;
}

std::string TargetServer::GetLatestPacket() {
	std::string temp;
	// Using the target semaphore to regulate access
	// to the m_latest_packet string. Shared memory, man.
	/*if (semTake(m_newTargetSem, 200) != OK) {
		// still waiting for server.
		cout << "WARNING: semTake did ont finish, something's up with the TargetServer" << endl;
		return "";
	}*/
	temp = m_latest_packet;
	//semGive(m_newTargetSem);
	
	return temp;
}

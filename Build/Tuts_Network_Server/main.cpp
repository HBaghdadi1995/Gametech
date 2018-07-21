
/******************************************************************************
Class: Net1_Client
Implements:
Author: Pieran Marris <p.marris@newcastle.ac.uk> and YOU!
Description:

:README:
- In order to run this demo, we also need to run "Tuts_Network_Client" at the same time.
- To do this:-
1. right click on the entire solution (top of the solution exporerer) and go to properties
2. Go to Common Properties -> Statup Project
3. Select Multiple Statup Projects
4. Select 'Start with Debugging' for both "Tuts_Network_Client" and "Tuts_Network_Server"

- Now when you run the program it will build and run both projects at the same time. =]
- You can also optionally spawn more instances by right clicking on the specific project
and going to Debug->Start New Instance.




FOR MORE NETWORKING INFORMATION SEE "Tuts_Network_Client -> Net1_Client.h"



(\_/)
( '_')
/""""""""""""\=========     -----D
/"""""""""""""""""""""""\
....\_@____@____@____@____@_/

*//////////////////////////////////////////////////////////////////////////////

#pragma once

#include <enet\enet.h>
#include <nclgl\GameTimer.h>
#include <nclgl\Vector3.h>
#include <nclgl\common.h>
#include <ncltech\NetworkBase.h>
#include "PathFinder.h"

//Needed to get computer adapter IPv4 addresses via windows
#include <iphlpapi.h>
#pragma comment(lib, "IPHLPAPI.lib")


#define SERVER_PORT 1234
#define UPDATE_TIMESTEP (1.0f / 30.0f) //send 30 position updates per second

NetworkBase server;
GameTimer timer;
float accum_time = 0.0f;
float rotation = 0.0f;

int m_size = 16;
float density = 0.5f;

void Win32_PrintAllAdapterIPAddresses();

PathFinder *p = new PathFinder("Maze");

int onExit(int exitcode)
{
	server.Release();
	system("pause");
	exit(exitcode);
}

void SendMaze() {
	string m = p->GeneratePacket();
	const char* c = m.c_str();
	ENetPacket* position_update = enet_packet_create(c, strlen(c), 0);
	enet_host_broadcast(server.m_pNetwork, 0, position_update);
}

void newMaze(int i, float f) {
	p->GenerateNewMaze(i, f);
	SendMaze();
}

void SendPath(ENetPeer *e) {
	string m = p->GetPathString();
	const char* c = m.c_str();

	ENetPacket* packet = enet_packet_create(c, strlen(c) + 1, 0);
	enet_peer_send(e, 0, packet);
}

void SendAvatars() {
	//if (p->avatars.size() != 0) {
		string m = p->avatarPacket + "@@";
		const char* c = m.c_str();
		ENetPacket* position_update = enet_packet_create(c, strlen(c), 0);
		enet_host_broadcast(server.m_pNetwork, 0, position_update);
	//}
}

int main(int arcg, char** argv)
{
	if (enet_initialize() != 0)
	{
		fprintf(stderr, "An error occurred while initializing ENet.\n");
		return EXIT_FAILURE;
	}

	//Initialize Server on Port 1234, with a possible 32 clients connected at any time
	if (!server.Initialize(SERVER_PORT, 32))
	{
		fprintf(stderr, "An error occurred while trying to create an ENet server host.\n");
		onExit(EXIT_FAILURE);
	}

	printf("Server Initiated\n");


	Win32_PrintAllAdapterIPAddresses();

	newMaze(16,0.5);

	timer.GetTimedMS();
	while (true)
	{
		float dt = timer.GetTimedMS() * 0.001f;
		accum_time += dt;

		//Handle All Incoming Packets and Send any enqued packets
		
		//p->onUpdate();

		server.ServiceNetwork(dt, [&](const ENetEvent& evnt)
		{
			switch (evnt.type)
			{
			case ENET_EVENT_TYPE_CONNECT:
				printf("- New Client Connected\n");
				SendMaze();
				break;

			case ENET_EVENT_TYPE_RECEIVE:
				printf("\t Client %d says: %s\n", evnt.peer->incomingPeerID, evnt.packet->data);
				
				if (evnt.packet->data[0] == '@')
				{
					string s = "";
					for (int i = 0; i < 5; i++) {
						s += evnt.packet->data[i+1];
					}
					
					if (s.compare("reset") == 0) {
						newMaze(m_size,density);
					}
					if (s.compare("coord") == 0) {
						s = (char*)evnt.packet->data;
						p->CalculatePath(s);
						SendPath(evnt.peer);
					}
					if (s.compare("paths") == 0) {
						s = (char*)evnt.packet->data;
						p->AddAvatar(s);
					}
					if (s.compare("sizes") == 0) {
						s = (char*)evnt.packet->data;
						string integ = "";
						for (int i = 6; s.at(i) != '@'; i++) {
							integ += s.at(i);
						}
						m_size = stoi(integ);
						newMaze(m_size, density);
					}
				}
				
				enet_packet_destroy(evnt.packet);
				break;

			case ENET_EVENT_TYPE_DISCONNECT:
				printf("- Client %d has disconnected.\n", evnt.peer->incomingPeerID);
				break;
			}
		});

		//Broadcast update packet to all connected clients at a rate of UPDATE_TIMESTEP updates per second
		if (accum_time >= UPDATE_TIMESTEP)
		{
			p->onUpdate(accum_time);
			SendAvatars();
			accum_time = 0;
		}

		Sleep(0);
	}

	system("pause");
	server.Release();
}




//Yay Win32 code >.>
//  - Grabs a list of all network adapters on the computer and prints out all IPv4 addresses associated with them.
void Win32_PrintAllAdapterIPAddresses()
{
	//Initially allocate 5KB of memory to store all adapter info
	ULONG outBufLen = 5000;


	IP_ADAPTER_INFO* pAdapters = NULL;
	DWORD status = ERROR_BUFFER_OVERFLOW;

	//Keep attempting to fit all adapter info inside our buffer, allocating more memory if needed
	// Note: Will exit after 5 failed attempts, or not enough memory. Lets pray it never comes to this!
	for (int i = 0; i < 5 && (status == ERROR_BUFFER_OVERFLOW); i++)
	{
		pAdapters = (IP_ADAPTER_INFO *)malloc(outBufLen);
		if (pAdapters != NULL) {

			//Get Network Adapter Info
			status = GetAdaptersInfo(pAdapters, &outBufLen);

			// Increase memory pool if needed
			if (status == ERROR_BUFFER_OVERFLOW) {
				free(pAdapters);
				pAdapters = NULL;
			}
			else {
				break;
			}
		}
	}


	if (pAdapters != NULL)
	{
		//Iterate through all Network Adapters, and print all IPv4 addresses associated with them to the console
		// - Adapters here are stored as a linked list termenated with a NULL next-pointer
		IP_ADAPTER_INFO* cAdapter = &pAdapters[0];
		while (cAdapter != NULL)
		{
			IP_ADDR_STRING* cIpAddress = &cAdapter->IpAddressList;
			while (cIpAddress != NULL)
			{
				printf("\t - Listening for connections on %s:%u\n", cIpAddress->IpAddress.String, SERVER_PORT);
				cIpAddress = cIpAddress->Next;
			}
			cAdapter = cAdapter->Next;
		}

		free(pAdapters);
	}

}
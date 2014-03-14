
/* *** l2ptelesc_example.c ***
 * 
 * Author: Matthew Wilkinson. 
 * Institute: Space Geodesy Facility, Herstmonceux UK.
 * Research Council: British Geological Survey, Natural Environment Research Council.
 * 
 * Version: 1.0
 * Last Modified: 5th March 2014
 * 
 * Please visit http://www.bgs.ac.uk/downloads/softdisc.html for our disclaimer.
 * 
 * The listen2planes telescope example client connects to the running l2pserver using the TCP IP address and
 * the port 2020. It sends regular azimuth and elevation updates for the telescope and laser direction.
 * 
 * These positions are made available to other application connections to the server.  The l2pclient uses
 * this information to provide warnings and a laser inhibit signal.
 * 
 * The user must integrate this example code in to their system to provide the required telescope information.
 * 
 */



#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <limits.h>
#include <time.h>
#include <fcntl.h>


 
int make_l2psocket ()
{
    int c;
    int sock;
    char srvIP[15];
    struct hostent *host;
    struct sockaddr_in server_addr;  
    
    /* IP address of PC running l2pserver */
    sprintf(srvIP,"193.61.194.29");
    
    host = gethostbyname(srvIP);

    if ((sock = socket(PF_INET, SOCK_STREAM, 0)) == -1) 
    {
	printf("l2pclient: Socket fail\n");
// 	exit(1);
	sock=-1;
    }
    else
    {
	memset(&server_addr, -1, sizeof(server_addr));
	server_addr.sin_family = PF_INET;     
	server_addr.sin_port = htons(2020);   
	server_addr.sin_addr = *((struct in_addr *)host->h_addr);
	bzero(&(server_addr.sin_zero),8); 
	
	c=connect(sock, (struct sockaddr *)&server_addr,sizeof(struct sockaddr));
	if (c == -1) 
	{
		printf("l2pclient: Connection to server fail\n");
	// 	exit(1);
		sock=-1;
	}
     }
  return sock;
}

// 		  ...
// 		  ...
// 		  ...
// 		  ...
// 		  ...
// 		  ...

int main (void)
{
  int sockl2p;
  int len;
  int oldflags;
  char l2pSND[256];
  char l2pRECV[256];
  float azim, elev;
  char azims[7], elevs[6];
  
// 		  ...
// 		  ...
// 		  ...
// 		  ...
// 		  ...
// 		  ...
// 		  ...
// 		  ...
  
  /* Create the socket to the l2p server */
  sockl2p = make_l2psocket ();
  
   // set non blocking socket
//   oldflags = fcntl (sockl2p, F_GETFL, 0);
//   oldflags |= O_NONBLOCK;
//   fcntl (sockl2p, F_SETFL, oldflags);
  
  if (sockl2p == -1)
    printf("l2pclient: listen2planes not in operation\n");
  else 
    printf("l2pclient: listen2planes socket() established on %d\n",sockl2p);
    
  //     		  ...
  // 		  ...
  // 		  ...
  // 		  ...
  // 		  ...
  // 		  ...
    
      sprintf(l2pSND,"telscp: \0");
      len = send(sockl2p, l2pSND, strlen(l2pSND)+1, 0);
  while(1)
  {
  //  		  ...
  // 		  ...
  // 		  ...
  // 		  ...
  // 		  ...
    
    sprintf(azims,"220.0");
    sprintf(elevs,"20.0");

    if (sockl2p > 0)
    {
//       len = read(sockl2p, l2pRECV, 256);
      len = recv(sockl2p, l2pRECV, sizeof(l2pRECV),0);

      sscanf(azims,"%f",&azim);
      sscanf(elevs,"%f",&elev);
      sprintf(l2pSND,"telscp:  %6.2f %5.2f\0",azim,elev);
      len = send(sockl2p, l2pSND, strlen(l2pSND)+1, 0);
      if (len < 0)
      {
	printf("l2pclient: socket send error, closing...\n");
	close(sockl2p);
	sockl2p=-1;
	len=0;
	exit(1);
      }

      printf("%s\n",l2pSND);
    }

  // 		  ...
  // 		  ...
  // 		  ...
  // 		  ...
  // 		  ...
  // 		  ...
	
	      usleep(3000);				 /* control send fequency*/
  }
  
  close(sockl2p);
  
}
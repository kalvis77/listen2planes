
/*******l2pserver.c ***
 * 
 * Author: Matthew Wilkinson.
 * Institute: Space Geodesy Facility, Herstmonceux UK.
 * Research Council: British Geological Survey, Natural Environment Research Council.
 * 
 * Version: 1.0
 * Last Modified: 5th March 2014
 * 
 * Please visit here to read our disclaimer: http://www.bgs.ac.uk/downloads/softdisc.html
 * 
 * 
 * The listen2planes server makes a permanent connection
 * to the SBS-3 receiver IP address.  It receives the raw binary
 * messages and selects the ADS-B information which it translates
 * to get plane identification, position components and velocity
 * components. It then calculates the azimuth elevation and range
 * for the given station coordinates, which must be input accordingly.
 * The resulting positions and velocities are made available on a TCP/IP
 * with the IP address of the PC running the program and port 2020.  The
 * server is able to accept multiple connections. 
 * 
 *	 The Linux compile command is:
 *	  	gcc -lm -o l2pserver l2pserver.c
 *
 *	 To run the server type:
 * 		l2pserver <IP of data stream>  <port>  <elev cut off>
 * 
 * The IP address and port number of the SBS-3 must be set using the
 * BaseStation software.  The l2pserver also works with the alternative
 * raw data stream provided by BaseStation.
 * 
*/
  
  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>
  #include <unistd.h>
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <time.h>  
  #include <errno.h>   
  #include <netdb.h> 
  #include <sys/ioctl.h>
  #include <math.h>
  #include <fcntl.h>
  
  #define PORT 2020			 		/* port for server */
  #define BUFFER_SIZE 256
  #define TRUE  1
  #define FALSE 0
  
  
  int NLtablelookup(double);	 			/* function to determine latitude zone */
  char * flightnum_character(char[]);	 		/* function to acquire flight number character */

  int main(int argc, char *argv[])
  {
    
    /* tcpip server variables  */
    int rc;						/* Used as output int */
    int on = 1;						/* setsocket() variable */
    int len;						/* len of buffer in bytes */
    int seln;						/* Select() server connection */
    struct timeval timeout;   				/* Timeout value */
    int fd;						/* output of check on file descriptor */
    int sockfd = 0;					/* return from socket() */
    int newfd;						/* newly accept()ed socket descriptor */
    int fdmax;						/* maximum file descriptor number in use*/
    fd_set master;					/* master file descriptor list */
    fd_set read_fds;					/* temp file descriptor list for select() */
    struct sockaddr_in serveraddr;			/* server address */
    struct sockaddr_in clientaddr;			/* client address */
    int addrlen;					/* client address length */
    int listener;					/* listening socket descriptor */
    int portn;						/* port for connection to data stream */
    char SNDbuf[BUFFER_SIZE];				/* buffer for client data send*/
    char RECVbuf[BUFFER_SIZE];				/* buffer for client data receive*/
    char buf[100];                                      /* buffer to read received message from connections */
    int desc_ready;					/* read file descriptor */
    int cx;	 					/* connection index */
    char stype[2];					/* type of l2p socket connection */
    char cxtype[100][2];				/* connection type in/out */
    int close_conn;					/* flag to close socket connection */
    
    
    /* l2p ADS-B message read variables  */
    unsigned char sbsRECV[1000];			/* Hex buffer received from SBS-3 tcpip */
    unsigned char prevBuff[1000];			/* Previous Hex buffer received from SBS-3 tcpip */
    unsigned char oversize[1000];			/* Large buffer containing multiple ADS-B messages */
    int nbytes = 0;					/* bytes in buffer */
    int totnbytes;					/* total number of bytes in buffer */
    int prev_nb;					/* Previous number of bytes in buffer */
    char bit1[5],bit2[5];				/* 4-bit string for 1st and 2nd hex characters */
    char rB[1000][3];					/* Hex buffer separated by bytes*/
    int ADS_B[300];					/* ADS-B binary message */
    int squitter[300];					/* Squitter binary message */
    int qbit;						/* qbit in ADS-B message */
    int tsbi[100];					/* time stamp binary */
    int altbi[100],latbi[100],longbi[100];		/* position elements binary */
    int TC[5];						/* Time code binary */
    int cpr[10000]; 					/* ODD EVEN CPR flag */
    int Tbit;						/* Time flag */
    char fnc[8][7];					/* flight number in 8 x 6bits */
    char * fli;						/* return character from flightnumber() */
    char flightnumber[1000][9];				/* flightnumbers of aircraft */
    int DFbi[5];					/* Data format type binary */
    float DF;						/* Data format type binary */
    int CAbi[3];					/* Capability number binary */
    float CA;						/* Capability number */
    float Tcode[10000];					/* Time code in message */
    double altitude[10000];				/* Broadcast altitude */
    double tstamp[10000];				/* 'rolling timestamp' from message (not used) */
    double latr[10000];					/* latitude record */				
    double latref;					/* latitude reference in index calc */
    double latrec[10000];				/* latitude in ADS-B message */
    double latindex;					/* latitude index */
    double latitude;					/* latitude result */
    double lat0,lat1;					/* latitude even and odd message */
    double rlat0,rlat1;					/* latitude even and odd real records */
    double dlat0,dlat1;					/* odd and even variables for calculation */
    double longr[10000];				/* longitude record */				
    double lonref;					/* longitude reference in index calc */		
    double flonref;					/* longitude corrected to 0-360 */
    double longindex;					/* longitude index */
    double longrec[10000];				/* longitude record in ADS-B message */				
    double longitude;					/* longitude result */
    double lon0,lon1;					/* longitude even and odd message */
    double rlon;					/* real longitude record */
    double dlon;					/* variable dlon depending on ni */
    int ni;						/* variable depending on number of lat zones */
    int NL0,NL1;					/* output from latitude zones lookup NLtablelookup() */
    char tstring[1000][11];				/* output time string */
    int binaryout[10000];				/* ADS-B message in binary form */
    char ICAO[1000][7];					/* Aircraft Hex ID */
    char oI[1000][7];					/* Aircraft Hex ID array */
    double oalt[1000];					/* altitude output array */
    double olat[1000];					/* latitude output array */
    double olong[1000];					/* longitude output array */
    double oAz[1000],oEl[1000];				/* azimuth and elevation output arrays */
    double oep[1000],oRng[1000];			/* epoch and range output arrays */
    char parity[7];					/* Check parity at end of ADS-B message */
    char pair[2],opair[1000][2];			/* Method by which position is calculated, pair or singular */
    int matchn;						/* Entry index of odd/even match */
    int pln, opln;					/* New plane entry index and previous*/
    int numl;						/* Number of plane entries */
    float Elcutoff;					/* Elevation Cutoff for plane detection in degrees */


    /* Az/El coordinate system translation */
    double Elevation, Azimuth, Rng;			/* Az/El coordinates for Station */
    double ecc;						/* Eccentricity of shape Earth */
    double smjr, smnr;					/* semi-major and semi-minor Earth */
    double STAT_X, STAT_Y, STAT_Z;			/* Station X,Y,Z coordinates */
    double X, Y, Z, Nradi,v[3],w[3];			/* Aircraft X,Y,Z coordinates */
    float latrad,longrad;				/* Aircraft latitude and longitude in radians */
    double STAT_lat, STAT_long, STAT_alt;			/* Station Lat, Lon, Alt coordinates */
    double STAT_latr, STAT_longr;				/* Station Lat, Lon in radians */
    double EWvel[1000];					/* East-West velocity */
    double NSvel[1000];					/* North-South velocity */
    double Altvel[1000];				/* Altitude velocity */
    double vEW,vNS,vAlt;				/* Velocity component magnitudes in message */
    int EWbin[10],NSbin[10],Altbin[9];			/* Binary velocity components */
    int EW;						/* 0 = East, 1 = West velocity direction */
    int NS;						/* 0 = North, 1 =  South velocity direction */
    int Alt;						/* 0 = Up, 1 = Down velocity direction */
    float subtype;					/* 1 = Normal aircraft, 2 = supersonic */
    float emcat;					/* Aircraft emitter category */
    
    
    double pcseconds;					/* microseconds from PC clock */
    float jd,hr,min,sec;				/* Values calculated from PC clock */
    double fod;						/* fraction of day used in calculation */
    double tod[10000];					/* time seconds of day */
    float Tazim, Telev;					/* Telescope pointing angles */
    int firel,fireo;					/* System fire flag received from l2pclient */
    double tep;				
    double telesc_tickep;				/* interval between telescope message sends */
    int m,n,a,c,e,i,j,k,q;
    double pi=3.1415926535898;				/* PI constant */
    const char* b[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8","9",
                        "a", "b", "c", "d", "e", "f"};
    const char* quads[] = {"0000", "0001", "0010", "0011", "0100", "0101",
                     "0110", "0111", "1000", "1001", "1010", "1011",
                     "1100", "1101", "1110", "1111"};
     
    struct sockaddr_in sbs_addr; 			/* IP address for SBS-3 */
    struct timeval tv;
    
    
     
    for (cx=0;cx<100;cx++)
	sprintf(cxtype[cx]," \0");               	/* clear connection type */
        
    /************************/ 
    /* SETUP tcpip server   */
    /************************/ 
    
    /* Create an PF_INET stream socket  */
    if((listener = socket(PF_INET, SOCK_STREAM, 0)) == -1)	/* get the listener */
    {
      perror("l2pserver-socket() error");		
      exit(1);
    }
      
    if(setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) == -1)
    {
      perror("l2pserver-setsockopt() error");
      close(listener);
      exit(1);
    }

    /* Set socket to be non-blocking. */
    rc = ioctl(listener, FIONBIO, (char *)&on);
    if (rc < 0)
    {
	perror("ioctl() failed");
	close(listener);
	exit(-1);
    }
    
    /* Bind the socket  */
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = PF_INET;
    serveraddr.sin_addr.s_addr = INADDR_ANY;
    serveraddr.sin_port = htons(PORT);

    if(bind(listener, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) == -1)
    {
      perror("l2pserver-bind() error\n");
      close(listener);
      exit(1);
    }
    
    /* listen */
    if(listen(listener, 50) == -1)
    {
      perror("l2pserver-listen() error\n");
      close(listener);
      exit(1);
    }

    FD_ZERO(&master);					/* clear the master and temp sets */
    fdmax = listener; 					/* keep track of the biggest file descriptor */
    FD_SET(listener, &master);				/* add the listener to the master set */
      
    timeout.tv_sec = 0;
    timeout.tv_usec = 6000;				/* set timeout value for select() */

    memset(RECVbuf, -1,sizeof(RECVbuf));		/* clear client receive buffer */
    memset(SNDbuf, -1,sizeof(SNDbuf));			/* clear client send buffer */
    
    memset(sbsRECV, -1,sizeof(sbsRECV));		/* clear SBS-3 receive buffer */
    
    
    
        
    /************************/ 
    /* SETUP listen2planes  */
    /************************/ 
    
    /* Initialise variables */	
    m=0;    						/* number of current broadcasts */
    numl=0;
    sprintf(oI[0]," ");
    prev_nb=0;
    Tazim=0.0;
    Telev=0.0;
    sprintf(SNDbuf,"#"); 
    sprintf(stype,"#\0");
    pln=-1;
    opln=0;
    telesc_tickep=-1.0;
    tep=0.0;
    firel=2;
        
    /* Set station coordinates for centre of Az/El variables */
    STAT_lat=50.8674;
    STAT_long=0.3361;
    STAT_alt=75.0;
    STAT_latr=2.0*pi*STAT_lat/360.0;  			/* to radians */
    STAT_longr=2.0*pi*STAT_long/360.0;    
    
    /* Set equivalent XYZ coordinates for station */
    STAT_X=4033463.1;
    STAT_Y=23662.8;
    STAT_Z=4924305.6;
    
    dlat0=6.0;
    dlat1=360.0/59.0;
     
    /* Time and date from PC */
    gettimeofday(&tv, NULL);
    pcseconds = (tv.tv_sec)  + (tv.tv_usec) / 1e6 ; 
    jd=floor(2440587+pcseconds/86400.0) ;		/* Julian day. 2440587 is days before 1st Jan 1970 */
    fod=(2440587+pcseconds/86400.0)-1.0*jd;
    tod[m]=86400.0*fod;
    hr=floor(24.0*fod);
    min=floor(1440.0*(fod-(hr/24.0)));
    sec=86400.0*(fod-(hr/24.0)-(min/1440.0));
    sec=floor(sec);
    jd=jd-2400000.0;					/* Modified Julian Day */
    
  
  
    /* Check l2pserver command line variables */
    if(argc == 4)
    {
      printf ("%.0f hr  %.0f min  %2.0f sec\n", hr,min,sec);
      sscanf(argv[2],"%d",&portn);
      sscanf(argv[3],"%f",&Elcutoff);
    }
    else if(argc == 3)
    {
      sscanf(argv[2],"%d",&portn);
      Elcutoff=5.0;      				/* default elevation cut off */
    }
    else
    {
        printf("\n    Usage: %s <ip of server>  <port>  <elev cut off>\n",argv[0]);
        printf("\n    SBS-3: 193.61.194.54 10001      PC: 193.61.194.32 30006\n\n");
        return 1;
    } 

	
       
    /* Make connection to SBS-3 */
    if((sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n SBS Error : Error : Could not create socket \n");
        return 1;
    } 
  
    memset(&sbs_addr, -1, sizeof(sbs_addr));
    sbs_addr.sin_family = PF_INET;
    sbs_addr.sin_port = htons(portn);   		/* set port number from TCPIP connection */

    if(inet_pton(PF_INET, argv[1], &sbs_addr.sin_addr)<=0)
    {
        printf("\n SBS Error : inet_pton error occured\n");
        return 1;
    } 

    if( connect(sockfd, (struct sockaddr *)&sbs_addr, sizeof(sbs_addr)) < 0)
    {
       printf("\n SBS Error : Connect Failed \n");
       return 1;
    } 
      
    
    /*** Start of loop ***.  
     * The connection to the SBS-3 was succesful and the raw data 
     * tcpip socket is repeatedly read and once an ADS-B message is
     * received it is translated,stored and the results are made
     * available through the l2p server.
     ***/
    while((nbytes = read(sockfd, sbsRECV, sizeof(sbsRECV)-1)) > 0)
    {	  
	memcpy(&read_fds, &master, sizeof(master));	
      	  
	
    /* Occasionally more than one message is received
     * on a single tcpip line. Read the total number of
     * bytes received and deal with one message at a time.*/
	totnbytes=nbytes;
	if(sbsRECV[0] == 0x10 && nbytes <5)
	{
	  for (i=0;i<nbytes;i++)
	    prevBuff[i]=sbsRECV[i];
	  
	  totnbytes = 0; 
	}
	else if(nbytes > 15 && prev_nb <5)
	{	 	
	  for (i=0;i<=nbytes;i++)
	    sbsRECV[nbytes-i+prev_nb]=sbsRECV[nbytes-i];
		      
	  for (i=0;i<prev_nb;i++)
	    sbsRECV[i]=prevBuff[i];
	  
	  nbytes = nbytes+prev_nb;
	  totnbytes = nbytes;
	}
	prev_nb=nbytes;
	

    /* All 0x10 0x10 pairs should be replaced by a single 0x10 entry */
	j=0;
	for (i=0;i<totnbytes;i++)
	{	
	  oversize[i]=0x99;
	  oversize[j]=sbsRECV[i];
	  if(sbsRECV[i] != 0x10 || sbsRECV[i+1] != 0x10)
	    j=j+1;
	}
	totnbytes=j;
	
	
    /* Put messages in to a buffer in turn from oversized buffer */
	q=2;
	while(q <= totnbytes)
	{
	    i=0;
	    while((oversize[q+i] != 0x10 || oversize[q+i+1] != 0x02) && q+i < totnbytes)
	      {
		sbsRECV[i]=oversize[q+i];
		i=i+1; 
	      }
	    nbytes=i; 
	    q=q+nbytes+2;
	    
	    sprintf(SNDbuf,"#"); 
	    pln=-1;
	    
      
    /* Select only packets beginning with byte 0x01 */
	    if(sbsRECV[0] == 0x01)// || sbsRECV[0] == 0x05)
	    {
	      if(nbytes > 20)
	      {
		memset(binaryout, 0, sizeof(binaryout));
		for (i=0;i<nbytes;i++)
		  {	
		    sprintf(rB[i],"%02x", sbsRECV[i]);   // read message as %x HEX
		    
		    
    /* Convert HEX to binary by substituting 4bit strings for HEX characters*/
		    for (j=0;j<16;j++)       
		    { 
		      if(rB[i][0] == *b[j])
			sprintf(bit1,"%s", quads[j]);
		      if(rB[i][1] == *b[j])
			sprintf(bit2,"%s", quads[j]);
		    }
		  
		    sscanf(bit1,"%1d%1d%1d%1d",&binaryout[i*8+0],&binaryout[i*8+1],&binaryout[i*8+2],&binaryout[i*8+3]);
		    sscanf(bit2,"%1d%1d%1d%1d",&binaryout[i*8+4],&binaryout[i*8+5],&binaryout[i*8+6],&binaryout[i*8+7]);	    
		  }  
	    
	      sprintf(ICAO[m],"%2s%2s%2s",rB[6],rB[7],rB[8]);    // HEX plane id
	      ICAO[m][6]='\0';
	      
	      gettimeofday(&tv, NULL);
	      pcseconds = (tv.tv_sec)  + (tv.tv_usec) / 1e6 ; 

	      jd=floor(2440587+pcseconds/86400.0);
	      fod=(2440587+pcseconds/86400.0)-1.0*jd;
	      tod[m]=86400.0*fod;              // epoch of message
	      hr=floor(24.0*fod);
	      min=floor(1440.0*(fod-(hr/24.0)));
	      sec=86400.0*(fod-(hr/24.0)-(min/1440.0));
	      jd=jd-2400000.0;
	
	      
    /* calculate 'rolling timestamp' from message (not used) */
	      k=0;
	      tstamp[m]=0.0;     
	      for(j=16;j<40;j++)
		  {
		    tsbi[k]=binaryout[j];
		    tstamp[m]=tstamp[m]+tsbi[k]*pow(2.0,23.0-1.0*k);
		    k=k+1;
		  }
	      tstamp[m]=tstamp[m]/20.0e6;
		    
		  
    /* extract the ADS-B message string */
	      k=0;                       
	      for(j=40;j<224;j++)
		  {
		    ADS_B[k]=binaryout[j];
		    k=k+1;
		  } 
	      
	      
    /* Read Downlink format type */
	      k=0;
	      DF=0.0;
	      for(j=0;j<5;j++)
		  {
		    DFbi[k]=ADS_B[j];
		    DF=DF+DFbi[k]*pow(2.0,4.0-1.0*k);
		    k=k+1;
		  }
  
  
    /* Read Capability number */
	      k=0;
	      CA=0.0;
	      for(j=5;j<8;j++)
		  {
		    CAbi[k]=ADS_B[j];
		    CA=CA+CAbi[k]*pow(2.0,2.0-1.0*k);
		    k=k+1;
		  }  
		    
		    
    /* Extract squitter string */
	      k=0;              
	      for(j=32;j<(nbytes-5)*8;j++)
		  {
		    squitter[k]=ADS_B[j]; 
		    k=k+1;
		  } 
			
			
    /* Read type code from squitter */
	      k=0;
	      Tcode[m]=0.0;
	      for(j=0;j<5;j++)
		  {
		    TC[k]=squitter[j];
		    Tcode[m]=Tcode[m]+TC[k]*pow(2.0,4.0-1.0*k);
		      
		    k=k+1;
		  } 
	      Tbit=squitter[20];    
	      
	      
    /* Read parity from squitter */
	      sprintf(parity,"%02x%02x%02x\0",sbsRECV[16], sbsRECV[17], sbsRECV[18]); 
	      
	      
    /* Select Downlink Format 17 (ADS-B) and 18 (TIS-B) */
	      if((DF == 17.0 || DF == 18.0) && strcmp(parity,"000000\0") == 0)
		{ 
		  
    /* Select type codes 9-22, excluding 19 */
		  if(Tcode[m] >= 9.0 && Tcode[m] <= 22.0 && Tcode[m] != 19.0 )
		    { 		      
		      k=0;
		      a=0;
		      c=0;
		      altitude[m]=0.0;
		      
		      
    /* Read the 8th altitude bit as the Q-bit,
     * remove from squitter and calculate altitude
     * in feet with remaining bits */
		      qbit=squitter[15];
		      for(j=8;j<20;j++)
			{
			  if(j != 15)
			    {
			      altbi[k]=squitter[j]; 
			      altitude[m]=altitude[m]+altbi[k]*pow(2.0,10.0-1.0*k);
			      k=k+1;
			    }
			}	
			
    /* Calculate altitude in feet depending on q-bit */
		      if(qbit == 0)
			altitude[m]=100.0*altitude[m];
		      if(qbit == 1)
			altitude[m]=25.0*altitude[m]-1000.0;
			 
		      
    /* Read CPR code as odd or even message */
		      cpr[m]=squitter[21]; 

		      
    /* Read the latitude index entry */
		      k=0;
		      latrec[m]=0.0;
		      for(j=22;j<39;j++)    // extract latitude bits
			{
			    latbi[k]=squitter[j]; 
			    latrec[m]=latrec[m]+latbi[k]*pow(2.0,16.0-1.0*k);
			    k=k+1;
			} 
		
		
    /* Read the longitude index entry */
		      k=0;
		      longrec[m]=0.0;
		      for(j=39;j<56;j++)    // extract longitude bits
			  {
			      longbi[k]=squitter[j];
			      longrec[m]=longrec[m]+longbi[k]*pow(2.0,16.0-1.0*k);
			      k=k+1;
			  }      
		      rlat0=0.0;
		      rlat1=0.0;
		      rlon=0.0;
		      lat0=0.0;
		      lat1=0.0;
		      lon0=0.0;
		      lon1=0.0;
		      NL0=-1;
		      NL1=0;
		      if(cpr[m] == 0)     // is latest message odd or even?
			  {
			    lat0=latrec[m];
			    lon0=longrec[m];
			  }
		      if(cpr[m] == 1)
			  {
			    lat1=latrec[m];
			    lon1=longrec[m];
			  }
					    
			  
    /* Add data to arrays and refresh array so
     * that all data is less than 10 seconds old */
		      e=0;
		      for(n=0;n<=m;n++)
			  {
			    if((tod[m]-tod[n] <= 10.0) && (tod[m] >= tod[n]))
			    {
			      sprintf(ICAO[e],"%s",&ICAO[n]);
			      tod[e]=tod[n];
			      tstamp[e]=tstamp[n];
			      altitude[e]=altitude[n];
			      latrec[e]=latrec[n];
			      longrec[e]=longrec[n];
			      cpr[e]=cpr[n];
			      Tcode[e]=Tcode[n]; 
			      e=e+1;	 
			    }
			  }
		      m=e-1;
		
		      
    /* Retrieve Odd or Even pair from buffer */
		      for(n=0;n<m;n++)
			  {
			    if(strcmp(ICAO[n],ICAO[m]) == 0)   //equal
			    {
			      if(cpr[n] ==1 && cpr[m] ==0)
			      {
				matchn=n;
				lat1=latrec[matchn];
				lon1=longrec[matchn];
			      }
			      if(cpr[n] ==0 && cpr[m] ==1)
			      {
				matchn=n;
				lat0=latrec[matchn];
				lon0=longrec[matchn];
			      }
			    }
			  } 
			  
		      latitude=0.0;
		      longitude=0.0;
		      latref=0.0;
		      lonref=0.0;
		      
		      
    /* Retrieve reference long/lat for local decoding */
		      for(k=0;k<numl;k++)
			  {
			    if(strcmp(ICAO[m],oI[k])== 0)
			    {
			      latref=latr[k];
			      lonref=longr[k];
			    }
			  }
		      flonref=lonref;
		      if(lonref<0.0)
			  flonref=flonref+360.0;
			  
		      sprintf(pair,"");
		      
		      
    /* Calculate latitude if there is a paired ODD/EVEN message to new message */
		      if(lat0 != 0.0 && lat1 != 0.0)  
		      {
			  latindex = floor((59.0*lat0-60.0*lat1)/pow(2.0,17.0)+0.5);   	//calculate lat index
			  rlat0=dlat0*(fmod(latindex,60.0)+lat0/pow(2.0,17.0));  	//even latitude
			  rlat1=dlat1*(fmod(latindex,59.0)+lat1/pow(2.0,17.0));   	// odd latitude
			  NL0=NLtablelookup(rlat0);    					// lookup latitude zones
			  NL1=NLtablelookup(rlat1);
		      }
			
			
    /* Check if paired messages are within same latitude zone */
		      if(NL0 == NL1)
		      {
			  sprintf(pair,"p\0");
		      }
		      
    /* If no message pair or messages in different latitude zones use local latref method */
		      else if(lat0 != 0.0 && latref != 0.0 && cpr[m] == 0)
		      {
			  latindex = floor(0.5+(fmod(latref,dlat0)/dlat0) - lat0/pow(2.0,17.0));
			  if(latindex == 0.0)
			  {
			    latindex = floor(latref/dlat0) + floor(0.5+(fmod(latref,dlat0)/dlat0) - lat0/pow(2.0,17.0));
			    rlat0=dlat0*(fmod(latindex,60.0)+lat0/pow(2.0,17.0));  //even latitude
			    NL0=NLtablelookup(rlat0);    // lookup latitude zones
			    NL1=NL0;
			    sprintf(pair,"s\0"); 
			  }
		      }
		      else if(lat1 != 0.0 && latref != 0.0 && cpr[m] == 1)
		      {
			  latindex = floor(0.5+(fmod(latref,dlat1)/dlat1) - lat1/pow(2.0,17.0));
			  if(latindex == 0.0)
			  {
			    latindex = floor(latref/dlat1) + floor(0.5+(fmod(latref,dlat1)/dlat1) - lat1/pow(2.0,17.0));
			    rlat1=dlat1*(fmod(latindex,59.0)+lat1/pow(2.0,17.0));   // odd latitude
			    NL1=NLtablelookup(rlat1);    // lookup latitude zones
			    NL0=NL1;
			    sprintf(pair,"s\0"); 
			  }
		      }    
				    
		      if(strcmp(parity,"000000\0") != 0)
			  sprintf(pair,"C\0"); 
		      
				    
		      if(cpr[m] == 0)
			  latitude=rlat0;
		      if(cpr[m] == 1)
			  latitude=rlat1; 	    
	  
		      if(latitude < -180.0) 
			latitude = latitude+360.0;
		      else if(latitude > 180.0) 
			latitude = latitude-360.0;
			    
      // 		 if(longrec[m] == 0.0 && latrec[m] == 0.0)
      // 		 {
		      if(sbsRECV[18] == 0x00 &&     // check for complete lat lon entry
			  sbsRECV[17] == 0x00 && 
			  sbsRECV[16] == 0x00 && 
			  sbsRECV[15] == 0x00 && 
			  sbsRECV[14] == 0x00 && 
			  sbsRECV[13] == 0x00)
			{
			    m=m-1;
			}
		      else if(NL0 == NL1) 
			{
			    if(cpr[m] ==0)
			      rlon=lon0;
			    if(cpr[m] ==1)
			      rlon=lon1; 
			    
			    ni=NL0-cpr[m];
			    if(ni < 1)
			      ni=1;
			    dlon=360.0/ni;
			    
			    
    /* Calculate longitude if there is a paired ODD/EVEN message to new message */
			    if(lon0 != 0.0 && lon1 != 0.0)   // if there is a paired ODD/EVEN to message
			    {	    	
			      longindex=floor(((lon0*(NL0-1)-lon1*NL0)/pow(2.0,17.0))+0.5);  // calculate long index
			      longitude=dlon*(fmod(longindex,1.0*ni)+(rlon/pow(2.0,17.0)));
			    }
			    
    /* If no message pair use local last known position lonref method */
			    else if(lon0 != 0.0 && lonref != 0.0)
			    {
			      longindex = floor(lonref/dlon) + floor(0.5+(fmod(flonref,dlon)/dlon) - lon0/pow(2.0,17.0));
			      longitude=dlon*(longindex+(lon0/pow(2.0,17.0)));			
			    }
			    else if(lon1 != 0.0 && lonref != 0.0)
			    {
			      longindex = floor(lonref/dlon) + floor(0.5+(fmod(flonref,dlon)/dlon) - lon1/pow(2.0,17.0));
			      longitude=dlon*(longindex+(lon1/pow(2.0,17.0)));
			    }
			 	
			 	
    /* Keep longitude between 0 - 360 */     
			    if(longitude >180.0) 
			      longitude = longitude-360.0;
			    else if(longitude <-180.0) 
			      longitude = longitude+360.0;			    
				
			    
    /* Convert to radians */
			    latrad=2.0*pi*latitude/360.0;
			    longrad=2.0*pi*longitude/360.0; 
			    
			    
    /* Calculate aircraft azimuth, elevation and range by 
     * converting to XYZ coordinates first */
			    smjr=6378137.0;   // semi major axis of Earth
			    smnr=smjr-smjr/298.257222101;   // semi minor axis of Earth
			    ecc=sqrt(pow(smjr,2.0) - pow(smnr,2.0))/smjr;
			    Nradi=smjr / sqrt(1 - pow(ecc,2.0) * sin(latrad) * sin(latrad));
			    X=(Nradi+0.3048*altitude[m])*cos(latrad)*cos(longrad);
			    Y=(Nradi+0.3048*altitude[m])*cos(latrad)*sin(longrad);
			    Z=((1-pow(ecc,2.0))*Nradi+0.3048*altitude[m])*sin(latrad);
			    
			    v[0]=X-STAT_X;
			    v[1]=Y-STAT_Y;
			    v[2]=Z-STAT_Z;
			    w[0]=v[0]*sin(STAT_latr)*cos(STAT_longr) + v[1]*sin(STAT_latr)*sin(STAT_longr) - v[2]*cos(STAT_latr);
			    w[0]=-1.0*w[0];
			    w[1]=-1.0*v[0]*sin(STAT_longr) + v[1]*cos(STAT_longr);
			    w[2]=v[0]*cos(STAT_latr)*cos(STAT_longr) + v[1]*cos(STAT_latr)*sin(STAT_longr) + v[2]*sin(STAT_latr);
			    
			    Rng = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
			    Azimuth=fmod(atan2(w[1],w[0]), 2.0*pi);
			    Azimuth=360.0*Azimuth/(2.0*pi);
			    if(Azimuth < 0.0) Azimuth = Azimuth+360.0;
			    
			    Elevation=atan2(w[2], sqrt(w[0]*w[0] + w[1]*w[1]));
			    Elevation=360.0*Elevation/(2.0*pi);    
			    
	       
    /* Find position of plane id in arrays */
			    pln=-1;
			    for(k=0;k<numl;k++)
			    {
			      if(strcmp(ICAO[m],oI[k])== 0)
				pln=k;
			    }
	    
			    if(pln == -1)
			    {
			      pln=numl;
			      numl=numl+1; 
			      EWvel[pln]=0.0;
			      NSvel[pln]=0.0;
			      Altvel[pln]=0.0;
			      sprintf(flightnumber[pln],"--------\0");
			    }
	    	    
	    	    
    /* Update aircraft data entries */
			    olat[pln]=latitude;
			    latr[pln]=latitude;
			    olong[pln]=longitude;
			    longr[pln]=longitude;
			    oalt[pln]=altitude[m];
			    oAz[pln]=Azimuth;
			    oEl[pln]=Elevation;
			    oep[pln]=tod[m];
			    oRng[pln]=Rng;
			    sprintf(oI[pln],"%s",ICAO[m]);
			    sprintf(opair[pln],"%s",pair);
			    sprintf(tstring[pln],"%2.0f:%02.0f:%04.1f\0",hr,min,sec);
		
			    
    /* Remove aircraft data entries after 120 seconds */
			    e=0;
			    for(k=0;k<numl;k++)
			    {
			      if((tod[m]-oep[k] <= 120.0) && (tod[m] >= oep[k]))
			      {
				olat[e]=olat[k];
				latr[e]=latr[k];
				olong[e]=olong[k];
				longr[e]=longr[k];
				oalt[e]=oalt[k];
				oAz[e]=oAz[k];
				oEl[e]=oEl[k];
				oep[e]=oep[k];
				oRng[e]=oRng[k];
				sprintf(oI[e],"%s",oI[k]);
				sprintf(opair[e],"%s",opair[k]);
				sprintf(tstring[e],"%s",tstring[k]);
				EWvel[e]=EWvel[k];
				NSvel[e]=NSvel[k];
				Altvel[e]=Altvel[k]; 
				sprintf(flightnumber[e],"%8s",flightnumber[k]);
				if(k == pln)
				  pln=e;
				e=e+1;
			      }			    
			    }
			    numl=e;
			    
			        
    /* Remove aircraft reference positions after 60 seconds */
			    for(k=0;k<numl;k++)
			    {
			      if((tod[m]-oep[k] >= 60.0) && (tod[m] >= oep[k]))
			      {
				latr[k]=0.0;
				longr[k]=0.0;
      // 			  printf("REf reset %d %s\n",k,oI[k]);
			      }
			    }		  
			}
			m=m+1;
		    }   
		    
    /* Read in type code 4 for flight number */
		    else if(Tcode[m] == 4.0)
		    {
 
    /* Find position of plane id in arrays */
		      pln=-1;
		      for(k=0;k<numl;k++)
		      {
			if(strcmp(ICAO[m],oI[k])== 0)
			      pln=k;
		      }
		       if(pln >= 0)
		      {
		      
			  oep[pln]=tod[m]; 
			  k=0;
			  emcat=0.0;     
			  for(j=5;j<8;j++)
			    {
			      emcat=emcat+squitter[j]*pow(2.0,2.0-1.0*k);
			      k=k+1;
			    } 
			    
			  sprintf(flightnumber[pln],"");     
			  for(k=0;k<8;k++)
			    {
			      sprintf(fnc[k],"%1d%1d%1d%1d%1d%1d\0",squitter[8+k*6+0],squitter[8+k*6+1],squitter[8+k*6+2],squitter[8+k*6+3],squitter[8+k*6+4],squitter[8+k*6+5]);
			      
    /* Call flightnum_character() function for each 6bit string to give one of 8 characters*/
			      fli=flightnum_character(fnc[k]);
			      strcat (flightnumber[pln], fli);
			    } 
			  if(strcmp(flightnumber[pln],"        \0") == 0)   // to stop planes broadcasting blank ids
			    sprintf(flightnumber[pln],"--------\0");
		      }
		    }
		    
		    
    /* Read in type code 19 for aircraft velocity components */
		    else if(Tcode[m] == 19.0)
		    {  
		      
		      
    /* Find position of plane id in arrays */
		      pln=-1;
		      for(k=0;k<numl;k++)
		      {
			if(strcmp(ICAO[m],oI[k])== 0)
			      pln=k;
		      }
		      
		       if(pln >= 0)
		      {
			oep[pln]=tod[m]; 
			k=0;
			
			
    /* Read subtype */
			subtype=0.0;     
			for(j=5;j<8;j++)
			  {
			    subtype=subtype+squitter[j]*pow(2.0,2.0-1.0*k);
			    k=k+1;
			  } 
			
			
    /* Read East West direction and magnitude*/
			EW=squitter[13];
			k=0;
			vEW=0.0;         
			for(j=14;j<24;j++)
			  {
			    EWbin[k]=squitter[j];
			    vEW=vEW+EWbin[k]*pow(2.0,9.0-1.0*k);
			    k=k+1;
			  } 
			vEW=vEW-1.0;
			if(subtype ==2.0 || subtype ==4.0)
			  vEW=4.0*vEW;
			if(EW == 1)
			  vEW=-1.0*vEW;
			  
			
    /* Read North South direction and magnitude*/
			NS=squitter[24];
			k=0;
			vNS=0.0;      
			for(j=25;j<35;j++)
			  {
			    NSbin[k]=squitter[j];
			    vNS=vNS+NSbin[k]*pow(2.0,9.0-1.0*k);
			    k=k+1;
			  }  
			  vNS=vNS-1.0;
			  if(subtype ==2.0 || subtype ==4.0)
			    vNS=4.0*vNS;
			  if(NS == 1)
			    vNS=-1.0*vNS;
			  
			  
    /* Read altitude direction and magnitude*/
			Alt=squitter[36];
			k=0;
			vAlt=0.0;      
			for(j=37;j<46;j++)
			  {
			    Altbin[k]=squitter[j];
			    vAlt=vAlt+Altbin[k]*pow(2.0,8.0-1.0*k);
			    k=k+1;
			  }  
			  if(vAlt != 0.0)
			  {
			    vAlt=64.0*(vAlt-1.0);
			    if(Alt == 1 && vAlt > 0.0)
			      vAlt=-1.0*vAlt;
			  }
			    
			if(NSvel[pln] != vNS || EWvel[pln] != vEW || Altvel[pln] != vAlt)
			{
			  NSvel[pln]=vNS;
			  EWvel[pln]=vEW;
			  Altvel[pln]=vAlt;
			}
			else
			{
			  pln =-1;
			}
		      }				
		    }
		    
		    
    /* Write to tcpip send string */
 		    if(pln != -1)
		    {
		      sprintf(SNDbuf,"%5.0f %10.3lf %s %s %10.5lf  %10.5lf  %6.0lf  %8.4lf  %13.8lf  %11.8lf  %6.1lf  %6.1lf  %5.0lf",jd, oep[pln],oI[pln],flightnumber[pln],olat[pln],olong[pln],oalt[pln],oRng[pln]/1000.0,oAz[pln],oEl[pln],NSvel[pln],EWvel[pln],Altvel[pln]);
		      sprintf(stype,"s\0");
		    }
 		   
		}
	      }
	    }
	    
    /* Send the new telescope data through server when no ADS-B to send */
	    if(pln == -1)
	    {
	      
	      gettimeofday(&tv, NULL);
	      pcseconds = (tv.tv_sec)  + (tv.tv_usec) / 1e6 ; 

	      jd=floor(2440587+pcseconds/86400.0);
	      fod=(2440587+pcseconds/86400.0)-1.0*jd;
	      jd=jd-2400000.0;
	      tep=jd + fod;              // epoch of message
	    
	      if(tep-telesc_tickep > 1.0/86400.0)
	      {
		sprintf(SNDbuf,"%5.0f %10.3lf telscp %6.2lf  %5.2lf %d",jd,fod*86400.0,Tazim, Telev,firel);
		telesc_tickep=tep;
		sprintf(stype,"t\0");
		firel=2;
	      }

	    }
	    
	    timeout.tv_sec = 0;
	    timeout.tv_usec = 10000; 
	    
	     
	    if((pln == -1 || oEl[pln] > Elcutoff) && strcmp(SNDbuf,"#\0") != 0)
	    {    
	      
    /* Use previous time stamp for telescope messages */
	      if(pln == -1) pln=opln;
		
    /* Check for connections using the select() function */   
	      seln = select(fdmax+1, &read_fds, 0, 0, &timeout);
	      if(seln == -1)
		{
		  printf("%s  Break - Server select() error.  #conx %d\n",tstring[pln],fdmax);
		  break;
		} 	  
	      else if(seln > 0)
		{
		  
	/* One or more descriptors are readable.  Need to send data to each */
		  desc_ready = seln;
		  for(cx = 0; cx <= fdmax &&  desc_ready > 0; cx++) 
		  { 
		    
	/* Check to see if this descriptor is ready */
		    fd=FD_ISSET(cx, &read_fds);
		    if(fd)
		    {   
		      desc_ready -= 1;	      
		      if(cx == listener)
		      {
			addrlen = sizeof(clientaddr);
			
	/* Connection request on original socket. */ 
			if((newfd = accept(listener,  (struct sockaddr *)&clientaddr, &addrlen)) == -1)
			{ 
			  printf("%s  %s: Accept Error Break\n",tstring[pln], argv[0]);
			  break;
			}
			FD_SET(newfd, &master); 		/* add to master set */
			if(newfd > fdmax)
			  fdmax = newfd;			/* keep track of the maximum */
			  
			sprintf(cxtype[newfd],"n\0");
			printf("\n%s  %s: New connection from address %s on socket number %d\n",tstring[pln], argv[0],  inet_ntoa(clientaddr.sin_addr),newfd);
		      }
		      else
		       {
                         close_conn = FALSE;

			if(strcmp(cxtype[cx],"c\0") == 0 || strcmp(cxtype[cx],"r\0") == 0 || strcmp(cxtype[cx],"n\0") == 0 || (strcmp(cxtype[cx],"t\0") == 0 && strcmp(stype,"t\0") == 0))
			{
			  rc = recv(cx, RECVbuf, sizeof(RECVbuf), 0);
			  if (rc <= 0)
			   {
				close_conn = TRUE;
       			        printf("%s  %s: Server Recv Error: Closing connection %d\n",tstring[pln], argv[0],cx);

			   }

			   
	/* Check for new connection type */
			  if(strcmp(cxtype[cx],"n\0") == 0)
			    {
			      sscanf(RECVbuf,"%s",&buf);
			      
	  /* Determine message type, telescope or client reply. */
			      if(strncmp(buf,"client",6) == 0)
				sprintf(cxtype[cx],"c\0");
			      if(strncmp(buf,"reader",6) == 0)
				sprintf(cxtype[cx],"r\0");
			      if(strncmp(buf,"telscp:",7) == 0)
				sprintf(cxtype[cx],"t\0");
			      
			    }
			 
			  
	  /* Read client message for laser fire control  */
			  if(strcmp(cxtype[cx],"c\0") == 0)
			  {
			    sscanf(RECVbuf,"%s %*s %d",&buf,&firel);
			    if(firel < 1 & fireo != firel)
			      telesc_tickep=0.0;
			    fireo=firel;
			  }
			  
	  /* Read buffer sent from telescope client on connection cx */
			  else if(strcmp(cxtype[cx],"t\0") == 0)
			  {
			      sscanf(RECVbuf,"%s %f %f",&buf, &Tazim,&Telev);
			      if(Tazim > 360.0)
				  Tazim=Tazim-360.0;
			      
			  }

			   
			  len = strlen(SNDbuf)+1;  
			  rc = send(cx, SNDbuf, len, MSG_NOSIGNAL); 
			  if (rc <= 0)
			  {
			    close_conn = TRUE;
			    printf("%s  %s: Server Send Error: Closing connection %d\n",tstring[pln], argv[0],cx);
			  }
			   

                            if (close_conn)
                            {
			      close(cx);				 /* close failed connection */
			      FD_CLR(cx, &master);			 /* remove from master set */
			      if (cx == fdmax)
				{
				  if (FD_ISSET(fdmax, &master) == 0)
				      fdmax -= 1;			 /* reduce master set number */
				}
                              if(strcmp(cxtype[cx],"t\0") == 0)
                                {
				  Tazim=0.0;
				  Telev=0.0;
			        }
			      sprintf(cxtype[cx]," \0");
			    
			    }
			  } // endif select on cxtype and stype
		      }  // endif cx is listener
		    }  // endif file descripter ready
		  }  // for loop through connections cx
		} // endif select detects ready connections
	      sprintf(SNDbuf,"#"); 
// 	      usleep(5000);				 /* wait 5ms*/
	    } 
  
	     if(pln != -1) opln=pln;	    
    }    
  }
  return 0;
  }
  
  
  
    /* Function to determine which latitude zone the messages were sent from */
int NLtablelookup(double lat)
{
  int NL;
  float NL2;
  int NZ;
  float pi=3.1415926535898;
  
  NZ=15;

  NL2=  floor(2.0*pi/(acos( 1 - ( (1-cos(pi/(2.0*NZ))) / pow(cos(pi*fabs(lat)/180.0),2.0))))); 
  
  NL=NL2;
  return NL;

}



    /* Function to convert a 6bit string to a flight number character */
char * flightnum_character(char str6[])
{ 
  int bit[6];
  int id1,id2;
  const char* a[] = {" ", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O"};
  const char* b[] = {"P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", " ", " ", " ", " ", " "};
  const char* c[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", " ", " ", " ", " ", " ", " "};
  char * outc = malloc(2);
    
  sscanf(str6,"%1d%1d%1d%1d%1d%1d",&bit[0],&bit[1],&bit[2],&bit[3],&bit[4],&bit[5]);
  
  id1=1*bit[1]+2*bit[0];
  id2=1*bit[5]+2*bit[4]+4*bit[3]+8*bit[2];
  
  if(id1 == 0)
  { 
    sprintf(outc,"%1s\0",a[id2]);
  }
  else if(id1 == 1)
  {
    sprintf(outc,"%1s\0",b[id2]);
  }
  else if(id1 == 2)
  {
    sprintf(outc," \0");
  }
  else if(id1 == 3)
  { 
    sprintf(outc,"%1s\0",c[id2]);
  } 
  
    return outc;  
  
}


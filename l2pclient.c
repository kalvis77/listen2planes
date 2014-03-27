
/* *** l2pclient.c ***
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
 * The listen2planes client connects yo the TCP/IP address of the PC running the l2pserver
 * on the port 2020.The IP address must be input to the variable srvIP below. The client will
 * receive the stream of translated aircraft positions and velocities and output these to the screen.
 * 
 * It uses the latitude, longitude and altitude velocities to predict the aircraft position
 * 10 seconds and 2 seconds in to the future.
 * 
 * If the telescope positions have also been fed in to the server the client will also display these
 * telescope azimuth and elevation updates.
 * 
 * Set YNclient to TRUE to enable the aircraft warning system. A boundary is defined around each
 * aircraft using its instantaneous position and predicted positions. The boundary is larger around the
 * predictions in a triangular shape and these values can be adjusted. If the telescope is pointing within
 * these boundaries alarms are sounded to warn the observer. Sounds taken from http://www.freesound.org
 * 
 *	The Linux compile command is:
 *	  	gcc -lm -o l2pclient l2pclient.c
 * 
 * 	The Linux command is:
 * 		l2pclient <Elevation Cut Off> <Write to file l2p.out y/n>     	or
 * 
 * 		l2pclient	(this uses a default elevation cutoff of
 *				 10 degrees and turns write to file off)
 */


#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define BUFFER_SIZE 256
#define TRUE  1
#define FALSE 0
  
struct point { double Azimuth; double Elevation; double Rng;};		/* struct for Az,El,range coords */
struct point STATazel(double latrad, double longrad, double altitude);	/* struct for lat,long,alt coords */
  
int main(int argc, char *argv[])
{
    /* tcpip variables */
        int cn;							/* Output from connect() */
        int sock;						/* socket for tcpip connection to server */
	int len; 						/* Buffer length */
	int oldflags;						/* Used to set l2pserver to non-blocking */
	int numl;						/* Number of detected aircraft */
	float Elcutoff;						/* Elevation cutoff for display */
        char SNDbuf[BUFFER_SIZE];				/* Send buffer string */
        char RECVbuf[BUFFER_SIZE];				/* Receive buffer string */
        char srvIP[15];						/* IP address of PC running l2pserver */
        struct hostent *host;
        struct sockaddr_in server_addr; 
        	
    /* read buffer input variables form l2pserver */ 
	char itype[7];						/* Receive message type */
	float jd[256];						/* modified julian day */
	double tod[256];					/* time of day, seconds */
	char tstring[256][11];					/* time string */
	char ICAO[256][7];					/* aircraft ID */
	char flightnumber[256][9];				/* aircraft flight number */
	double latitude[256];					/* aircraft latitude */
	double longitude[256];					/* aircraft longitude */
	double altitude[256];					/* aircraft altitude */
	double rng[256];					/* aircraft range */
	double azimuth[256];					/* aircraft azimuth */
	double elevation[256];					/* aircraft elevation */
	double NSvel[256];					/* North South velocity */
	double EWvel[256];					/* East West velocity */
	double altvel[256];					/* Altitude velocity */
	double Tazim, Telev;					/* Read telescope azimuth and elevation used*/
	
    /* aircraft alarm variables */  
    	int YNclient;
	double nowep;						/* current epoch */
	double alarmep,alarmep2;				/* alarm epochs */
	double waitep; 						/* limit updates */
	double wep;						/* 10s warning epoch */
	double wlat,wlong,walt;					/* 10s warning coordinates */
	double aep;						/* 2s warning epoch */
	double alat,along,aalt;					/* 2s warning coordinates */
	double wpredR[256];					/* Warning 10s ahead range */
	double wpredAz[256];					/* Warning 10s ahead azimuth */
	double wpredEl[256];					/* Warning 10s ahead elevation */
	double apredR[256];					/* Alarm 2s ahead range */
	double apredAz[256];					/* Alarm 2s ahead azimuth */
	double apredEl[256];					/* Alarm 2s ahead elevation */	
	double wTazim, wTelev;					/* Warning telescope azimuth and elevation */
	double aTazim, aTelev;					/* Alarm telescope azimuth and elevation */	
	
	double slope;						/* slope el/az between aircraft position and prediction */
	double Elv;						/* Elevation to use in cosfacter calculations */
	double el2;						/* aircraft el on pos to pred line */
	double az2;						/* aircraft az on pos to pred line */
	double azlen;						/* Az separation between telescope and telescope on pos-pred line */ 
	double ellen;						/* El separation between telescope and telescope on pos-pred line */
	double theta;						/* angle to calculate telescope pos-pred line separation */ 
	double wR; 						/* Min separation of aircraft to warning pos-pred line */
	double aR;						/* Min separation of aircraft to alarm pos-pred line */
	double Tazim2;						/* Az of closest point to telescope on pos-pred line */
	double Telev2;						/* El of closest point to telescope on pos-pred line */
	double wbndry;						/* size of boundary around warning */
	double abndry;						/* size of boundary around alarm area */
	double triw;						/* degree of spread of warning boundary around pred */
	double tria;						/* degree of spread of alarm boundary around pred */
	double scale;						/* distance along pos-pred line */
	double wps_azsep;					/* Warning Az separation between telescope and plane */
	double wps_elsep;					/* Warning El separation between telescope and plane */
	double aps_azsep;					/* Alarm Az separation between telescope and plane */
	double aps_elsep;					/* Alarm El separation between telescope and plane */

	double wpr_azsep;					/* Az separation between warning prediction and telescope */
	double wpr_elsep;					/* El separation between warning prediction and telescope */
	double wpl_azsep;					/* Az separation between plane and warning prediction */
	double wpl_elsep;					/* El separation between plane and warning prediction */
	double wps_azsep2;					/* Az separation between telescope on warning line and plane position */
	double wps_elsep2;					/* El separation between telescope on warning line and plane position*/
	double wpr_azsep2;					/* Az separation between telescope on line and warning prediction */
	double wpr_elsep2;					/* El separation between telescope on line and warning prediction */
	double wl2pos1;						/* Difference on pos-pred line from aircraft position to telescope on line and pred */
	double wl2pos2;						/* Difference on pos-pred line from aircraft pred to telescope on line and position */
	int wpssign;
	int wprsign;
	
	double apr_azsep;					/* Az separation between alarm prediction and telescope */
	double apr_elsep;					/* El separation between alarm prediction and telescope */
	double apl_azsep;					/* Az separation between plane and alarm prediction*/
	double apl_elsep;					/* El separation between plane and alarm prediction*/
	double aps_azsep2;					/* Az separation between telescope on alarm line and plane */
	double aps_elsep2;					/* El separation between telescope on alarm line and plane */
	double apr_azsep2;					/* Az separation between and telescope on line and alarm prediction*/
	double apr_elsep2;					/* El separation between telescope on line and alarm prediction*/
	double al2pos1;						/* Difference on pos-pred line from aircraft position to telescope on line and pred */
	double al2pos2;						/* Difference on pos-pred line from aircraft pred to telescope on line and pred */
	int apssign;
	int aprsign;
	
	int woverh,aoverh;					/* Flag to indicate if the warning and alarm plane pred lines goes over the zenith */
	double Tcosfactor;					/* cos scale factor for telescope elevation azimuths */
	double wPLcosfactor,aPLcosfactor;			/* cos scale factors plane position and pred separation */
	double wPScosfactor,aPScosfactor;			/* cos scale factors plane position and telescope separation */
	double wPRcosfactor,aPRcosfactor;			/* cos scale factors plane pred and telescope separation */
	double wPS2cosfactor,aPS2cosfactor;			/* cos scale factors plane position and telescope on line separation */
	double wPR2cosfactor,aPR2cosfactor;			/* cos scale factors plane pred and telescope on line separation */
	
	struct timeval tv;					/* struct to read system time */
	float hr,min,sec;					/* time variables */
	double Nt, SNDt;					/* epochs to monitor time since last send() */
	char fwout[2];						/* Y/N flag to print output to a file */
	char hlp[3];						/* Help -h flag */
	int fire;						/* result ok to fire */							
	int firel;						/* flag to control fire */	
	int fireo;						/* flag to broadcast by l2pserver to control fire */
        int l,k,e;
	double pi=3.1415926535898;
		
	FILE *fid;						/* output file id */
	
    /* Input IP address of PC running l2pserver */
	sprintf(srvIP,"123.123.123.123");
	sprintf(srvIP,"193.61.194.29");
	
    /* Read command line arguments */
	if(argc == 3)
	{
	  sscanf(argv[1],"%f",&Elcutoff);
	  sscanf(argv[2],"%c",&fwout);
	  fwout[1]='\0';
	}
	else if(argc == 2)
	{
	  sscanf(argv[1],"%s",hlp);
	  if(strcmp(hlp,"-h") == 0)
	  {
	    printf("\n    Usage: %s <elev cut off> <write to file l2p.out y/n>\n",argv[0]);
	    return 1;
	  }
	} 
	else if(argc == 1)
	{
	  printf("\n Using defaults.  Use -h for help\n");
	  sleep(1);
	  Elcutoff=10;
	  sprintf(fwout,"n\0");
	}
	else
	{
	  printf("\n    Usage: %s <elev cut off> <write to file l2p.out y/n>\n",argv[0]);
	  return 1;
	}
	
	
    /* Open output write file */
	if(strcmp(fwout,"y") == 0)
	    fid=fopen("l2p.out","w");
	
	
    /* flag to enable client warning system (requires telescope positions) , TRUE/FALSE */
	YNclient=TRUE;						
	
    /* Initialise variables */
	numl=0;
	waitep=0.0;
	alarmep=0.0;
	alarmep2=0.0;
	firel=1;
	firel=1;
	Nt = 0;
	SNDt = 0;
	memset(RECVbuf, 0,sizeof(RECVbuf));
	memset(SNDbuf, 0,sizeof(SNDbuf));
	
    /* Set size and shape of boundaries around aircraft */
    /*** Adjust these variables to increase or decrease no-fire zone around aircraft */ 
	wbndry=3.0;
	abndry=1.5;
	triw=2.0;
	tria=1.5;
	
	
    /* Setup tcpip socket networking variables */
        host = gethostbyname(srvIP);

        if ((sock = socket(PF_INET, SOCK_STREAM, 0)) == -1) 
	{
	    printf("%s: Socket error\n",argv[0]);
            exit(1);
        }

	memset(&server_addr, -1, sizeof(server_addr));
        server_addr.sin_family = PF_INET;     
        server_addr.sin_port = htons(2020);   
        server_addr.sin_addr = *((struct in_addr *)host->h_addr);
        bzero(&(server_addr.sin_zero),8); 

	cn=connect(sock, (struct sockaddr *)&server_addr,sizeof(struct sockaddr));
        if (cn == -1) 
        {
	    printf("%s: Connect error\n",argv[0]);
            exit(1);
        }
	
	// set the incoming socket to non-blocking 
	oldflags = fcntl (sock, F_GETFL, 0);
	oldflags |= O_NONBLOCK;
	fcntl (sock, F_SETFL, oldflags);
  
//         sprintf(SNDbuf,"client   fire %d\0",firel);
// 	len = send(sock, SNDbuf, strlen(SNDbuf)+1,0);
           
   
    /* If connection was successful put program in to a permanent while loop */	  
        while(1)
        {   
	  
    /* Receive data buffer from l2pserver */
          len=recv(sock,RECVbuf,sizeof(RECVbuf),0);
	  	  
	  if (len == 0)
	  {
	    printf("%s: Recv error\n",argv[0]);
	    close(sock);
	    fclose(fid);
	    exit( -1);
	  }
	  
	  	
    /* Print to file */	  
	  if(strcmp(fwout,"y") == 0)
	  { 
	    if (len > 10)
	       fprintf(fid,"%s\n",RECVbuf);
	  }  
	  
    /* Read data buffer type as 3rd variable */
 	  sscanf(RECVbuf,"%*f %*lf %s",&itype);
	  

    /* Read buffer containing telescope direction */
	  if(strcmp(itype,"telscp\0")== 0)
	  { 
	    sscanf(RECVbuf,"%f %lf %*s  %lf  %lf %d",&jd[numl], &tod[numl],&Tazim,&Telev,&fireo);
	    nowep=tod[numl];

	  }
    /* Read buffer containing aircraft position and velocity information */
	  else if(len > 30)
	  {
	    sscanf(RECVbuf,"%f %lf %s %s %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %lf",&jd[numl], &tod[numl],&ICAO[numl],&flightnumber[numl],&latitude[numl],&longitude[numl],&altitude[numl],&rng[numl],&azimuth[numl],&elevation[numl],&NSvel[numl],&EWvel[numl],&altvel[numl]);
	  
    /* Create time string for message */
	    nowep=tod[numl];
	    hr=floor(24.0*(tod[numl]/86400.0));
	    min=floor((tod[numl] - hr*3600.0)/60.0);
	    sec=tod[numl] - hr*3600.0 - min*60.0;
	    sprintf(tstring[numl],"%2.0f:%02.0f:%04.1f\0",hr,min,sec);
	    
    /* Check plane ID with buffer record */
	    l= -1;
	    for(k=0;k<numl;k++)
	    {
	      if(strcmp(ICAO[k],ICAO[numl])== 0)
	      l=k;
	    }

    /* Add entry if not in buffer record */
	    if(l == -1)
	    {
	      l=numl;
	      numl=numl+1;
	    }
	    else
	    {
	      jd[l]=jd[numl];
	      tod[l]=tod[numl]; 
	      sprintf(tstring[l],"%s",tstring[numl]);
	      sprintf(ICAO[l],"%s",ICAO[numl]);
	      sprintf(flightnumber[l],"%s",flightnumber[numl]);
	      latitude[l]=latitude[numl];
	      longitude[l]=longitude[numl];
	      altitude[l]=altitude[numl];
	      rng[l]=rng[numl];
	      azimuth[l]=azimuth[numl];
	      elevation[l]=elevation[numl];
	      NSvel[l]=NSvel[numl];
	      EWvel[l]=EWvel[numl];
	      altvel[l]=altvel[numl];
	    }
	  }    
	
	  
	  gettimeofday(&tv, NULL);
	  Nt = (tv.tv_sec)  + (tv.tv_usec) / 1e6 ; 
	  if (len > 0 || Nt-SNDt > 0.5) 
	  {
    /* Update send buffer with fire status */
	  if(YNclient)
	    sprintf(SNDbuf,"client   fire %d\0",firel);
	  else
	    sprintf(SNDbuf,"reader\0");
	  
	  len = send(sock, SNDbuf, strlen(SNDbuf)+1,0);
	  if(Nt-SNDt > 0.5) 
	    waitep=0.0;
	  SNDt = Nt;
	  }
	  
	  
    /* Restrict updates and calculations to a few times per second */
	  if(fabs(nowep - waitep) > 0.3)
	      { 
		
	      e=0;
	      for(k=0;k<numl;k++)
	      {  
		if((nowep - tod[k] <= 60.0) && (nowep >= tod[k]))
		{
		  
    /* refresh buffer with only the latest messages */
		  jd[e]=jd[k];
		  tod[e]=tod[k]; 
		  sprintf(ICAO[e],"%s",ICAO[k]);
		  sprintf(tstring[e],"%s",tstring[k]);
		  sprintf(flightnumber[e],"%s",flightnumber[k]);
		  latitude[e]=latitude[k];
		  longitude[e]=longitude[k];
		  altitude[e]=altitude[k];
		  rng[e]=rng[k];
		  azimuth[e]=azimuth[k];
		  elevation[e]=elevation[k];
		  NSvel[e]=NSvel[k];
		  EWvel[e]=EWvel[k];
		  altvel[e]=altvel[k];
		  
    /* define an epoch 10 seconds ahead and make a prediction using velocity components */
		  wep=nowep - tod[e]+10.0;
		  wlat=(NSvel[e]*wep/3600.0)/60.0;    // use 1nm ~= 1' latitude
		  wlong=((EWvel[e]*wep/3600.0)/cos(2.0*pi*latitude[e]/360.0))/60.0;  // Use cos(latitude) factor
		  walt=altvel[e]*wep/60.0;
		  
    /* call STATazel() to get Az and EL angles */ 
		  struct point STATsky = STATazel(2.0*pi*(latitude[e]+wlat)/360.0,
					      2.0*pi*(longitude[e]+wlong)/360.0,
					      (altitude[e]+walt));
		  wpredAz[e]=STATsky.Azimuth;
		  wpredEl[e]=STATsky.Elevation;
		  wpredR[e]=STATsky.Rng/1000.0;
		  
    /* Make 2 second predictions */
		  aep=nowep - tod[e]+2.0;			
		  alat=(NSvel[e]*aep/3600.0)/60.0;    // 1nm ~= 1'
		  along=((EWvel[e]*aep/3600.0)/cos(2.0*pi*latitude[e]/360.0))/60.0; 
		  aalt=altvel[e]*aep/60.0;
		  STATsky = STATazel(2.0*pi*(latitude[e]+alat)/360.0,
					      2.0*pi*(longitude[e]+along)/360.0,
					      (altitude[e]+aalt));	  
		  apredAz[e]=STATsky.Azimuth;
		  apredEl[e]=STATsky.Elevation;
		  apredR[e]=STATsky.Rng/1000.0;	  
		  
		    
      /* Only consider aircraft above elevation cutoff */
		    if(elevation[e] > Elcutoff && YNclient && Telev > 0.0)
		    {
		      Tcosfactor=cos(2.0*pi*Telev/360.0);
		      
		      wTazim=Tazim;
		      wTelev=Telev;
		      aTazim=Tazim;
		      aTelev=Telev;
		      woverh=FALSE;
		      aoverh=FALSE;
		      
		      wpl_azsep=wpredAz[e] - azimuth[e];				
		      if(fabs(wpl_azsep) >180.0)
			wpl_azsep=wpl_azsep - wpl_azsep*360.0/fabs(wpl_azsep);
		      
      /* Determine whether warning prediction crosses the zenith */
		      if(fabs(wpl_azsep) >=90.0)
		      {
      /* Adjust pred to use elevations > 90.0 and correct Az by 180.0 */
			wpredAz[e]=wpredAz[e]+180.0;
			if(wpredAz[e] > 360.0)
				wpredAz[e]=wpredAz[e]-360.0;
			wpredEl[e]=180.0 - wpredEl[e];
			woverh=TRUE;
		      }
		      
		      
      /* Determine whether alarm prediction crosses the zenith */
		      apl_azsep=apredAz[e] - azimuth[e];
		      if(fabs(apl_azsep) >180.0)
			apl_azsep=apl_azsep - apl_azsep*360.0/fabs(apl_azsep);
		      
		      if(fabs(apl_azsep) >=90.0)
		      {
      /* Adjust pred to use elevations > 90.0 and correct Az by 180.0 */
			apredAz[e]=apredAz[e]+180.0;
			if(apredAz[e] > 360.0)
			      apredAz[e]=apredAz[e]-360.0;
			apredEl[e]=180.0 - apredEl[e];
			aoverh=TRUE;
		      }
		      
		      
      /* If plane crossing zenith correct the telescope position 
       * to use elevations > 90.0 and correct Az by 180.0 for both predictions */
		      wps_azsep=Tazim - azimuth[e];
		      if(fabs(wps_azsep) >180.0)
			wps_azsep=wps_azsep - wps_azsep*360.0/fabs(wps_azsep);
		      
		      if(fabs(wps_azsep) >=90.0)
		      {
			if(woverh ==TRUE)
			{
			    wTazim=wTazim+180.0;
			    if(wTazim > 360.0)
				    wTazim=wTazim-360.0;
			    wTelev=180.0 - wTelev;
			}
			
			if(aoverh ==TRUE)
			{
			    aTazim=aTazim+180.0;
			    if(aTazim > 360.0)
				    aTazim=aTazim-360.0;
			    aTelev=180.0 - aTelev;
			}
		      }
		      
		      
		      
		      
      /* Calculate cos() factor values for each separation element used */
		      Elv=90.0-0.5*(fabs(90.0-wpredEl[e])+fabs(90.0-elevation[e]));
		      wPLcosfactor=fabs(cos(2*pi*Elv/360.0));
		      Elv=90.0-0.5*(fabs(90.0-apredEl[e])+fabs(90.0-elevation[e]));
		      aPLcosfactor=fabs(cos(2*pi*Elv/360.0));
		      
		      Elv=90.0-0.5*(fabs(90.0-wTelev)+fabs(90.0-elevation[e]));
		      wPScosfactor=fabs(cos(2*pi*Elv/360.0));
		      Elv=90.0-0.5*(fabs(90.0-aTelev)+fabs(90.0-elevation[e]));
		      aPScosfactor=fabs(cos(2*pi*Elv/360.0));
		      
		      Elv=90.0-0.5*(fabs(90.0-wTelev)+fabs(90.0-wpredEl[e]));
		      wPRcosfactor=fabs(cos(2*pi*Elv/360.0));
		      Elv=90.0-0.5*(fabs(90.0-aTelev)+fabs(90.0-apredEl[e]));
		      aPRcosfactor=fabs(cos(2*pi*Elv/360.0));
		      
		      
      /* Difference between predictions and current aircraft direction */
		      wpl_azsep = wpredAz[e] - azimuth[e];
		      wpl_elsep = wpredEl[e] - elevation[e];
		      apl_azsep = apredAz[e] - azimuth[e];
		      apl_elsep = apredEl[e] - elevation[e];   
			    
      /* Difference between telescope and current aircraft direction */
		      wps_azsep = wTazim - azimuth[e];
		      wps_elsep = wTelev - elevation[e];
		      aps_azsep = aTazim - azimuth[e];
		      aps_elsep = aTelev - elevation[e];
		      
      /* Difference between predictions and telescope direction */
		      wpr_azsep = wpredAz[e] - wTazim;
		      wpr_elsep = wpredEl[e] - wTelev;
		      apr_azsep = apredAz[e] - aTazim;
		      apr_elsep = apredEl[e] - aTelev;
		      
      /* Keep values between -180-180 degrees */
		      if(fabs(wps_azsep) >180.0)
			wps_azsep=wps_azsep - wps_azsep*360.0/fabs(wps_azsep);
		      if(fabs(aps_azsep) >180.0)
			aps_azsep=aps_azsep - aps_azsep*360.0/fabs(aps_azsep);
		      
		      if(fabs(wpl_azsep) >180.0)
			wpl_azsep=wpl_azsep - wpl_azsep*360.0/fabs(wpl_azsep);
		      if(fabs(apl_azsep) >180.0)
			apl_azsep=apl_azsep - apl_azsep*360.0/fabs(apl_azsep);
		      
		      if(fabs(wpr_azsep) >180.0)
			wpr_azsep=wpr_azsep - wpr_azsep*360.0/fabs(wpr_azsep);
		      if(fabs(apr_azsep) >180.0)
			apr_azsep=apr_azsep - apr_azsep*360.0/fabs(apr_azsep);
		      
	
		      
		      
		      if(wpl_azsep == 0.0 )
		      {
			wR=fabs(wps_azsep*Tcosfactor);
			theta=2*pi*90.0/360.0;
		      }
		      else if(wpl_elsep == 0.0 )
		      {
			wR=fabs(wps_elsep);
			theta=0.0;
		      }
		      else
		      {
      /* Slope is gradient of elevation with azimuth */
			slope=wpl_elsep/wpl_azsep;
			
      /* az2 and el2 are where the aircraft el and az extend to meet the pos-pred line */
			el2=elevation[e] + slope*wps_azsep;		
			az2=azimuth[e] + wps_elsep/slope;
		      
      /* Separations from pos-pred line to telescope components*/
			azlen=(Tazim - az2);
		      
			if(fabs(azlen) >180.0)
			  azlen=azlen - azlen*360.0/fabs(azlen);
      /* Scale azlen by telescope cos() factor*/
			azlen=azlen*Tcosfactor;
			ellen=Telev - el2;
		      
      /* Calculate angle of pos-pred line*/
			theta=atan2(ellen,azlen);
		      
      /* Calculate distance of telescope to pos-pred line*/
			wR=ellen*(cos(theta));
		      }
		      
      /* Az and El components of vector to pos-pred line */
		      Tazim2=wTazim - wR*sin(theta);
		      Telev2=wTelev - wR*cos(theta);
			  
      /* Az and El components of aircraft position to telescope on pos-pred line */
		      wps_azsep2=Tazim2 - azimuth[e];
		      wps_elsep2=Telev2 - elevation[e];
		      
      /* Az and El components of telescope on pos-pred line to aircraft pred*/
		      wpr_azsep2=wpredAz[e] - Tazim2;
		      wpr_elsep2=wpredEl[e] - Telev2;
		      
		      Elv=90.0-0.5*(fabs(90.0-Telev2)+fabs(90.0-elevation[e]));
		      wPS2cosfactor=fabs(cos(2*pi*Elv/360.0));
		      Elv=90.0-0.5*(fabs(90.0-Telev2)+fabs(90.0-wpredEl[e]));
		      wPR2cosfactor=fabs(cos(2*pi*Elv/360.0));
		      
			      
      /* Keep Az and El components within -180-180 */
		      if(fabs(wps_azsep2) >180.0)
			wps_azsep2=wps_azsep2 - wps_azsep2*360.0/fabs(wps_azsep2);
		      if(fabs(wpr_azsep2) >180.0)
			wpr_azsep2=wpr_azsep2 - wpr_azsep2*360.0/fabs(wpr_azsep2);	    
		      
		      
      /* Repeat part for alarm prediction */
      
		      if(apl_azsep == 0.0 )
		      {
			aR=fabs(aps_azsep*Tcosfactor);
			theta=2*pi*90.0/360.0;
		      }
		      else if(apl_elsep == 0.0 )
		      {
			aR=fabs(aps_elsep);
			theta=0.0;
		      }
		      else
		      {
			slope=apl_elsep/apl_azsep;
			el2=elevation[e] + slope*aps_azsep;		
			az2=azimuth[e] + aps_elsep/slope;
			azlen=(Tazim - az2);
			if(fabs(azlen) >180.0)
			  azlen=azlen - azlen*360.0/fabs(azlen);
			azlen=azlen*Tcosfactor;
			ellen=Telev - el2;
			theta=atan2(ellen,azlen);
			aR=ellen*(cos(theta));
		      }
		      
		      Tazim2 = aTazim - aR*sin(theta);
		      Telev2 = aTelev - aR*(cos(theta));
					      
      /* Az and El components of from aircraft position to telescope on pos-pred line */
		      aps_azsep2 = Tazim2 - azimuth[e];
		      aps_elsep2 = Telev2 - elevation[e];
      /* Az and El components of from telescope on pos-pred line to aircraft pred*/
		      apr_azsep2 = apredAz[e] - Tazim2;
		      apr_elsep2 = apredEl[e] - Telev2;
	  
		      Elv=90.0-0.5*(fabs(90.0-Telev2)+fabs(90.0-elevation[e]));
		      aPS2cosfactor=fabs(cos(2*pi*Elv/360.0));
		      Elv=90.0-0.5*(fabs(90.0-Telev2)+fabs(90.0-apredEl[e]));
		      aPR2cosfactor=fabs(cos(2*pi*Elv/360.0));
	  
      /* Keep Az and El components within 180-180 */
		      if(fabs(aps_azsep2) >180.0)
			aps_azsep2=aps_azsep2 - aps_azsep2*360.0/fabs(aps_azsep2);
		      if(fabs(apr_azsep2) >180.0)
			apr_azsep2=apr_azsep2 - apr_azsep2*360.0/fabs(apr_azsep2);
			    
				
		      wpssign=1.0;
		      wprsign=1.0;
		      if(wpl_azsep== 0.0)
		      {
			      if(wps_elsep2*wpl_elsep< 0.0)
				      wpssign=-1.0;
			      if(wpr_elsep2*wpl_elsep< 0.0)
				      wprsign=-1.0;
		      }
		      else if(wpl_elsep== 0.0)
		      {
			      if(wps_azsep2*wpl_azsep< 0.0)
				      wpssign=-1.0;
			      if(wpr_azsep2*wpl_azsep< 0.0)
				      wprsign=-1.0;
		      }				      
		      else
		      {	      if(atan2(wps_elsep2,wps_azsep2)*atan2(wpl_elsep,wpl_azsep)< 0.0)
				 wpssign=-1.0;
			      if(atan2(wpr_elsep2,wpr_azsep2)*atan2(wpl_elsep,wpl_azsep)< 0.0)
				 wprsign=-1.0;
		      }
	
		      
		      apssign=1.0;
		      aprsign=1.0;
		      if(apl_azsep== 0.0)
		      {
			      if(aps_elsep2*apl_elsep< 0.0)
				      apssign=-1.0;
			      if(apr_elsep2*apl_elsep< 0.0)
				      aprsign=-1.0;
		      }
		      else if(apl_elsep== 0.0)
		      {
			      if(aps_azsep2*apl_azsep< 0.0)
				      apssign=-1.0;
			      if(apr_azsep2*apl_azsep< 0.0)
				      aprsign=-1.0;
		      }				      
		      else
		      {	      if(atan2(aps_elsep2,aps_azsep2)*atan2(apl_elsep,apl_azsep)< 0.0)
				 apssign=-1.0;
			      if(atan2(apr_elsep2,apr_azsep2)*atan2(apl_elsep,apl_azsep)< 0.0)
				 aprsign=-1.0;
		      }
	
		      
		      
      /* Differences between warning pred point and telescope and telescope on pos-pred line */
		      wl2pos1=wprsign*sqrt(pow((wPR2cosfactor*wpr_azsep2),2.0) + pow(wpr_elsep2,2.0)) - sqrt(pow((wPLcosfactor*wpl_azsep),2.0) + pow(wpl_elsep,2.0));
		      wl2pos2=wpssign*sqrt(pow((wPS2cosfactor*wps_azsep2),2.0) + pow(wps_elsep2,2.0)) - sqrt(pow((wPLcosfactor*wpl_azsep),2.0) + pow(wpl_elsep,2.0));
		      
      /* Differences between alarm pred point and telescope and telescope on pos-pred line */
		      al2pos1=aprsign*sqrt(pow((aPR2cosfactor*apr_azsep2),2.0) + pow(apr_elsep2,2.0)) - sqrt(pow((aPLcosfactor*apl_azsep),2.0) + pow(apl_elsep,2.0));
		      al2pos2=apssign*sqrt(pow((aPS2cosfactor*aps_azsep2),2.0) + pow(aps_elsep2,2.0)) - sqrt(pow((aPLcosfactor*apl_azsep),2.0) + pow(apl_elsep,2.0));
				      
		      fire=1;
		      
      /* Interrogate criteria for warning */
		      
      /* If telescope is behind position on line check if telescope is within a radius */
		      if (wl2pos1 >= 0.0)
		      {
			if(sqrt(pow((wPScosfactor*wps_azsep),2.0) + pow(wps_elsep,2.0)) < wbndry)
			  fire=0;
		      }
			      
      /* If telescope is beyond pred on line check if telescope is within a radius */
		      if (wl2pos2 >= 0.0)
		      { 
			if(sqrt(pow((wPRcosfactor*wpr_azsep),2.0) + pow(wpr_elsep,2.0)) <= triw*wbndry)
			  fire=0;
		      }				  
	    
      /* If telescope is within position and pred on line check if telescope is within a band */
		      if (wl2pos1 < 0.0 && wl2pos2 < 0.0)
		      {	
			scale=sqrt(pow((wPS2cosfactor*wps_azsep2),2.0) + pow(wps_elsep2,2.0)) / sqrt(pow((wPLcosfactor*wpl_azsep),2.0) + pow(wpl_elsep,2.0));
					
			if (fabs(wR) < wbndry+(triw - 1.0)*wbndry*scale)
			  fire=0;  
		      }		     
		      
      /* Interrogate gate criteria for alarm */
				    
      /* If telescope is behind position on line check if telescope is within a radius */
		      if (al2pos1 >= 0.0)
		      {
			if(sqrt(pow((aPScosfactor*aps_azsep),2.0) + pow(aps_elsep,2.0)) < abndry)
			  fire=-1;
		      }
			      
      /* If telescope is beyond pred on line check if telescope is within a radius */
		      if (al2pos2 >= 0.0)
		      { 
			if(sqrt(pow((aPRcosfactor*apr_azsep),2.0) + pow(apr_elsep,2.0)) <= tria*abndry)
			  fire=-1;
		      }		  
	    
      /* If telescope is within position and pred on line check if telescope is within a band */
		      if (al2pos1 < 0.0 && al2pos2 < 0.0)
		      {	
			scale=sqrt(pow((aPS2cosfactor*aps_azsep2),2.0) + pow(aps_elsep2,2.0)) / sqrt(pow((aPLcosfactor*apl_azsep),2.0) + pow(apl_elsep,2.0));
					
			if (fabs(aR) < abndry+(tria - 1.0)*abndry*scale)
			  fire=-1;  
		      }
		            
      /* If warning or alarm is set, play a sound */
		      if(fabs(nowep - alarmep2) > 1.1 )
		      {
			if(fire <= 0)
			{
			  if(fire == 0)
				system("aplay -d 1 -c 1 -t wav -q sounds/198410__botjao__digital-alarm-03.wav &" ); // http://www.freesound.org/people/botjao/sounds/198410/
			  if(fire == -1)
				system("aplay -d 1 -c 1 -t wav -q sounds/170944__jan18101997__smoke-alarm-piep-piep.wav &" ); //http://www.freesound.org/people/Jan18101997/sounds/170944/
				
			  firel=fire;
			  alarmep2=nowep;
			}
			else
			  firel=1;
		      }
		    }
		    
		    e=e+1;
		      
		  }
		  
		}
	      numl=e;   
		
	      
    /* Update aircraft data buffer to screen */      
	      printf("\n\n\n\n\n\n\n\n\n");
	      printf(" ###  Telescope Position :  Azimuth = %5.1lf  /  Elevation = %4.1lf                                 %d\n", Tazim, Telev,firel);
	      printf("  #     Epoch    ident   flight#      Lat   Long  Alt ft Range km    Azim    Elev      A now +10s   E now +10s\n");		  
  
	      for(k=0;k<numl;k++)
		{
		  if(elevation[k] > Elcutoff)
		  { 
		    printf("%3d  %10s  %6s  %-8s :: %5.2lf %5.2lf %6.0lf   %5.1lf    %7.2lf   %5.2lf      %7.2lf      %5.2lf  \n",k,tstring[k],ICAO[k],flightnumber[k], latitude[k],longitude[k],altitude[k],rng[k],azimuth[k],elevation[k],wpredAz[k],wpredEl[k]);
		  }
		}
	      printf("\n");
	      waitep=nowep;
	    }
	    
	    
	sprintf(RECVbuf,"\0");
	sprintf(SNDbuf,"\0");
	sprintf(itype,"\0");
        }   
            
}

    
    /* Function STATazel() to calculate the aircraft azimuth, elevation
     * and range from the latitude, longitude and altitude of
     * the aircraft and XYZ coordinates of the telescope */
struct point STATazel(double latrad, double longrad, double altitude)
{ 
    double smjr;
    double smnr;
    double ecc;
    double Nradi;
    double X,Y,Z;
    double v[3],w[3];
    double STAT_latr,STAT_longr,STAT_alt;
    double STAT_X,STAT_Y,STAT_Z;
    double pi=3.1415926535898;
    struct point result;
         
    STAT_X=4033463.1;  // for Herstmonceux, UK
    STAT_Y=23662.8;
    STAT_Z=4924305.6;
    
    STAT_latr=2.0*pi*50.8674/360.0;   // to radians
    STAT_longr=2.0*pi*0.3361/360.0; 
    STAT_alt=75.0;   
    
    smjr=6378137.0;   
    smnr=smjr - smjr/298.257222101;
    ecc=sqrt(pow(smjr,2.0) - pow(smnr,2.0))/smjr;
    Nradi=smjr / sqrt(1 - pow(ecc,2.0) * sin(latrad) * sin(latrad));
    X=(Nradi+0.3048*altitude)*cos(latrad)*cos(longrad);
    Y=(Nradi+0.3048*altitude)*cos(latrad)*sin(longrad);
    Z=((1 - pow(ecc,2.0))*Nradi+0.3048*altitude)*sin(latrad);
    
    v[0]=X - STAT_X;
    v[1]=Y - STAT_Y;
    v[2]=Z - STAT_Z;
    w[0]=v[0]*sin(STAT_latr)*cos(STAT_longr) + v[1]*sin(STAT_latr)*sin(STAT_longr) - v[2]*cos(STAT_latr);
    w[0]= -1.0*w[0];
    w[1]= -1.0*v[0]*sin(STAT_longr) + v[1]*cos(STAT_longr);
    w[2]=v[0]*cos(STAT_latr)*cos(STAT_longr) + v[1]*cos(STAT_latr)*sin(STAT_longr) + v[2]*sin(STAT_latr);
    
    result.Rng = sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    result.Azimuth=fmod(atan2(w[1],w[0]), 2.0*pi);
    result.Azimuth=360.0*result.Azimuth/(2.0*pi);
    if(result.Azimuth < 0.0) result.Azimuth = result.Azimuth+360.0;
    
    result.Elevation=atan2(w[2], sqrt(w[0]*w[0] + w[1]*w[1]));
    result.Elevation=360.0*result.Elevation/(2.0*pi);
     
    return result;
}

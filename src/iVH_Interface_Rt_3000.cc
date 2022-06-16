/* ===================================================
 *  file:       iVH_Interface.hh
 * ---------------------------------------------------
 *  purpose:	the plugin of the perfect sensor
 * ---------------------------------------------------
 *  first edit:	01.02.2016 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
/* ====== INCLUSIONS ====== */
#include <iostream>
#include "iVH_Interface_Rt_3000.hh"
#include "SensorIface.hh"
#include "DynamicsIface.hh"
#include "ScpParser.hh"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include "RDBHandler.hh"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/shm.h>
#include <chrono>
#include <thread>
// #include <OpenDRIVE.hh>
// #include "OdrManager.hh"
// #include "OdrJuncController.hh"
#define DEFAULT_PORT        48190  
#define DEFAULT_PORT6        8080   /* for GPS */
#define DEFAULT_BUFFER_SIZE 204800

// using Eigen::MatrixXd;

std::string szServer("127.0.0.1");

// pthread_t tid;
// struct sockaddr_in cliaddr;
// int cliaddr_len;

// actual class declaration for access via shared mechanisms
// is hidden behind this little function; make sure, this
// function is included in all data accesss libraries
// @todo: make this a macro
extern "C"
{

    Framework::Plugin *
    makeModule()
    {
        return new Module::iVH_Interface();
    }
} // extern "C"

namespace Module
{
    iVH_Interface::iVH_Interface() : mLastFrameNo(0),
                                     mFrameCounter(0),
                                     mOutSocketTCP(-1),
                                     mOutSocketUDP(-1),
                                     mOutSocketUDP2(-1),
                                     EgoID(0)
    {
        // std::cerr << "iVH_Interface: CTOR called, this=" << this << std::endl;
        pIniVehicle = new vehicleInitialInformation;
    }

    iVH_Interface::~iVH_Interface()
    {
        close(mOutSocketTCP);
        close(mOutSocketUDP);
        // std::cerr << "iVH_Interface: DTOR called, this=" << this << std::endl;
    }

    bool
    iVH_Interface::init()
    {
        fprintf(stderr, "iVH_Interface::init: called\n");
        createComSocketTCP();
        createComSocketUDP_GPS();

        // cliaddr_len = sizeof(cliaddr);
        // pthread_create(&tid,NULL,udpdriversend,&mOutSocketUDP);
        
        // createComSocketUDP2();
        return true;
    }

    bool
    iVH_Interface::reset(const double &simTime)
    {
        // first call reset routine of base class
        SensorPlugin::reset(simTime);
        return true;
    }

    int
    iVH_Interface::update(const unsigned long &frameNo, SensorIface *ifaceData)
    {
        // RT 3000(UDP)에서 나온 데이터 Recv하는 코드

        if (ifaceData->mSimTime == 0)
        {
            getInitialPos(ifaceData);
        }

        //Set a driver control
        if (!ifaceData || (EgoID < 0))
        {
            // fprintf(stderr,"2 \n");
            return 0;
        }

        if (!mNewFrame)
        {
            // fprintf(stderr,"3 \n");
            return 0;
        }
        
        ellipsoidData elips;
        RT_3000 rt_3000;
        memset(&rt_3000, 0, sizeof(RT_3000));

        utmData utm;
        wgsData wgsB;
        iVH_Interface module;

        // - RDB_OBJECT_STATE_t *objectlist_treeze = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));

        memset(objectlist_treeze, 0, sizeof(RDB_OBJECT_STATE_t));

        struct sockaddr_in cliaddr;
        memset(&cliaddr, 0, sizeof(cliaddr));

        int cliaddr_len;
        cliaddr_len = sizeof(sockaddr_in);

        ssize_t retval = recvfrom(mOutSocketUDP, (RT_3000 *)&rt_3000, sizeof(RT_3000), MSG_WAITALL, (struct sockaddr *)&cliaddr, (socklen_t *)&cliaddr_len);
        if (retval == -1)
        {
            perror("receive failed");
            exit(EXIT_FAILURE);
        }

        // fprintf(stderr,"====================== Before 16 hex Receive from RT 3000 ======================\n");
        // fprintf(stderr, "Sync : %08x\n", rt_3000.Sync);
        // fprintf(stderr, "time : %08x\n", rt_3000.time); // Time is transmitted as milliseconds into the current GPS minute. The range is 0–59,999 ms.
        // fprintf(stderr, "acc_x : %08x\n", rt_3000.acc_x);
        // fprintf(stderr, "acc_y : %08x\n", rt_3000.acc_y);
        // fprintf(stderr, "acc_z : %08x\n", rt_3000.acc_z);
        // fprintf(stderr, "ang_x : %08x\n", rt_3000.ang_x);
        // fprintf(stderr, "ang_y : %08x\n", rt_3000.ang_y);
        // fprintf(stderr, "ang_z : %08x\n", rt_3000.ang_z);
        // fprintf(stderr, "GPS_latitude : ");
        // for (int i = 0; i < 8; i++)
        // {
        //     fprintf(stderr, "%x", latitude[i]);
        // }
        // fprintf(stderr, "\n");
        // fprintf(stderr, "GPS_longitude : ");
        // for (int i = 0; i < 8; i++)
        // {
        //     fprintf(stderr,"%x", longitude[i]);
        // }
        // fprintf(stderr, "\n");
        // fprintf(stderr, "Nav_status : %08x\n", rt_3000.Nav_status);
        // fprintf(stderr, "checksum1 : %08x\n", rt_3000.checksum1);
        // fprintf(stderr, "Altitude : %08x\n", *(unsigned int*)&rt_3000.Altitude);
        // fprintf(stderr, "North_vel : %08x\n", rt_3000.North_vel);
        // fprintf(stderr, "East_vel : %08x\n", rt_3000.East_vel);
        // fprintf(stderr, "Down_vel : %08x\n", rt_3000.Down_vel);
        // fprintf(stderr, "Heading : %08x\n", rt_3000.Heading);
        // fprintf(stderr, "pitch : %08x\n", rt_3000.pitch);
        // fprintf(stderr, "Roll : %08x\n", rt_3000.Roll);
        // fprintf(stderr, "checksum2 : %08x\n", rt_3000.checksum2);
        // fprintf(stderr, "status_channel : %08x\n", rt_3000.status_channel);
        // fprintf(stderr, "checksum3 : %08x\n", rt_3000.checksum3);
        // fprintf(stderr,"===================================================================\n");

        // fprintf(stderr,"====================== Receive from RT 3000 ======================\n");
        // fprintf(stderr, "Sync : %c\n", rt_3000.Sync);
        // fprintf(stderr, "time : %d\n", rt_3000.time); // Time is transmitted as milliseconds into the current GPS minute. The range is 0–59,999 ms.
        // fprintf(stderr, "acc_x : %d\n", rt_3000.acc_x);
        // fprintf(stderr, "acc_y : %d\n", rt_3000.acc_y);
        // fprintf(stderr, "acc_z : %d\n", rt_3000.acc_z);
        // fprintf(stderr, "ang_x : %d\n", rt_3000.ang_x);
        // fprintf(stderr, "ang_y : %d\n", rt_3000.ang_y);
        // fprintf(stderr, "ang_z : %d\n", rt_3000.ang_z);
        // fprintf(stderr, "GPS_latitude : %lf\n", rt_3000.GPS_latitude);
        // fprintf(stderr, "GPS_longitude : %lf\n", rt_3000.GPS_longitude);
        // fprintf(stderr, "Nav_status : %c\n", rt_3000.Nav_status);
        // fprintf(stderr, "checksum1 : %c\n", rt_3000.checksum1);
        // fprintf(stderr, "Altitude : %f\n", rt_3000.Altitude);
        // fprintf(stderr, "North_vel : %d\n", rt_3000.North_vel);
        // fprintf(stderr, "East_vel : %d\n", rt_3000.East_vel);
        // fprintf(stderr, "Down_vel : %d\n", rt_3000.Down_vel);
        // fprintf(stderr, "Heading : %d\n", rt_3000.Heading);
        // fprintf(stderr, "pitch : %d\n", rt_3000.pitch);
        // fprintf(stderr, "Roll : %d\n", rt_3000.Roll);
        // fprintf(stderr, "checksum2 : %c\n", rt_3000.checksum2);
        // fprintf(stderr, "status_channel : %c\n", rt_3000.status_channel);
        // fprintf(stderr, "checksum3 : %c\n", rt_3000.checksum3);
        // fprintf(stderr,"===================================================================\n");
        
        int ellipsoidModelID = 22;

        ellipsoidDB(ellipsoidModelID, elips);

        wgsB.latitude = rtod(rt_3000.GPS_latitude);
        wgsB.longitude = rtod(rt_3000.GPS_longitude);

        module.WGStoUTM(elips, wgsB, utm);

        // for TrafficDemo.xml
        // x : 7419.655
        // y : -3108.481

        // for kcity_scenario_treeze.xml
        // x : -14.708
        // y : -32.917

        objectlist_treeze->base.id = EgoID;
        objectlist_treeze->base.category = RDB_OBJECT_CATEGORY_PLAYER;
        objectlist_treeze->base.type = RDB_OBJECT_TYPE_PLAYER_CAR;
        // objectlist_treeze->base.pos.x = utm.Easting + pIniVehicle->initialPosition.x - double(333103.826788);// - double(302492.652312);// - double(295058.248312); // + pIniVehicle->initialPosition.x;
        // fprintf(stderr,"pos_x : %f\n", utm.Easting + pIniVehicle->initialPosition.x - double(333103.826788)); // - double(295058.248312));
        // objectlist_treeze->base.pos.y = utm.Northing + pIniVehicle->initialPosition.y - double(4118148.188060);// - double(4126823.295782); // + pIniVehicle->initialPosition.y;
        // fprintf(stderr,"pos_y : %f\n",utm.Northing + pIniVehicle->initialPosition.y - double(4118148.188060));// - double(4126823.295782));
        objectlist_treeze->base.pos.x = utm.Easting + pIniVehicle->initialPosition.x - double(399862.810644);       // - double(333103.826788);// - double(302492.652312);// - double(295058.248312); // + pIniVehicle->initialPosition.x;
        fprintf(stderr, "pos_x : %f\n", utm.Easting + pIniVehicle->initialPosition.x - double(399862.810644));   // - double(333103.826788)); // - double(295058.248312));
        objectlist_treeze->base.pos.y = utm.Northing + pIniVehicle->initialPosition.y - double(4092210.531804);     // - double(4118148.188060);// - double(4126823.295782); // + pIniVehicle->initialPosition.y;
        fprintf(stderr, "pos_y : %f\n", utm.Northing + pIniVehicle->initialPosition.y - double(4092210.531804)); // - double(4123747.983782));// - double(4126823.295782));
        objectlist_treeze->base.pos.z = pIniVehicle->initialPosition.z;

        objectlist_treeze->base.pos.h = float(rt_3000.Heading) * 0.000001f;      //yaw
        fprintf(stderr, "pos_h : %lf\n", float(rt_3000.Heading) * 0.000001f); // rt_3000.Heading);
        objectlist_treeze->base.pos.p = 0.0;                                      //pitch
        objectlist_treeze->base.pos.r = 0.0;                                      //roll
        objectlist_treeze->base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;

        objectlist_treeze->ext.speed.x = 0.0;
        objectlist_treeze->ext.speed.y = 0.0;
        objectlist_treeze->ext.speed.z = 0.0;
        objectlist_treeze->ext.speed.h = float(rt_3000.Heading) * 0.0001f;
        objectlist_treeze->ext.speed.p = 0.0;
        objectlist_treeze->ext.speed.r = 0.0;
        objectlist_treeze->ext.speed.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;

        objectlist_treeze->ext.accel.x = double(rt_3000.acc_x) * 0.0001;
        objectlist_treeze->ext.accel.y = double(rt_3000.acc_y) * 0.0001;
        objectlist_treeze->ext.accel.z = 0.0;
        // objectlist_treeze->ext.accel.h = float(rt_3000.ang_z) * 0.001f;
        objectlist_treeze->ext.accel.h = (float(rt_3000.Heading) * 0.0001f - prev_Heading) * 100;
        objectlist_treeze->ext.accel.p = 0.0;
        objectlist_treeze->ext.accel.r = 0.0;
        objectlist_treeze->ext.accel.flags = RDB_COORD_FLAG_POINT_VALID;

        objectlist_treeze->base.visMask = RDB_OBJECT_VIS_FLAG_TRAFFIC | RDB_OBJECT_VIS_FLAG_RECORDER;

        prev_Heading = float(rt_3000.Heading) * 0.0001f;

        sendOwnObjectState(mOutSocketTCP, ifaceData->mSimTime, static_cast<unsigned int>(ifaceData->mFrameNo), objectlist_treeze);

        fprintf(stderr, "\n------- RT 3000 int Data ------\n");
        fprintf(stderr, "int data - Sync: %c\n", rt_3000.Sync);
        fprintf(stderr, "int data - time: %d\n", rt_3000.time);
        fprintf(stderr, "int data - acc_x: %f\n", rt_3000.acc_x * 0.0001);
        fprintf(stderr, "int data - acc_y: %f\n", rt_3000.acc_y * 0.0001);
        fprintf(stderr, "int data - acc_z: %f\n", rt_3000.acc_z * 0.0001);
        fprintf(stderr, "int data - ang_x: %f\n", rt_3000.ang_x * 0.00001);
        fprintf(stderr, "int data - ang_y: %f\n", rt_3000.ang_y * 0.00001);
        fprintf(stderr, "int data - ang_z: %f\n", rt_3000.ang_z * 0.00001);
        fprintf(stderr, "int data - Nav_status: %c\n", rt_3000.Nav_status);
        fprintf(stderr, "int data - checksum1: %c\n", rt_3000.checksum1);
        fprintf(stderr, "int data - GPS_latitude: %f\n", rt_3000.GPS_latitude);
        fprintf(stderr, "int data - GPS_longitude: %f\n", rt_3000.GPS_longitude);
        fprintf(stderr, "int data - Altitude: %lf\n", rt_3000.Altitude);
        fprintf(stderr, "int data - North_vel: %f\n", rt_3000.North_vel * 0.0001);
        fprintf(stderr, "int data - East_vel: %f\n", rt_3000.East_vel * 0.0001);
        fprintf(stderr, "int data - Down_vel: %f\n", rt_3000.Down_vel * 0.0001);
        // printf("\tint data - Heading: %f\n", rtodf(float(Heading_int) * 0.000001));
        fprintf(stderr, "int data - Heading: %f\n", rt_3000.Heading * 0.000001);
        fprintf(stderr, "int data - pitch: %f\n", rt_3000.pitch * 0.000001);
        fprintf(stderr, "int data - Roll: %f\n", rt_3000.Roll * 0.000001);
        fprintf(stderr, "int data - checksum2: %c\n", rt_3000.checksum2);
        fprintf(stderr, "int data - status_channel: %c\n", rt_3000.status_channel);
        fprintf(stderr, "int data - Batch_S: %f\n", rt_3000.Batch_S);
        fprintf(stderr, "int data - checksum3: %c\n", rt_3000.checksum3);



        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*
            UDP recv to VTD send 교육용 코드

        if (ifaceData->mSimTime == 0)
        {
            getInitialPos(ifaceData);
        }

        //Set a driver control
        if (!ifaceData || (EgoID < 0))
        {
            // fprintf(stderr,"2 \n");
            return 0;
        }

        if (!mNewFrame)
        {
            // fprintf(stderr,"3 \n");
            return 0;
        }
        
        ellipsoidData elips;
        tEgoResponse wonwoo;
        memset(&wonwoo, 0, sizeof(tEgoResponse));

        utmData utm;
        wgsData wgsB;
        iVH_Interface module;

        RDB_OBJECT_STATE_t *objectlist_wonwoo = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));

        memset(objectlist_wonwoo, 0, sizeof(RDB_OBJECT_STATE_t));

        struct sockaddr_in cliaddr;
        memset(&cliaddr, 0, sizeof(cliaddr));

        int cliaddr_len;
        cliaddr_len = sizeof(sockaddr_in);

        ssize_t retval = recvfrom(mOutSocketUDP, (tEgoResponse *)&wonwoo, sizeof(tEgoResponse), MSG_WAITALL, (struct sockaddr *)&cliaddr, (socklen_t *)&cliaddr_len);
        if (retval == -1)
        {
            perror("receive failed");
            exit(EXIT_FAILURE);
        }
        
        int ellipsoidModelID = 22;

        ellipsoidDB(ellipsoidModelID, elips);

        wgsB.latitude = wonwoo.GPS_latiutde;
        wgsB.longitude = wonwoo.GPS_longitude;

        module.WGStoUTM(elips, wgsB, utm);

        fprintf(stderr, "WGS to UTM converted.\n");

        objectlist_wonwoo->base.id = EgoID;
        objectlist_wonwoo->base.category = RDB_OBJECT_CATEGORY_PLAYER;
        objectlist_wonwoo->base.type = RDB_OBJECT_TYPE_PLAYER_CAR;
        objectlist_wonwoo->base.pos.x = utm.Easting + pIniVehicle->initialPosition.x - double(333103.826788);
        fprintf(stderr,"pos_x : %lf\n",utm.Easting);
        objectlist_wonwoo->base.pos.y = utm.Northing + pIniVehicle->initialPosition.y - double(4118148.188060);
        fprintf(stderr,"pos_x : %lf\n",utm.Northing);
        objectlist_wonwoo->base.pos.z =  pIniVehicle->initialPosition.z;

        objectlist_wonwoo->base.pos.h = float(1.57-dtor(wonwoo.GPS_azimuth)); //yaw
        fprintf(stderr,"pos_x : %f\n",float(1.57-dtor(wonwoo.GPS_azimuth)));
        objectlist_wonwoo->base.pos.p = 0.0; //pitch
        objectlist_wonwoo->base.pos.r = 0.0; //roll
        objectlist_wonwoo->base.pos.flags = RDB_COORD_FLAG_POINT_VALID | RDB_COORD_FLAG_ANGLES_VALID;
        objectlist_wonwoo->base.visMask = RDB_OBJECT_VIS_FLAG_TRAFFIC | RDB_OBJECT_VIS_FLAG_RECORDER;

        objectlist_wonwoo->ext.accel.x = wonwoo.INS_x_acc;
        objectlist_wonwoo->ext.accel.y = wonwoo.INS_y_acc;
        objectlist_wonwoo->ext.accel.z = wonwoo.INS_z_acc;
        objectlist_wonwoo->ext.accel.h = float(dtor(wonwoo.INS_yaw_acc));
        objectlist_wonwoo->ext.accel.p = float(dtor(wonwoo.INS_pitch_acc));
        objectlist_wonwoo->ext.accel.r = float(dtor(wonwoo.INS_roll_acc));
        objectlist_wonwoo->ext.accel.flags = RDB_COORD_FLAG_POINT_VALID;

        sendOwnObjectState(mOutSocketTCP, ifaceData->mSimTime, static_cast<unsigned int>(ifaceData->mFrameNo), objectlist_wonwoo);
        */

        dummyTCrecv(mOutSocketTCP);

        free(objectlist_treeze);

        mLastFrameNo = mFrameNo;

        // for thread sleep
        // int t = rt_3000.time;

        // int64_t ms = 10;
        
        // if (prev_t > 0)
        // {
        //     // ms = t - prev_t;
        //     // if(prev_t > t)
        //     // {
        //     //     ms = 65535-prev_t+t;
        //     //}
        //     // int64_t ms = 3000;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        // }
        // prev_t = t;
        
        return 1;
    }

    void
    iVH_Interface::createComSocketUDP_GPS()
    {
        struct sockaddr_in servaddr; 

        // Creating socket file descriptor 
        if ( (mOutSocketUDP = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket creation failed"); 
            exit(EXIT_FAILURE); 
        } 

        memset(&servaddr, 0, sizeof(servaddr)); 

        // Filling server information 
        servaddr.sin_family    = AF_INET; // IPv4 
        servaddr.sin_addr.s_addr = INADDR_BROADCAST;
        servaddr.sin_port = htons(3000);

        char netif[] = "enp0s31f6";
        // char netif[] = "enp4s0";
        // char netif[] = "eth0";
        // int netif = 1;
        setsockopt(mOutSocketUDP, SOL_SOCKET, SO_BINDTODEVICE, &netif, sizeof(netif));

        if ( bind(mOutSocketUDP, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 )
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        fprintf(stderr,"Connect!\n");

        /*
        기존 코드
        if (mOutSocketUDP >= 0)
            return;

        mOutSocketUDP = socket(AF_INET, SOCK_DGRAM, 0);

        fprintf(stderr, "Module::createComSocket: mOutSocketUDP=%d\n", mOutSocketUDP);

        if (mOutSocketUDP == -1)
        {
            fprintf(stderr, "Module::createComSocket: could not open socket\n");
            return;
        }

        struct sockaddr_in udp_received_addr;
        memset(&udp_received_addr, 0, sizeof(udp_received_addr));

        udp_received_addr.sin_family = AF_INET;
        udp_received_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // target address (RT 3000)
        udp_received_addr.sin_port = htons(3000);

        if (bind(mOutSocketUDP, (const struct sockaddr *)&udp_received_addr, sizeof(udp_received_addr)) < 0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        */

        /////////////////////////////////////////// ROS 통신 테스트 ////////////////////////////////////////////////////
        /*

        if (mOutSocketUDP >= 0)
            return;

        mOutSocketUDP = socket(AF_INET, SOCK_DGRAM, 0);

        fprintf(stderr, "Module::createComSocket: mOutSocketUDP=%d\n", mOutSocketUDP);

        if (mOutSocketUDP == -1)
        {
            fprintf(stderr, "Module::createComSocket: could not open socket\n");
            return;
        }

        int opt = 1;

        if (setsockopt(mOutSocketUDP, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) == -1)
            fprintf(stderr, "Module::createComSocket: could not set port to reuse\n");
        
        */
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    void
    iVH_Interface::createComSocketUDP2()
    {
        // already created?
        if (mOutSocketUDP2 >= 0)
            return;

        mOutSocketUDP2 = socket(AF_INET, SOCK_DGRAM, 0);

        fprintf(stderr, "Module::createComSocket: mOutSocketUDP=%d\n", mOutSocketUDP2);

        if (mOutSocketUDP2 == -1)
        {
            fprintf(stderr, "Module::createComSocket: could not open socket\n");
            return;
        }

        // int opt = 1;

        // if (setsockopt(mOutSocketUDP2, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) == -1)
        //     fprintf(stderr, "Module::createComSocket: could not set port to reuse\n");

        struct sockaddr_in udp_received_addr;
        memset(&udp_received_addr, 0, sizeof(udp_received_addr));

        udp_received_addr.sin_family = AF_INET;
        udp_received_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        udp_received_addr.sin_port = htons(48190);

        if (bind(mOutSocketUDP2, (const struct sockaddr *)&udp_received_addr, sizeof(udp_received_addr)) < 0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
    }

    bool
    iVH_Interface::handleSCPElement(Framework::ScpParser *parser)
    {
        if (!parser)
            return false;

        bool handledByBaseClass = SensorPlugin::handleSCPElement(parser);

        switch (parser->getCurrentOpcode())
        {
        case Framework::ScpParser::ocConfig:
            if (parser->hasAttribute("EgoID"))
            {
                EgoID = parser->getInt("EgoID");
                fprintf(stderr, "iVH_Interface::handleSCPElement: Ego ID is %d \n", EgoID);
            }

            break;

        case Framework::ScpParser::ocStart:
            fprintf(stderr, "iVH_Interface::handleSCPElement: got start command\n");
            break;

        case Framework::ScpParser::ocStop:
            fprintf(stderr, "iVH_Interface::handleSCPElement: got stop command\n");
            break;

        default:
            return handledByBaseClass;
            break;
        }
        return true;
    }

    void
    iVH_Interface::createComSocketTCP()
    {
        struct sockaddr_in server;
        struct hostent *host = NULL;

        // already created?
        if (mOutSocketTCP >= 0)
            return;

        mOutSocketTCP = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        fprintf(stderr, "Module::createComSocket: mOutSocketTCP=%d\n", mOutSocketTCP);

        if (mOutSocketTCP == -1)
        {
            fprintf(stderr, "Module::createComSocket: could not open socket\n");
            return;
        }

        int opt = 1; //////////
        setsockopt(mOutSocketTCP, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)); //////////

        server.sin_family = AF_INET;
        server.sin_port = htons(DEFAULT_PORT);
        server.sin_addr.s_addr = inet_addr(szServer.c_str());

        if (server.sin_addr.s_addr == INADDR_NONE)
        {
            host = gethostbyname("127.0.0.1");
            if (host == NULL)
            {
                fprintf(stderr, "Unable to resolve server: %s\n", "127.0.0.1");
                return;
            }
            memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);
        }
        // wait for connection
        bool bConnected = false;

        while (!bConnected)
        {
            if (connect(mOutSocketTCP, (struct sockaddr *)&server, sizeof(server)) == -1)
            {
                fprintf(stderr, "connect() failed: %s\n", strerror(errno));
                sleep(1);
            }
            else
                bConnected = true;
        }

        fprintf(stderr, "connected!\n");
    }

    void
    iVH_Interface::dummyTCrecv(int &sendSocket)
    {
        unsigned int bytesInBuffer = 0;
        size_t bufferSize = sizeof(RDB_MSG_HDR_t);
        // unsigned int  count         = 0;
        unsigned char *pData = (unsigned char *)calloc(1, bufferSize);
        bool bMsgComplete = false;
        char *szBuffer = new char[DEFAULT_BUFFER_SIZE]; // allocate on heap

        // read a complete message
        while (!bMsgComplete)
        {
            //if ( sSendTrigger && !( count++ % 1000 ) )
            //  sendTrigger( sClient, 0, 0 );

            ssize_t ret = recv(sendSocket, szBuffer, DEFAULT_BUFFER_SIZE, 0);
            int tempRet = (int)ret;
            if (tempRet == -1)
            {
                printf("recv() failed: %s\n", strerror(errno));
                break;
            }

            if (tempRet != 0)
            {
                // do we have to grow the buffer??
                if ((bytesInBuffer + tempRet) > bufferSize)
                {
                    pData = (unsigned char *)realloc(pData, bytesInBuffer + tempRet);
                    bufferSize = bytesInBuffer + tempRet;
                }

                memcpy(pData + bytesInBuffer, szBuffer, tempRet);
                bytesInBuffer += tempRet;

                // already complete messagae?
                if (bytesInBuffer >= sizeof(RDB_MSG_HDR_t))
                {
                    RDB_MSG_HDR_t *hdr = (RDB_MSG_HDR_t *)pData;

                    // is this message containing the valid magic number?
                    if (hdr->magicNo != RDB_MAGIC_NO)
                    {
                        printf("message receiving is out of sync; discarding data");
                        bytesInBuffer = 0;
                    }

                    while (bytesInBuffer >= (hdr->headerSize + hdr->dataSize))
                    {
                        unsigned int msgSize = hdr->headerSize + hdr->dataSize;
                        bool         isImage = false;

                        // print the message
                        //if ( sVerbose )
                        //    Framework::RDBHandler::printMessage( ( RDB_MSG_t* ) pData, true );

                        // now parse the message
                        parseRDBMessage( ( RDB_MSG_t* ) pData, isImage );

                        // remove message from queue
                        memmove( pData, pData + msgSize, bytesInBuffer - msgSize );
                        bytesInBuffer -= msgSize;

                        bMsgComplete = true;
                    }
                }
            }
        }
    } 

    void 
    iVH_Interface::parseRDBMessage( RDB_MSG_t* msg, bool & isImage )
    {
        if ( !msg )
        return;

        if ( !msg->hdr.dataSize )
            return;
        
        RDB_MSG_ENTRY_HDR_t* entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( char* ) msg ) + msg->hdr.headerSize );
        uint32_t remainingBytes    = msg->hdr.dataSize;
            
        while ( remainingBytes )
        {
            parseRDBMessageEntry( msg->hdr.simTime, msg->hdr.frameNo, entry );

            // fprintf(stderr,"data_size : %lu\n",sizeof(entry->dataSize));

            isImage |= ( entry->pkgId == RDB_PKG_ID_IMAGE );

            remainingBytes -= ( entry->headerSize + entry->dataSize );
            
            if ( remainingBytes )
            entry = ( RDB_MSG_ENTRY_HDR_t* ) ( ( ( ( char* ) entry ) + entry->headerSize + entry->dataSize ) );
        }
    }



    void 
    iVH_Interface::parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr )
    {
        if ( !entryHdr )
            return;
        
        int noElements = entryHdr->elementSize ? ( entryHdr->dataSize / entryHdr->elementSize ) : 0;
        
        if ( !noElements )  // some elements require special treatment
        {
            switch ( entryHdr->pkgId )
            {
                case RDB_PKG_ID_START_OF_FRAME:
                    // fprintf( stderr, "void parseRDBMessageEntry: got start of frame\n" );
                    break;
                    
                case RDB_PKG_ID_END_OF_FRAME:
                    // fprintf( stderr, "void parseRDBMessageEntry: got end of frame\n" );
                    break;
                    
                default:
                    return;
                    break;
            }
            return;
        }

        // unsigned char ident   = 1;

        char*         dataPtr = ( char* ) entryHdr;
            
        dataPtr += entryHdr->headerSize;
            
        while ( noElements-- )
        {
            // bool printedMsg = true;
                
            switch ( entryHdr->pkgId )
            {               
                // case RDB_PKG_ID_ROADMARK:
                //     handleRDBitem( simTime, simFrame, *( ( RDB_ROADMARK_t* ) dataPtr ), ident );
                //     break;
                                     
                case RDB_PKG_ID_OBJECT_STATE:
                    handleRDBitem( simTime, simFrame, *( ( RDB_OBJECT_STATE_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, noElements );
                    // fprintf(stderr,"noElement : %d\n", noElements);
                    break;

                // case RDB_PKG_ID_SENSOR_OBJECT:
                //     handleRDBitem( simTime, simFrame, *( ( RDB_SENSOR_OBJECT_t* ) dataPtr ), ident , noElements );
                //     // fprintf(stderr,"noElement : %d\n", noElements);
                //     break;                       
                        
                // case RDB_PKG_ID_TRAFFIC_SIGN:
                //     handleRDBitem( simTime, simFrame, *( ( RDB_TRAFFIC_SIGN_t* ) dataPtr ), ident );
                //     break;
                        
                // case RDB_PKG_ID_ENVIRONMENT:
                //     handleRDBitem( simTime, simFrame, *( ( RDB_ENVIRONMENT_t* ) dataPtr ), ident );
                //     break;
                        
                // case RDB_PKG_ID_TRAFFIC_LIGHT:
                //     handleRDBitem( simTime, simFrame, *( ( RDB_TRAFFIC_LIGHT_t* ) dataPtr ), entryHdr->flags & RDB_PKG_FLAG_EXTENDED, ident );
                //     break;
                        
                // case RDB_PKG_ID_DRIVER_PERCEPTION:
                //     handleRDBitem( simTime, simFrame, *( ( RDB_DRIVER_PERCEPTION_t* ) dataPtr ), ident );
                //     break;
            }
            dataPtr += entryHdr->elementSize;                
        }
    }

    void 
    iVH_Interface::handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_OBJECT_STATE_t & item, bool isExtended, int noElements )
    {
        memset(objectlist, 0, sizeof(RDB_OBJECT_STATE_t));

        if (item.base.id == (uint32_t)EgoID)
        {
            prevobjectlist = &item;
            objectlist = &item;

            // fprintf(stderr,"====================== before UTM Convert ======================\n");
            // fprintf(stderr,"base_pos_X : %f\n", prevobjectlist->base.pos.x);
            // fprintf(stderr,"base_pos_Y : %f\n", prevobjectlist->base.pos.y);
            // fprintf(stderr,"===================================================================\n");

            // fprintf(stderr,"===================================================================\n");
            // fprintf(stderr,"id : %d\n", item.base.id);
            // fprintf(stderr,"geo_dimX : %f\n", prevobjectlist->base.geo.dimX);
            // fprintf(stderr,"geo_dimY : %f\n", prevobjectlist->base.geo.dimY);
            // fprintf(stderr,"geo_offX : %f\n", prevobjectlist->base.geo.offX);
            // fprintf(stderr,"geo_offY : %f\n", prevobjectlist->base.geo.offY);
            // fprintf(stderr,"pos_X : %f\n", prevobjectlist->base.pos.x);
            // fprintf(stderr,"pos_Y : %f\n", prevobjectlist->base.pos.y);
            // fprintf(stderr,"-------------------------------------------------------------------\n");
            // fprintf(stderr, "speed_pos_x : %f\n",prevobjectlist->ext.speed.x);   // => 속력구하는 공식 (x,y,z축) : 루트 x^2 + y^2 + z^2
            // fprintf(stderr, "speed_pos_y : %f\n",prevobjectlist->ext.speed.y);
            // fprintf(stderr, "speed_pos_z : %f\n",prevobjectlist->ext.speed.z);
            // fprintf(stderr, "heading angle : %f\n",prevobjectlist->ext.speed.h);
            // fprintf(stderr, "pitch angle : %f\n",prevobjectlist->ext.speed.p);
            // fprintf(stderr, "roll angle : %f\n",prevobjectlist->ext.speed.r);
            // fprintf(stderr,"-------------------------------------------------------------------\n");
            // fprintf(stderr, "accel_pos_x : %f\n",prevobjectlist->ext.accel.x);
            // fprintf(stderr, "accel_pos_y : %f\n",prevobjectlist->ext.accel.y);
            // fprintf(stderr, "accel_pos_z : %f\n",prevobjectlist->ext.accel.z);
            // fprintf(stderr, "heading angle : %f\n",prevobjectlist->ext.accel.h);
            // fprintf(stderr, "pitch angle : %f\n",prevobjectlist->ext.accel.p);
            // fprintf(stderr, "roll angle : %f\n",prevobjectlist->ext.accel.r);
            // fprintf(stderr,"-------------------------------------------------------------------\n");
            // fprintf(stderr, "EngineSpeed : %lf\n",sqrt((prevobjectlist->ext.speed.x * prevobjectlist->ext.speed.x) + (prevobjectlist->ext.speed.y * prevobjectlist->ext.speed.y) + (prevobjectlist->ext.speed.z * prevobjectlist->ext.speed.z)));
        }
    }

    void
    iVH_Interface::WGStoUTM(ellipsoidData ellip, wgsData wgs, utmData& utm)
    {
    // Lat and Long are in degrees;  North latitudes and East Longitudes are positive.
    double a = ellip.EquatorialRadius;
    double ee = 2/ellip.inverseFlattening-1/(ellip.inverseFlattening*ellip.inverseFlattening);
    wgs.longitude -= int((wgs.longitude+180)/360)*360;			//ensure longitude within -180.00..179.9
    double N, T, C, A, M;
    double LatRad = wgs.latitude*deg2rad;
    double LongRad = wgs.longitude*deg2rad;

    utm.Zone = int((wgs.longitude + 186)/6);
    if( wgs.latitude >= 56.0 && wgs.latitude < 64.0 && wgs.longitude >= 3.0 && wgs.longitude < 12.0 )  utm.Zone  = 32;
    if( wgs.latitude >= 72.0 && wgs.latitude < 84.0 ){			//Special zones for Svalbard
        if(      wgs.longitude >= 0.0  && wgs.longitude <  9.0 )  utm.Zone  = 31;
        else if( wgs.longitude >= 9.0  && wgs.longitude < 21.0 )  utm.Zone  = 33;
        else if( wgs.longitude >= 21.0 && wgs.longitude < 33.0 )  utm.Zone  = 35;
        else if( wgs.longitude >= 33.0 && wgs.longitude < 42.0 )  utm.Zone  = 37;
    }
    double LongOrigin = utm.Zone *6 - 183;			//origin in middle of zone
    double LongOriginRad = LongOrigin * deg2rad;

    double EE = ee/(1-ee);

    N = a/sqrt(1-ee*sin(LatRad)*sin(LatRad));
    T = tan(LatRad)*tan(LatRad);
    C = EE*cos(LatRad)*cos(LatRad);
    A = cos(LatRad)*(LongRad-LongOriginRad);

    M= a*((1 - ee/4    - 3*ee*ee/64 - 5*ee*ee*ee/256  ) *LatRad 
            - (3*ee/8 + 3*ee*ee/32 + 45*ee*ee*ee/1024) *sin(2*LatRad)
            + (15*ee*ee/256 + 45*ee*ee*ee/1024	  ) *sin(4*LatRad)
            - (35*ee*ee*ee/3072			  ) *sin(6*LatRad));
    
    utm.Easting = k0*N*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*EE)*A*A*A*A*A/120) + 500000.0;

    utm.Northing = k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                    + (61-58*T+T*T+600*C-330*EE)*A*A*A*A*A*A/720));
    }

    void 
    iVH_Interface::ellipsoidDB(int id, ellipsoidData& ellipD )
    {
        ellipsoidData ellip[23] = {
            { 0, "Airy1830",		    6377563.396,	299.3249646},
            { 1, "AiryModified",		6377340.189,	299.3249646},
            { 2, "AustralianNational",	6378160,	    298.25},
            { 3, "Bessel1841Namibia",	6377483.865,	299.1528128},
            { 4, "Bessel1841",		    6377397.155,	299.1528128},
            { 5, "Clarke1866",		    6378206.4,	    294.9786982},
            { 6, "Clarke1880",		    6378249.145,	293.465},
            { 7, "EverestIndia1830",	6377276.345,	300.8017},
            { 8, "EverestSabahSarawak",	6377298.556,	300.8017},
            { 9, "EverestIndia1956",	6377301.243,	300.8017},
            {10, "EverestMalaysia1969",	6377295.664,	300.8017},
            {11, "EverestMalay_Sing",	6377304.063,	300.8017},
            {12, "EverestPakistan",	    6377309.613,	300.8017},
            {13, "Fischer1960Modified",	6378155,	    298.3},
            {14, "Helmert1906",		    6378200,	    298.3},
            {15, "Hough1960",		    6378270,	    297},
            {16, "Indonesian1974",		6378160,	    298.247},
            {17, "International1924",	6378388,	    297},
            {18, "Krassovsky1940",		6378245,	    298.3},
            {19, "GRS80",			    6378137,	    298.257222101},
            {20, "SouthAmerican1969",	6378160,	    298.25},
            {21, "WGS72",			    6378135,	    298.26},
            {22, "WGS84",			    6378137,	    298.257223563}};

        ellipD = ellip[id];
    }

    double 
    iVH_Interface::dtor(double deg)
    {
        return deg/(180/PI);
    }

    double 
    iVH_Interface::rtod(double rad)
    {
        return rad*(180/PI);
    }

    void
    iVH_Interface::triggerFunc(int &sendSocket, const double &simTime, const unsigned int &simFrame)
    {
        Framework::RDBHandler myHandler;
        myHandler.initMsg();

        RDB_TRIGGER_t *myTrigger = (RDB_TRIGGER_t *)myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_TRIGGER);

        if (!myTrigger)
            return;

        myTrigger->frameNo = simFrame + 1;
        myTrigger->deltaT = 0.04f;
        // usleep(deltaTime*1e6);
        ssize_t retVal = send(sendSocket, (const char *)(myHandler.getMsg()), myHandler.getMsgTotalSize(), 0);

        if (!retVal)
            fprintf(stderr, "sendTrigger: could not send trigger\n");
    }

    void
    iVH_Interface::sendOwnObjectState(int &sendSocket, const double &simTime, const unsigned int &simFrame, RDB_OBJECT_STATE_t *sOwnObjectState)
    {
        Framework::RDBHandler myHandler;

        // start a new message
        myHandler.initMsg();

        // begin with an SOF identifier
        myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_START_OF_FRAME);

        // add extended package for the object state
        RDB_OBJECT_STATE_t *objState = (RDB_OBJECT_STATE_t *)myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_OBJECT_STATE, 1, true);

        if (!objState)
        {
            fprintf(stderr, "sendOwnObjectState: could not create object state\n");
            return;
        }

        // copy contents of internally held object state to output structure
        memcpy(objState, sOwnObjectState, sizeof(RDB_OBJECT_STATE_t));

        // terminate with an EOF identifier
        myHandler.addPackage(simTime, simFrame, RDB_PKG_ID_END_OF_FRAME);

        ssize_t retVal = send(sendSocket, (const char *)(myHandler.getMsg()), myHandler.getMsgTotalSize(), 0);

        if (!retVal)
            fprintf(stderr, "sendOwnObjectState: could not send object state\n");
    }

    void
    iVH_Interface::getInitialPos(SensorIface *sensorData)
    {
        if (!sensorData)
            return;

        if (!mSensorInData)
            return;

        // go through incoming dynamic objects
        SensorIface::DataVec *objVec = 0;
        SensorIface::DataMap dataMap = mSensorInData->getDataMap();
        SensorIface::DataMap::iterator srcMapIt = dataMap.begin();

        // get pointer to roadmark data vector
        if ((srcMapIt = dataMap.find(RDB_PKG_ID_OBJECT_STATE)) != dataMap.end())
            objVec = srcMapIt->second;

        // do nothing if object data is not available
        if (!objVec)
            return;

        for (ModuleIface::DataVec::iterator srcVecIt = objVec->begin(); (srcVecIt != objVec->end()); srcVecIt++)
        {
            RDB_OBJECT_STATE_t *objInfo = (RDB_OBJECT_STATE_t *)((*srcVecIt)->data);

            if (((int)objInfo->base.id) == EgoID)
            {
                pIniVehicle->initialPosition.x = objInfo->base.pos.x;
                pIniVehicle->initialPosition.y = objInfo->base.pos.y;
                pIniVehicle->initialPosition.z = objInfo->base.pos.z;
                pIniVehicle->initialPosition.h = objInfo->base.pos.h;
                pIniVehicle->initialPosition.p = objInfo->base.pos.p;
                pIniVehicle->initialPosition.r = objInfo->base.pos.r;
            }
        }
        return;
    }


    int
    iVH_Interface::pack(int value, char *buf)
    {
        union temp
        {
            int value;
            char c[4];
        } in, out;

        in.value = value;
        out.c[0] = in.c[3];
        out.c[1] = in.c[2];
        out.c[2] = in.c[1];
        out.c[3] = in.c[0];

        memcpy(buf, out.c, 4);

        return 4;
    }

    int
    iVH_Interface::pack_short(unsigned short value, char *buf)
    {
        union temp
        {
            unsigned short value;
            char c[2];
        } in, out;

        in.value = value;
        out.c[0] = in.c[1];
        out.c[1] = in.c[0];

        memcpy(buf, out.c, 4);

        return 4;
    }

    int
    iVH_Interface::pack_3(int value, char *buf)
    {
        union temp
        {
            int value;
            char c[3];
        } in, out;

        in.value = value;
        out.c[0] = in.c[2];
        out.c[1] = in.c[1];
        out.c[2] = in.c[0];

        memcpy(buf, out.c, 3);

        return 3;
    }

    int
    iVH_Interface::pack_float(float value, char *buf)
    {
        union temp
        {
            float value;
            char c[4];
        } in, out;

        in.value = value;
        out.c[0] = in.c[3];
        out.c[1] = in.c[2];
        out.c[2] = in.c[1];
        out.c[3] = in.c[0];

        memcpy(buf, out.c, 4);

        return 4;
    }

    int
    iVH_Interface::pack_double(double value, char *buf)
    {
        union temp
        {
            double value;
            char c[8];
        } in, out;

        in.value = value;
        out.c[0] = in.c[7];
        out.c[1] = in.c[6];
        out.c[2] = in.c[5];
        out.c[3] = in.c[4];
        out.c[4] = in.c[3];
        out.c[5] = in.c[2];
        out.c[6] = in.c[1];
        out.c[7] = in.c[0];

        memcpy(buf, out.c, 8);

        return 8;
    }

    // void *
    // iVH_Interface::udpdriversend(void *param)
    // {
    //     int socket = *((int *)param);

    //     while (1)
    //     {
    //         ssize_t retval = recvfrom(socket, (RT_3000 *)&rt_3000, sizeof(RT_3000), MSG_WAITALL, (struct sockaddr *)&cliaddr, (socklen_t *)&cliaddr_len);
    //         if (retval == -1)
    //         {
    //             perror("receive failed");
    //             exit(EXIT_FAILURE);
    //         }

            //     ivh_Sync = rt_3000.Sync;
            //     ivh_time = rt_3000.time;
            //     ivh_acc_x = rt_3000.acc_x;
            //     ivh_acc_y = rt_3000.acc_y;
            //     ivh_acc_z = rt_3000.acc_z;
            //     ivh_ang_x = rt_3000.ang_x;
            //     ivh_ang_y = rt_3000.ang_y;
            //     ivh_ang_z = rt_3000.ang_z;
            //     ivh_Nav_status = rt_3000.Nav_status;
            //     ivh_checksum1 = rt_3000.checksum1;
            //     ivh_GPS_latitude = rt_3000.GPS_latitude;
            //     ivh_GPS_longitude = rt_3000.GPS_longitude;
            //     ivh_Altitude = rt_3000.Altitude;
            //     ivh_North_vel = rt_3000.North_vel;
            //     ivh_East_vel = rt_3000.East_vel;
            //     ivh_Down_vel = rt_3000.Down_vel;
            //     ivh_Heading = rt_3000.Heading;
            //     ivh_pitch = rt_3000.pitch;
            //     ivh_Roll = rt_3000.Roll;
            //     ivh_checksum2 = rt_3000.checksum2;
            //     ivh_status_channel = rt_3000.status_channel;
            //     ivh_Batch_S = rt_3000.Batch_S;
            //     ivh_checksum3 = rt_3000.checksum3;

            // ellipsoidData elips;
            // tEgoResponse wonwoo;
            // memset(&wonwoo, 0, sizeof(tEgoResponse));

            // utmData utm;
            // wgsData wgsB;
            // iVH_Interface module;

            // int ellipsoidModelID = 22;

            // module.ellipsoidDB(ellipsoidModelID, elips);

            // wgsB.latitude = wonwoo.GPS_latiutde;
            // wgsB.longitude = wonwoo.GPS_longitude;

            // module.WGStoUTM(elips, wgsB, utm);
            // // WGStoUTM(elips, wgsB, utm);

            // Northing = utm.Northing;
            // Easting = utm.Easting;
            // Zone = utm.Zone;

            // fprintf(stderr, "WGS to UTM converted.\n");
            // int64_t ms = 10;
            // std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            // fprintf(stderr, "10ms time sleep..\n");
    //     }
    //     return 0;
    // }

} // namespace Module
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
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <bitset>
#include <iostream>

using namespace std;

std::ofstream myfile;

void * udpdriversend(void *param);

int mOutSocketUDP;

#pragma pack(push, 1) //1 byte alignment
        
    typedef struct
    {
        unsigned char Sync; // 0

        // Batch A
        unsigned short time;     // 1~2   각가속도 구하기 위해 사용할 수 있음
        int acc_x : 24; // 3~5  ext.accel.x 사용
        int acc_y : 24; // 6~8  ext.accel.y 사용
        int acc_z : 24; // 9~11
        int ang_x : 24; // 12~14
        int ang_y : 24; // 15~17
        int ang_z : 24; // 18~20

        unsigned char Nav_status; // 21

        unsigned char checksum1; // 22

        // Batch B
        double GPS_latitude;         // 23 ~ 30  그대로 사용
        double GPS_longitude;        // 31 ~ 38  그대로 사용
        float Altitude;              // 39 ~ 42
        int North_vel : 24; // 43 ~ 45
        int East_vel : 24;  // 46 ~ 48
        int Down_vel : 24;  // 49 ~ 51
        int Heading : 24;   // 52 ~ 54 base.pos.h 사용
        int pitch : 24;     // 55 ~ 57
        int Roll : 24;      // 58 ~ 60

        unsigned char checksum2; // 61

        unsigned char status_channel; // 62 범위 : 1~255

        // Batch S 
        
        double Batch_S; // 63 ~ 70

        unsigned char checksum3; // 71
        // int8_t byt[3];
        // float time;
        // double   GPS_longitude;
        // double   GPS_latitude;
        // double   GPS_height;
        // double   GPS_azimuth;
        // float   INS_yaw_acc;
        // float   INS_pitch_acc;
        // float   INS_roll_acc;
        // double   INS_x_acc;
        // double   INS_y_acc;
        // double   INS_z_acc;

    } RT_3000; // RT-3000 인터페이스 통신 프로토콜 내용 (UDP)
#pragma pack(pop)

RT_3000 rt_3000;
pthread_t tid;

struct sockaddr_in cliaddr;

int cliaddr_len;



int main(void){

        
        myfile.open("/home/lmk/rt_3000_ gps_data.txt");

        myfile << "Sync" << " " << "time" << " " << "acc_x" << " " << "acc_x_binary" << " " << "acc_x_binary_component" << " " << "acc_y" << " " << "acc_y_binary" << " " << "acc_y_binary_component"  << " " << "acc_z" << " " << "acc_z_binary"<< " " << "acc_z_binary_component" << " " << "ang_x" << " " << "ang_y"<< " " << "ang_z"<< " " << "Nav_status" << " " << "checksum1" << " "  ; 
        myfile << "GPS_latitude" << " " << "GPS_longitude" << " " << "Altitude" << " " << "North_vel"<< " " << "East_vel" << " " << "Down_vel" << " " << "Heading" << " " << "pitch" << " " << "Roll" << " " << "checksum2" << " " << "status_channel" << "\n" ;
        struct sockaddr_in servaddr; 
        
        bool bConnected = false;

        // Creating socket file descriptor 
        if ( (mOutSocketUDP = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket creation failed"); 
            exit(EXIT_FAILURE); 
        } 

        memset(&servaddr, 0, sizeof(servaddr)); 

        // Filling server information 
        servaddr.sin_family    = AF_INET; // IPv4 
        servaddr.sin_addr.s_addr = INADDR_BROADCAST;
        // servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr.sin_port = htons(3000);

        const int enable = 1;
        setsockopt(mOutSocketUDP, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) ;

        if ( bind(mOutSocketUDP, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 )
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        // while (!bConnected)
        // {
        //     if (bind(mOutSocketUDP, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        //     {
        //         perror("bind failed");
        //         sleep(1);
        //     }
        //     else
        //         bConnected = true;
        // }

        fprintf(stderr,"Connect!\n");

        memset(&rt_3000, 0, sizeof(RT_3000));

        cliaddr_len = sizeof(sockaddr_in);

        pthread_create(&tid,NULL,udpdriversend,&mOutSocketUDP);
        pthread_join(tid,NULL);
        myfile.close();

        
}

void * udpdriversend(void *param)
{
    
    int socket = *((int *)param);
    // ssize_t retval = recvfrom(socket, (RT_3000 *)&rt_3000, sizeof(RT_3000), MSG_WAITALL, (struct sockaddr *)&cliaddr, (socklen_t *)&cliaddr_len);
    // if (retval == -1)
    // {
    //     perror("receive failed");
    //     exit(EXIT_FAILURE);
    // }
    fprintf(stderr,"====================== Thread1 ======================\n");
    memset(&rt_3000, 0, sizeof(RT_3000));
    while (true)
    {
        ssize_t retval = recvfrom(socket, (RT_3000 *)&rt_3000, sizeof(RT_3000), MSG_WAITALL, (struct sockaddr *)&cliaddr, (socklen_t *)&cliaddr_len);
        if (retval == -1)
        {
            perror("receive failed");
            exit(EXIT_FAILURE);
        }
        fprintf(stderr,"====================== Receive from RT 3000 ======================\n");
        fprintf(stderr, "Sync : %d\n", rt_3000.Sync);
        myfile << int(rt_3000.Sync) << " " ;
        fprintf(stderr, "time : %d\n", rt_3000.time); // Time is transmitted as milliseconds into the current GPS minute. The range is 0–59,999 ms.
        myfile << rt_3000.time << " " ;
        fprintf(stderr, "acc_x : %f\n", float(rt_3000.acc_x)/10000);
        myfile << rt_3000.acc_x << " " ;
        myfile << bitset<32>(rt_3000.acc_x)<< " " ;
        myfile << ~bitset<32>(rt_3000.acc_x)<< " " ;
        cout << bitset<32>(rt_3000.acc_x) << endl;
        fprintf(stderr, "acc_y : %f\n", float(rt_3000.acc_y)/10000);
        myfile << rt_3000.acc_y << " " ;
        myfile << bitset<32>(rt_3000.acc_y)<< " " ;
        myfile << ~bitset<32>(rt_3000.acc_y)<< " " ;
        fprintf(stderr, "acc_z : %f\n", float(rt_3000.acc_z)/10000);
        myfile << rt_3000.acc_z << " " ;
        cout << bitset<32>(rt_3000.acc_z) << endl;
        myfile << bitset<32>(rt_3000.acc_z)<< " " ;
        myfile << ~bitset<32>(rt_3000.acc_z)<< " " ;
        cout << ~bitset<32>(rt_3000.acc_z) << endl;
        fprintf(stderr, "ang_x : %f\n", float(rt_3000.ang_x)/100000 * 180 / M_PI);
        myfile << rt_3000.ang_x << " " ;
        fprintf(stderr, "ang_y : %f\n", float(rt_3000.ang_y)/100000 * 180 / M_PI);
        myfile << rt_3000.ang_y << " " ;
        fprintf(stderr, "ang_z : %f\n", float(rt_3000.ang_z)/100000 * 180 / M_PI);
        myfile << rt_3000.ang_z << " " ;
        fprintf(stderr, "Nav_status : %d\n", rt_3000.Nav_status);
        myfile << int(rt_3000.Nav_status) << " " ;
        fprintf(stderr, "checksum1 : %d\n", rt_3000.checksum1);
        myfile << int(rt_3000.checksum1) << " " ;
        fprintf(stderr, "GPS_latitude : %.12lf\n", rt_3000.GPS_latitude * 180 / M_PI);
        myfile << rt_3000.GPS_latitude << " " ;
        fprintf(stderr, "GPS_longitude : %.12lf\n", rt_3000.GPS_longitude * 180 / M_PI);
        myfile << rt_3000.GPS_longitude << " " ;

        fprintf(stderr, "Altitude : %f\n", rt_3000.Altitude);
        myfile << rt_3000.Altitude << " " ;
        fprintf(stderr, "North_vel : %f\n", float(rt_3000.North_vel) /10000);
        myfile << rt_3000.North_vel << " " ;
        fprintf(stderr, "East_vel : %f\n", float(rt_3000.East_vel)/10000);
        myfile << rt_3000.East_vel << " " ;
        fprintf(stderr, "Down_vel : %f\n", float(rt_3000.Down_vel)/10000);
        myfile << rt_3000.Down_vel << " " ;
        fprintf(stderr, "Heading : %f\n", float(rt_3000.Heading)/1000000 * 180 / M_PI);
        myfile << rt_3000.Heading << " " ;
        fprintf(stderr, "pitch : %f\n", float(rt_3000.pitch)/1000000 * 180 / M_PI);
        myfile << rt_3000.pitch << " " ;
        fprintf(stderr, "Roll : %f\n", float(rt_3000.Roll)/1000000 * 180 / M_PI);
        myfile << rt_3000.Roll << " " ;
        fprintf(stderr, "checksum2 : %d\n", rt_3000.checksum2);
        myfile << int(rt_3000.checksum2) << " " ;
        fprintf(stderr, "status_channel : %d\n", rt_3000.status_channel);
        myfile << int(rt_3000.status_channel) << "\n" ;
        fprintf(stderr, "checksum3 : %d\n", rt_3000.checksum3);
        fprintf(stderr,"===================================================================\n");
        myfile.flush();
        sleep(0.01);
    }
    // return 0;
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

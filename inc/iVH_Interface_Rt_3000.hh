/* ===================================================
 *  file:       iVH_Interface.hh
 * ---------------------------------------------------
 *  purpose:	the plugin of the perfect sensor
 * ---------------------------------------------------
 *  first edit:	01.02.2016 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _MODULE_IVH_INTERFACE_TREEZE_DIRVINGSIMULATOR_HH
#define _MODULE_IVH_INTERFACE_TREEZE_DIRVINGSIMULATOR_HH

/* ====== INCLUSIONS ====== */
#include <cmath>			
#include <cstdio>			
#include <cstdlib>			
#include <cstring>			
#include <cctype>
#include <string>
#include "SensorPlugin.hh"
#include <iostream>
#include <iomanip>
#include <sstream>
#include "RDBHandler.hh"
#include <limits.h>

const double PI       =	4*atan(1);	
const double deg2rad  =	PI/180;
const double rad2deg  =	180/PI;
const double k0       =	0.9996;


namespace Module
{
// forward declarations
class SensorIface;

class iVH_Interface : public Module::SensorPlugin
{
    public:
        /**
        * factory function for creating a new object, derived from ParamIface 
        */
        static Framework::Plugin* makeModule();

    public:
        /**
        * constructor
        *   @param  myName networking name of the module
        **/
        explicit iVH_Interface();

        /**
        * Destroy the class. 
        */
        virtual ~iVH_Interface();
        
        /**
        * initialize interface
        * @return success/failure
        */
        virtual bool init();

        /**
        * reset the plugin (called e.g. after re-start of simulation)
        * @param    simTime the simulation time used as reset time
        * @return true if successful
        */
        virtual bool reset( const double & simTime );
                
        /**
        * handle an incoming sensor message
        * @param frameNo     the current frame number
        * @param sensorData  sensor interface data
        * @param dynamicData   dynamics interface data
        * @return return code
        */
        virtual int update( const unsigned long & frameNo, SensorIface* sensorData);
	
        /**
        * handle directly an SCP element
        * @param parser pointer to the ScpParser which contains the SCP element
        * @return true if element could be handled
        */
        virtual bool handleSCPElement( Framework::ScpParser* parser );

        /**
        * create the communication socket
        */
        void createComSocketTCP();
        // void createComSocketTCP_GPS();
        
        void createComSocketUDP_GPS();

        void createComSocketUDP2();
        // void createComSocketTCP_Image();

/**
 * Handle a RDBImage and print it out
//  * @param simTime
//  * @param simFrame
//  * @param img
//  */
//         void handleRDBitem(const double & simTime, const unsigned int & simFrame, RDB_IMAGE_t* img);



        // void createComSocketUDP_IMG();
      

    private:
        /**
        * remember last frame number
        */
        unsigned long mLastFrameNo;
	
        /**
         * internal frame counter
         */
        unsigned long mFrameCounter;


        /*
        * ouput socket
        */

       int mOutSocketTCP;
       int mOutSocketTCP_test;
       int mOutSocketUDP;
       int mOutSocketUDP2;

       int switchTriggering;

       int bConnectedDriver;
       int bConnectedDriver2;
       double prevtime = 0.0001;
       float prevAutonomous_cmd = 0.0f;
       float prevpos_x = 0.0f;

       float prevtime_obj1 = 0.0001f;
       float objprev1 = 0.0001f;
       float a,b,c,d;
       float a_left,b_left,c_left,d_left;
       float a_right,b_right,c_right,d_right;

       RDB_ROADMARK_t *roadmarklistLeft = (RDB_ROADMARK_t *)malloc(sizeof(RDB_ROADMARK_t));
       RDB_POINT_t *pt_left0 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_POINT_t *pt_left1 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_POINT_t *pt_left2 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_POINT_t *pt_left3 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_ROADMARK_t *roadmarklistRight = (RDB_ROADMARK_t *)malloc(sizeof(RDB_ROADMARK_t));
       RDB_POINT_t *pt_right0 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_POINT_t *pt_right1 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_POINT_t *pt_right2 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_POINT_t *pt_right3 = (RDB_POINT_t *)malloc(sizeof(RDB_POINT_t));
       RDB_OBJECT_STATE_t *objectlist = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));
       RDB_OBJECT_STATE_t *objectlist_wonwoo = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));
       RDB_OBJECT_STATE_t *Allobjectlist = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));
       RDB_DRIVER_PERCEPTION_t *lightindicator = (RDB_DRIVER_PERCEPTION_t *)malloc(sizeof(RDB_DRIVER_PERCEPTION_t));

       RDB_OBJECT_STATE_t *prevobjectlist = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));
       RDB_OBJECT_STATE_t *objlist_prev = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));
       RDB_OBJECT_STATE_t *objectlist_gps = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));

       int EgoID;

    private:
        #pragma pack(push, 1) //1 byte alignment
        
                typedef struct
                {
                    char Sync; // 0

                    // Batch A
                    unsigned short time;     // 1~2   각가속도 구하기 위해 사용할 수 있음
                    int acc_x : 24; // 3~5  ext.accel.x 사용
                    int acc_y : 24; // 6~8  ext.accel.y 사용
                    int acc_z : 24; // 9~11
                    int ang_x : 24; // 12~14
                    int ang_y : 24; // 15~17
                    int ang_z : 24; // 18~20

                    char Nav_status; // 21

                    char checksum1; // 22

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

                    char checksum2; // 61

                    char status_channel; // 62 범위 : 1~255

                    // Batch S : 채널이 많아서 우선 8Byte로 통합하였음.
                    
                    double Batch_S; // 63 ~ 70

                    char checksum3; // 71

                } RT_3000; // RT-3000 인터페이스 통신 프로토콜 내용 (UDP)

                typedef struct
                {
                    unsigned int ang_z : 24;
                } INIT;

                typedef struct
                {
                    double GPS_longitude;
                    double GPS_latiutde;
                    double GPS_height;
                    double GPS_azimuth;
                    float INS_yaw_acc;
                    float INS_pitch_acc;
                    float INS_roll_acc;
                    double INS_x_acc;
                    double INS_y_acc;
                    double INS_z_acc;
                } tEgoResponse; // 자사내의 테스트용 UDP Sender로부터 받기위한 데이터 구조체

                typedef struct
                {
                    double Northing;
                    double Easting;
                    int Zone;
                } utmData;

                typedef struct
                {
                    int eId;
                    double longitude;
                    double latitude;
                } wgsData;

                typedef struct
                {
                    int eid;
                    char name[20];
                    double EquatorialRadius;
                    double inverseFlattening;
                } ellipsoidData;

                typedef struct
                {
                    double x;        /**< x position                                                @unit m                                @version 0x0100 */
                    double y;        /**< y position                                                @unit m                                @version 0x0100 */
                    double z;        /**< z position                                                @unit m                                @version 0x0100 */
                    float h;         /**< heading angle                                             @unit rad                              @version 0x0100 */
                    float p;         /**< pitch angle                                               @unit rad                              @version 0x0100 */
                    float r;         /**< roll angle                                                @unit rad                              @version 0x0100 */
                    uint32_t flags;  /**< co-ordinate flags                                         @unit @link RDB_COORD_FLAG @endlink    @version 0x0100 */
                    uint32_t type;   /**< co-ordinate system type identifier                        @unit @link RDB_COORD_TYPE @endlink    @version 0x0100 */
                    uint32_t system; /**< unique ID of the corresponding (user) co-ordinate system  @unit _                                @version 0x0100 */
                } iVH_RDB_COORD_t;

                typedef struct
                {
                    double dimenX;
                    double dimenY;
                    double dimenZ;
                    double offX;
                    double offY;
                    double offZ;
                } vehicleSize; // 차량 크기 : VTD는 차량의 뒤차축 중심을 0,0,0으로 하기 때문에 차량 크기에서의 중심점을 따로 offset 시킨다. 따라서 차량크기의 중심점이다.

                typedef struct
                {
                    float latitude;
                    float longitude;
                    double locationX;
                    double locationY;
                    double heading;
                } vehicleGPS; // 차량 위치 : 차량 좌표계  차량의 위치는 UTM좌표계 또는 지구좌표계(Geo coordinate system). utm : x - East, y - North

                typedef struct
                {
                    float velocity;
                } vehicleVel; // 차량 속도 : km/h

                typedef struct
                {
                    float accX;
                    float accY;
                    float accZ;
                } vehicleAcc; // match : RDB_OBJECT_STATE_t->ext.accel.x, y, z은 double인데 왜 float인지 알아볼것

                typedef struct
                {
                    int emgrLight;
                } vehicleEmgrLight;

                typedef struct
                {
                    int leftLight;
                    int rightLight;
                } vehicleTrunsigLight;

                typedef struct
                {
                    int takeOver;
                } vehicleTakeover; // 제어권 전환 버튼을 눌렀을때 자율주행모드 : 1, 다시 버튼을 누르면 운전자가 수동주행 : 2

                typedef struct
                {
                    int leftLaneroadType;
                    int leftLaneid;
                    int leftLanecolor;
                    int rightLaneroadType;
                    int rightLaneid;
                    int rightLanecolor;
                    float leftlateralWidth;  // lateralWidth : 차량 중심으로부터 좌측 차선까지의 거리
                    float rightlateralWidth; // lateralWidth : 차량 중심으로부터 우측 차선까지의 거리
                } vehicleLane; // AV차량이 주행 중일때 차선/차로 정보 : 차량 중심으로부터 차선까지의 거리 좌/우측

                typedef struct
                {
                    float accidentLatitude;
                    float accidentLongitude;
                } vehicleAccident; // AV차량의 충돌 정보 : 충돌 했을 경우 위/경도 value가 나오고, 충돌 안하고 있을 경우 0이다.

                typedef struct
                {
                    int cutIn;
                    int cutOut;
                    float distance;
                    float velocity;
                } vehicleInfront; // 선행 차량 정보 : 앞차와의 거리 정보, 앞차의 속도 정보 절대속도로 얻을 수 있다. km/h이다. m/s쓸경우 단위 변경해야한다.

                typedef struct
                {
                    int trafficId;
                    int trafficType;
                    int trafficLeft;
                    int trafficGreen;
                    int trafficYellow;
                    int trafficRed;
                } vehicleTraffic; // 신호등

                typedef struct
                {
                    int cloudstate; // 비가 오는지 안오는지 까지만 적용
                    int weather1;
                    int weather2;
                } vehicleWeather; // 날씨

                typedef struct
                {
                    int PedestrianLatitude;
                    int PedestrianLongitude;
                } vehiclePedestrian; // 보행자

                typedef struct
                {
                    iVH_RDB_COORD_t initialPosition;
                    iVH_RDB_COORD_t initialSpeed;
                    iVH_RDB_COORD_t initialAcceleration;
                } vehicleInitialInformation;

                typedef struct
                {
                    vehicleSize size; // 차량 크기 : VTD는 차량의 뒤차축 중심을 0,0,0으로 하기 때문에 차량 크기에서의 중심점을 따로 offset 시킨다. 따라서 차량크기의 중심점이다.
                    vehicleGPS gps; // 차량 위치 : 차량 좌표계  차량의 위치는 UTM좌표계 또는 지구좌표계(Geo coordinate system). utm : x - East, y - North
                    vehicleVel vel; // 차량 속도 : km/h
                    vehicleAcc acc; // acc
                    vehicleEmgrLight emergence; //
                    vehicleTrunsigLight turnsig; //
                    vehicleTakeover takeover; // 제어권 전환 버튼을 눌렀을때 자율주행모드 : 1, 다시 버튼을 누르면 운전자가 수동주행 : 2
                    vehicleLane lane; // AV차량이 주행 중일때 차선/차로 정보 : 차량 중심으로부터 차선까지의 거리 좌/우측
                    vehicleAccident accident; // AV차량의 충돌 정보 : 충돌 했을 경우 위/경도 value가 나오고, 충돌 안하고 있을 경우 0이다.
                    vehicleInfront frontvehicle; // 선행 차량 정보 : 절대속도로 얻을 수 있다. km/h이다. m/s쓸경우 단위 변경해야한다.
                    vehicleTraffic traffic; // 신호등
                    vehicleWeather weather; // 날씨
                    vehiclePedestrian pedestrian; // 보행자
                    
                } vtdInfo;

#pragma pack(pop)
        
// #pragma pack(push, 1) //1 byte alignment

//         typedef struct
//         {
//             float EPS_Cmd;        //Brake cmd [0..1]
//             float ACC_Cmd;        //ACC cmd [0..1]
//             float Autonomous_Cmd; // Take Over cmd [0..1]
//             float Steering_Cmd;   // Steering cmd [rad]
//         } Controlinput;

//         typedef struct
//         {
//             float EPS_Cmd;        //Brake cmd [0..1]
//             float ACC_Cmd;        //ACC cmd [0..1]
//             float Autonomous_Cmd; // Take Over cmd [0..1]
//             float Steering_Cmd;   // Steering cmd [rad]
//             float velocity;
//             float rpm;
//         } Controlinput_totreeze;

//         typedef struct
//         {
//             int eid;
//             char name[20];
//             double EquatorialRadius;
//             double inverseFlattening;
//         } ellipsoidData;

//         typedef struct
//         {
//             int eId;
//             double dX;
//             double dY;
//             double dZ;
//         } datumData;

//         typedef struct
//         {
//             int eId;
//             double longitude;
//             double latitude;
//         } wgsData;

//         typedef struct
//         {
//             wgsData wgs;
//             double azimuth;
//         } gpsData;

//         typedef struct
//         {
//             double Northing;
//             double Easting;
//             int Zone;
//         } utmData;

//         typedef struct
//         {
//             double frtLeftWheel_pos_z;
//             double frtRightWheel_pos_z;
//             double rearLeftWheel_pos_z;
//             double rearRightWheel_pos_z;
//         } contactPatchPos;

// #pragma pack(pop)

        // utmData *prevutmdata = (utmData *)malloc(sizeof(utmData));
        // wgsData *prevwgsdata = (wgsData *)malloc(sizeof(wgsData));
        // gpsData *prevgpsdata = (gpsData *)malloc(sizeof(gpsData));
        

        void GPS_Data();

        RDB_OBJECT_STATE_t *objectlist_ego = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));
        RDB_ENGINE_t *engine_ego = (RDB_ENGINE_t *)malloc(sizeof(RDB_ENGINE_t));

        void objStateSend(const double &simTime,RDB_OBJECT_STATE_t* kNUTobjectState,RDB_DRIVER_PERCEPTION_t* kNUTlightindicator, RDB_WHEEL_t* kNUTwheel, RDB_ROADMARK_t* kNUTroadmarkLeft, RDB_ROADMARK_t* kNUTroadmarkRight,RDB_OBJECT_STATE_t* kNUTobjectState_GPS, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera0, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera1, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera2, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera3, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera4, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera5, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera6, RDB_SENSOR_OBJECT_t * kNUTsensorobject_camera7,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontlong0,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontlong1,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontlong2,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontlong3,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontlong4,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontshort0,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontshort1,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontshort2,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontshort3,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontshort4,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontleft0,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontleft1,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontleft2,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontleft3,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontleft4,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontright0,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontright1,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontright2,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontright3,RDB_SENSOR_OBJECT_t * kNUTsensorobject_frontright4,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearleft0,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearleft1,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearleft2,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearleft3,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearleft4,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearright0,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearright1,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearright2,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearright3,RDB_SENSOR_OBJECT_t * kNUTsensorobject_rearright4);
        void dummyTCrecv(int &sendSocket);
        void triggerFunc( int & sendSocket, const double & simTime, const unsigned int & simFrame);
        void getInitialPos(SensorIface* sensorData );
        void parseRDBMessage( RDB_MSG_t* msg, bool & isImage );
        void ivhDriverSend(RDB_DRIVER_CTRL_t* ivhDriverCTRL,RDB_OBJECT_STATE_t *obj,RDB_ENGINE_t *engine);
        void sendOwnObjectState( int & sendSocket, const double & simTime, const unsigned int & simFrame , RDB_OBJECT_STATE_t * sOwnObjectState );
        void parseRDBMessageEntry( const double & simTime, const unsigned int & simFrame, RDB_MSG_ENTRY_HDR_t* entryHdr );

        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_OBJECT_STATE_t & item, bool isExtended, int noElements );
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_DRIVER_PERCEPTION_t & item, unsigned char ident );
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_ROADMARK_t & info , unsigned char ident);
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_WHEEL_t & item, bool isExtended );
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_SENSOR_OBJECT_t & item, unsigned char ident, int noElements);
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_ENVIRONMENT_t & info , unsigned char ident);
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_TRAFFIC_SIGN_t & info , unsigned char ident);
        void handleRDBitem( const double & simTime, const unsigned int & simFrame, RDB_TRAFFIC_LIGHT_t & item, bool isExtended, int noElements );

        // void WGStoUTM(ellipsoidData ellip, wgsData wgs, utmData& utm); // 주요함수
        void WGStoUTM(ellipsoidData ellip, wgsData wgs, utmData& utm);
        void ivhVehicleDynamicsSend(RDB_OBJECT_STATE_t *iVHVehicleDynamics, SensorIface *ifaceData, ellipsoidData elips, iVH_Interface module, tEgoResponse wgs, utmData utm);

        // void UTMtoWGS(ellipsoidData ellip, utmData utm,  wgsData& wgs);
		
        void ellipsoidDB(int id,ellipsoidData& ellipD );
		double dtor(double deg);
		double rtod(double rad);
		// void azimuth(wgsData backwardPoint, wgsData forwardPoint, gpsData& gps);
        // void DatumConvert(ellipsoidData ellip, datumData datum, double LatIn, double LongIn, double HtIn, int dTo,  double& LatTo, double& LongTo, double& HtTo);
        static char* getIdentString( unsigned char ident );
        void print( const RDB_POINT_t & info, unsigned char ident, bool csv, bool csvHeader );
        static std::string coordType2string( unsigned int type );
        void MatrixCalcualtion( float a11, float a12, float a13, float a14, float a21, float a22, float a23, float a24,float a31,float a32,float a33,float a34,float a41,float a42,float a43,float a44, float y1, float y2, float y3, float y4 );

        vehicleInitialInformation* pIniVehicle ;
        
        vtdInfo vehicleinformation;

        // INIT init_ang_z;
        int init_ang_z = 0;
        unsigned short init_time = 0; // 시간
        int flag = 0;
	    int prev_t = 0;

        int pack(int value, char* buf);
	    int pack_3(int value, char *buf);
        int pack_short(unsigned short value, char* buf);
        int pack_float(float value, char* buf);
        int pack_double(double value, char* buf);
        static void* udpdriversend(void* param);

        float convert(const unsigned char* src);
        unsigned char* makeS24( unsigned int i, unsigned char* s24 );

        float prev_Heading = 0.0f;

        // RDB_OBJECT_STATE_t *objectlist_treeze = (RDB_OBJECT_STATE_t *)malloc(sizeof(RDB_OBJECT_STATE_t));

        // Controlinput input;

        // for pthread udpdriversend
        char ivh_Sync;
        unsigned short ivh_time;
        int ivh_acc_x;
        int ivh_acc_y;
        int ivh_acc_z;
        int ivh_ang_x;
        int ivh_ang_y;
        int ivh_ang_z;
        char ivh_Nav_status;
        char ivh_checksum1;
        double ivh_GPS_latitude;
        double ivh_GPS_longitude;
        float ivh_Altitude;
        int ivh_North_vel;
        int ivh_East_vel;
        int ivh_Down_vel;
        int ivh_Heading;
        int ivh_pitch;
        int ivh_Roll;
        char ivh_checksum2;
        char ivh_status_channel;
        double ivh_Batch_S;
        char ivh_checksum3;
};
} // namespace Sensor
#endif /* _MODULE_IVH_INTERFACE_TREEZE_DIRVINGSIMULATOR_HH */

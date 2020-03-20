#ifndef VTK_COMM_DEF_H_
#define VTK_COMM_DEF_H_
#include <stdint.h>
#include <vtk_basic_type.h>
#include <msg-cfg.h>

#ifndef PACKED_STRUCT
#define PACKED_STRUCT __attribute__ ((packed))
#endif

#define MAX_VERSION_INFO                          28u
#define MAX_ERROR_DESCRIPTION                     128u
#define MAX_ACK_DESCRIPTION                       92u

/**
* @CN
* @brief communication_op_type 枚举
*
* @EN
* @brief communication_op_type enumeration
* @END
*/
enum communication_op_type {
    STREAM_INVALID                         = 0,
    /**
    * @CN
    * @brief 事件流
    *
    * @EN
    * @brief event flow
    * @END
    */
    STREAM_EVENT                           = 1,
    /**
    * @CN
    * @brief 数据流
    *
    * @EN
    * @brief data flow
    * @END
    */
    STREAM_DATA                            = 2,
    /**
    * @CN
    * @brief 控制流
    *
    * @EN
    * @brief control flow
    * @END
    */
    STREAM_CONTROL                         = 3,
};

/**
* @CN
* @brief communication event type 枚举
*
* @EN
* @brief communication event type enumeration
* @END
*/
enum event_type {
    EVENT_INVAILD                          = 0,
    /** V2V Event */
    EVENT_V2V                              = 1,
    /** SPAT Event */
    EVENT_SPAT                             = 2,
    /** RSA Event */
    EVENT_RSA                              = 3,
    /** VRU Event */
    EVENT_VRU                              = 4,
    /** CANCEL Event */
    EVENT_CANCEL                           = 255,
    /** ERROR Event */
    EVENT_ERROR                            = 6,
    /** ACK Event */
    EVENT_ACK                              = 7,
    /** ALL ALERT Event */
    EVENT_ALL_ALERT                        = 8,
    /** Diagnosis Event */
    EVENT_DIAGNOSE                         = 9,
    /** Heartbeat Notification */
    EVENT_HEARTBEAT                        = 11,
    /**  Power Management Notice */
    EVENT_POWERMNG_NOTICE                  = 12,
    /**  Power Management Reply */
    EVENT_POWERMNG_REPLY                   = 13,

};

/**
* @CN
* @brief communication data type 枚举
*
* @EN
* @brief communication data type enumeration
* @END
*/
enum data_type {
    DATA_INVAILD                           = 0,
    /**
    * @CN
    * @brief 内部数据
    *
    * @EN
    * @brief Internal data
    * @END
    */
    /**
    * @CN
    * @brief 车辆行驶数据
    *
    * @EN
    * @brief Vehicle runtime data
    * @END
    */
    DATA_VEHICLE_RUNTIME                   = 1,
    /**
    * @CN
    * @brief 查看运行状态
    *
    * @EN
    * @brief Check the running status
    * @END
    */
    DATA_SYSTEM_INFO                       = 2,
    /**
    * @CN
    * @brief 查看运行模式
    *
    * @EN
    * @brief Check the operating mode
    * @END
    */
    DATA_SYSTEM_FUNCTION                   = 3,
    /**
    * @CN
    * @brief SPAT 数据流载荷
    *
    * @EN
    * @brief SPAT data flow load
    * @END
    */
    DATA_SIMULATED_SPAT                    = 4,
    /**
    * @CN
    * @brief 外部数据
    *
    * @EN
    * @brief Outside data
    * @END
    */
    /**
    * @CN
    * @brief 外部交通控制系统实时数据
    *
    * @EN
    * @brief Outside traffic control system real-time data
    * @END
    */

    /** The PDU of Probe Vehicle Data (OP Code: 5) is sent by the V2X service to
        external subsystems for update the surrounding vehicle information collected by a RSU device. */
    DATA_PROBE_VECHICLE                    = 5,

    DATA_BSM_RUNTIME                       = 6,

    DATA_OUTSIDE_SPAT                      = 7,

    DATA_OUTSIDE_MAP                       = 8,

    DATA_APPLICATION_FUNTION               = 9,

    DATA_DIAG_INFO                         = 10,

    DATA_OUTSIDE_PSM                       = 11,

    DATA_OUTSIDE_RSI                       = 12,

    DATA_OUTSIDE_EnCodedMsg_PC5            = 13,

    DATA_OUTSIDE_EnCodedMsg_UU             = 14,

    DATA_RAW_PASS                          = 98,

    DATA_DIRECT_PASS_DATA                  = 99,

    DATA_OUTSIDE_REALTIME_SPAT             = 100,
    /**
    * @CN
    * @brief 摄像头识别外部行人数据
    *
    * @EN
    * @brief The outside camera identification data for pedestrians
    * @END
    */
    DATA_OUTSIDE_PEDESTRIAN_CAMERA         = 101,
    /**
    * @CN
    * @brief 摄像头识别外部行人数据
    *
    * @EN
    * @brief The outside camera identification data for pedestrians
    * @END
    */
    DATA_OUTSIDE_PEDESTRIAN_POSITION       = 102,
    /**
    * @CN
    * @brief 外部识别的车辆实时数据
    *
    * @EN
    * @brief The outside identification real-time data for vehicle
    * @END
    */
    DATA_OUTSIDE_VECHICLE                  = 103,
    /** The PDU of Outside GNSS Data (OP Code: 104) is sent by the external subsystems to service */
    DATA_OUTSIDE_GNSS                      = 104,
    /** The PDU of Outside Roadside Safety Message Data is sent by the external subsystems to service */
    DATA_OUTSIDE_ROADSIDE_SAFETY_MESSAGE   = 105,
    /** The PDU of Outside Roadside Safety Information Data is sent by the external subsystems to service */
    DATA_OUTSIDE_ROADSIDE_SAFETY_INFORMATION = 106,
    /** The PDU of Outside RSU Config is sent by the external subsystems to service */
    DATA_OUTSIDE_RSU_CONFIG = 107,
    /** The PDU of Outside Simulation data is sent by the Sumo **/
    DATA_SUMO_VICHICLE                     = 108,
    /**
    * @CN
    * @brief TBD CAN 数据
    *
    * @EN
    * @brief TBD CAN data
    * @END
    */
    DATA_OUTSIDE_CAN                       = 200,
    /**
    * @CN
    * @brief 标定数据
    *
    * @EN
    * @brief The calibration data
    * @END
    */
    DATA_CALIBRATION_DATA                       = 201,

};

/**
* @CN
* @brief communication control type 枚举
*
* @EN
* @brief communication control type enumeration
* @END
*/
enum control_type {
    CONTROL_INVAILD                        = 0,
    /** Get System INFO Data */
    CONTROL_GET_SYSTEM_INFO                = 1,
    /** Get System Function Data */
    CONTROL_GET_SYSTEM_FUNCTION            = 2,
    /** Get Calibration Data */
    CONTROL_GET_CALIBRATION_DATA           = 3,
    /** GET APPLICATION FUNCTION */
    CONTROL_GET_APPLICATION_FUNCTION       = 4,
    /** GET APPLICATION FUNCTION */
    CONTROL_GET_DIAGNOSIS_INFO             = 5,
    /** Set Engineer_Mode  */
    CONTROL_SET_ENGINERR_MODE              = 100,
    /** Set Record Log     */
    CONTROL_SET_RECORD_LOG                 = 101,
    /** Set Vechicle State */
    CONTROL_SET_VEHICLE_STATE              = 102,
    /** Set System Function */
    CONTROL_SET_SYSTEM_FUNCTION            = 103,
    CONTROL_RAW_DATA                       = 98,
    CONTROL_SET_SPAT_FUNCTION              = 104,
    CONTROL_ALERT_TEST                     = 105,
    /** SET APPLICATION FUNCTION */
    CONTROL_SET_APPLICATION_FUNCTION       = 106,
    /** RESTORE FACTORY SETTINGS*/
    CONTROL_RESTORE_FACTORY_SETTINGS       = 107,
    /** NOTIFY START UP STATUS*/
    CONTROL_NOTIFY_STARTUP_STATUS          =108,
    /** RELOAD RSU CONFIGURATION*/
    CONTROL_RELOAD_RSU_CONFIG              = 109,
    /** Set Calibration Data */
    CONTROL_SET_CALIBRATION_DATA           = 200
};

/**
* @CN
* @brief powermanager event type 枚举
*
* @EN
* @brief powermanager event type  enumeration
* @END
*/
enum powermanager_event_type {
    POWERMANAGER_EVENT_NOTICE   = 1,
    POWERMANAGER_EVENT_REPLY    = 2,
};

/**
 * @brief The speed_advisory_type enum
 */
enum speed_advisory_type {
    SPEED_ADVICE_UNAVAILABLE                  = 0,
    //不提醒
    SPEED_ADVICE_NO_ADVICE                    = 1,
    //紧急停车
    SPEED_ADVICE_EMERGENCE_STOP               = 2,
    //减速,在路口处停车
    SPEED_ADVICE_DECELERATION_TO_STOP         = 3,
    //以最大加速度加速
    SPEED_ADVICE_MAX_ACCELERATION             = 4,
    //以普通加速度加速
    SPEED_ADVICE_MID_ACCELERATION             = 5,
    //以最大减速度减速
    SPEED_ADVICE_MAX_DECELERATION             = 6,
    //以普通减速度减速
    SPEED_ADVICE_MID_DECELERATION             = 7,
    //提醒驾驶员超速
    SPEED_ADVICE_EXCEED_THE_SPEED_LIMIT       = 8,
    //提醒驾驶员速度已低于最低车速
    SPEED_ADVICE_BELOW_MINIMUM_SPEED          = 9,
    //提醒驾驶员当前绿灯无法通过，且距离路口较远，可以保持当前车速继续行驶
    SPEED_ADVICE_FAR_FROM_INTERSECTION        = 10,
    //危险
    SPEED_ADVICE_DANGER                       = 11,
};

/**
 * @brief The red_light_violate_state enum
 */
enum red_light_violate_state {
    //无闯红灯危险
    NO_RISK_RED_LIGHT_VIOLATE                  = 0,
    //有闯红灯危险或已闯红灯
    RISK_RED_LIGHT_VIOLATE                     = 1,
};

/**
 * @brief The rsa_kind_type enum
 */
enum rsa_kind_type {
    // 0 represents unable to judge classification
    KIND_UNKNOW       = 0,
    // 1 means ahead left
    KIND_AHEAD_LEFT   = 1,
    // 2 means exact ahead
    KIND_AHEAD        = 2,
    // 4 means ahead right
    KIND_AHEAD_RIGHT  = 4,
};

/**
 * @brief The vru_type enum
 */
enum vru_type {
    // 行人
    VRU_TYPE_PERSON   = 1,
    // 非机动车,比如自行车
    VRU_TYPE_NO_MOTOR = 2,
    // 摩托车
    VRU_TYPE_MOTOR    = 3,
    // 其他
    VRU_TYPE_UNKNOW   = 4,
};


/** outboud communication header */
typedef struct outbound_communication_header {
    /** 0xAFDD2468 */
    uint32_t proto_id;
    /** see communication op_type 4bit */
    uint8_t  op_type   : 4;
    /**  current ver = 0 4 bit*/
    uint8_t  ver       : 4;
    /** see  communication op_code */
    uint8_t  op_code;
    /** range 0-255 use for events */
    uint8_t  msg_priority;
    /** reserved1 */
    uint8_t  rsvd1;
    /** Seconds since the Epoch*/
    uint32_t timestamp_seconds;
    /** Miliseconds since the last Second */
    uint32_t timestamp_miliseconds;
    /** Local OBU/RSU ID */
    uint32_t local_id;
    /** reserved2 */
    uint32_t rsvd2;
    /** reserved3 */
    uint16_t op_sn;
    /** length of PDU */
    uint16_t msg_len;
    /** reserved4 */
    uint32_t rsvd3;
    /** reserved5 */
    uint32_t rsvd4;
} PACKED_STRUCT outbound_communication_header_t;


/** inboud communication header */
typedef struct inbound_communication_header {
    /** 0xADFF246C */
    uint32_t proto_id;
    /** see communication op_type 4bit */
    uint8_t  op_type   : 4;
    /**  current ver = 0 4 bit*/
    uint8_t  ver       : 4;
    /** see  communication op_code include data, event, control */
    uint8_t  op_code;
    /** command sequence number */
    uint16_t op_sn;
    /** reserved1 */
    uint32_t rsvd1;
    /** reserved2 */
    uint16_t rsvd2;
    /** length of PDU */
    uint16_t msg_len;
    /** reserved3 */
    uint32_t rsvd3;
    /** reserved4 */
    uint32_t rsvd4;
} PACKED_STRUCT inbound_communication_header_t;

/**
* @CN
* @brief Local info定义本地车辆/路边单元信息
*
* @EN
* @brief Local info define local vehicle/roadside unit information
* @END
*/
typedef struct local_info {
    /**
     * Range: -900000000..900000000 (-90..90) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t  lat;
    /**
     * Range -1800000000..1800000000 (-180..180) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t  lon;
    /** Range: 0..3600, 3600 shall be used when unavailable. Unit: 1/10 degree */
    uint16_t heading;
    /** 0.001 m/s */
    uint16_t speed;
    /** reserved1 */
    uint32_t rsvd1;
    /** reserved2 */
    uint32_t rsvd2;
    /** reserved3 */
    uint32_t rsvd3;
    /** reserved4 */
    uint32_t rsvd4;
} PACKED_STRUCT local_info_t;

/**
* @CN
* @brief Long Local info定义加长的本地车辆/路边单元信息
*
* @EN
* @brief Long Local info define extended local vehicle/roadside unit information
* @END
*/
typedef struct long_local_info {
    /**
     * Range: -900000000..900000000 (-90..90) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t  lat;
    /**
     * Range -1800000000..1800000000 (-180..180) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t  lon;
    /** Range: 0..3600, 3600 shall be used when unavailable. Unit: 1/10 degree */
    uint16_t heading;
    /** 0.001 m/s */
    uint16_t speed;
    /** 0: S 1: C 2: T Engineering mode */
    uint8_t  CR_movment;
    /** Engineering mode units: 0.1m/s */
    uint8_t  len1;
    /** Engineering mode units: 0.1m/s */
    uint8_t  width1;
    /** 0: C 1: T 2: S Engineering mode*/
    uint8_t  SR_movment;
    /** Engineering mode units: 0.1 degree */
    uint16_t deviate1;
    /** Engineering mode units: 0.1 degree */
    uint16_t deviate2;
    /** Engineering mode units: 0.1m/s */
    uint8_t  len2;
    /** Engineering mode units: 0.1m/s */
    uint8_t  width2;
    /** 
     * range：-32767..32767 （-3276.7度到3276.6度）, 单位0.1度，-3276.7度或超出（-32767），3276.6度或超出（32766），unavailable(32767)。
     * 方向盘转角，右转为正值。
     */
    int16_t SteeringWheel;
    /** Represent the accuracy which can be expected from a GNSS system in 1cm steps.
        Range: 0-4095, any value equal or greater than 40.94 meter(4094),unavailable(4095), no
        error(0). units: 1cm */
    uint16_t PositionAccuracy;
    /** Represent the accuracy which can be expected from a GNSS system in 1cm steps.
        Range: 0-4095, any value equal or greater than 40.94 meter(4094),unavailable(4095), no
        error(0). units: 1cm */
    uint16_t ElevationAccuracy;
    /** 0: ALL LIGHTS OFF 4: LEFT TURN SIGNAL ON 8: RIGHT TURN SIGNAL ON */
    uint8_t  lights;
    /** 0: None 4: 制动防抱死系统功能触发引发危险 8：牵引力控制系统功能触发引发危险 16：车身稳定性系统功能触发引发危险 64：车道偏移预警系统功能触发
        引发危险 128: Hard Braking 2048: The hazard lights are active. */
    uint16_t events;
    /** neutral (0), — 空挡 park (1), — 驻车挡 forwardGears (2), — 前进挡
        reverseGears (3), — 倒挡 reserved1 (4), reserved2 (5), reserved3 (6), unavailable (7) — 无效或未装配. */
    uint8_t  transmisn_state;
    /** Represent X-axis acceleration. Negative values indicate
        deceleration, and possible braking action. Range:
        -2000..2001, the value 2000 shall be used for values greater than 2000, the value
        -2000 shall be used for values less than -2000, avalue of 2001 shall be used
        for Unavailable. units: 0.01m/s^2 */
    int16_t acc_x;
    /** Represent Y-axis acceleration. Lateral acceleration is the
        acceleration along the Y axis or perpendicular to the
        vehicle’s general direction of travel in parallel with
        a left-to right centerline. Range: -2000..2001, the value 2000 shall be used for
        values greater than 2000, the value -2000 shall be used for values less than
        -2000, a value of 2001 shall be used for Unavailable. units: 0.01m/s^2 */
    int16_t acc_y;
    /** A data element representing the signed vertical
        acceleration of the vehicle along the vertical axis
        in units of 0.02 G (where 9.80665 meters per second
        squared is one G, i.e., 0.02 G = 0.1962 meters per second
        squared). Range: -127..127 (-2.52 to +2.54 G), for
        ranges >= 2.54 G(+127), for ranges < = 2.52 G(-126),unavailable(-127).
    */
    int8_t  acc_z;
    /** range：-32767..32767, unavailable(32767). yaw rate，represent a vehicle’s rotation about its vertical axis within。右转为正值。*/
    int16_t yaw_rate;
    /** reserved1 */
    uint8_t  rsvd1;

} PACKED_STRUCT long_local_info_t;

/**
* @CN
* @brief Remote info定义邻居车辆信息
*
* @EN
* @brief Remote info define remote vehicle information
* @END
*/
typedef struct remote_info {
    /** Remote OBU ID */
    uint32_t remote_id;
    /** 0-65536 Engineering mode 20bits*/
    uint32_t Orientation: 20;
    /** Range: 0..3600, 3600 shall be used when unavailable. Unit: 1/10 degree 12bits*/
    uint32_t heading    : 12;
    /** remote vehicle distance Units:0.1m */
    uint16_t distance;
    /** remote vehicle x direction distance Units:0.1m */
    int16_t  x10;
    /** remote vehicle x direction distance Units:0.1m */
    int16_t  x01;
    /** reserved1 */
    uint16_t filter_state;
    /** Range: -1800000000..1800000000, 0 shall be used when unavailable */
    int32_t  lon;
    /** Range:-900000000..900000000, 0 shall be used when unavailable */
    int32_t  lat;
    /** remote speed, unit:0.001m/s */
    uint16_t speed;
    /** remote type, 1:Normal Vehicle, 2:Emergency Vehicle, 3:Motorcycle */
    uint8_t  type;
    /** 判断是否修证 */
    uint8_t  is_correct;
    /**
      * Range: -4096..61439.
      * Represents the geographic position above or below the reference ellipsoid(typically WGS-84).
      * The number has a resolution of 1 decimeter and represents an asymmetric range of positive and negative
      * values. Any elevation higher than +6143.9 meters is represented as +61439.
      * Any elevation lower than -409.5 meters is represented as -4095. If the sending device does not know its elevation,
      * it shall encode the Elevation data element with -4096.
      */
    int32_t elevation;
    /** vehicle width , Range: 0..1023 ,  unit:  1cm  */
    uint16_t width;
    /** vehicle length, Range: 0.. 4095 ,  unit:  1cm  */
    uint16_t length;
    /** the timestamp, unit:  seconds  */
    uint32_t  tv_sec;
    /** the rest milliseconds,  unit:  milliseconds   */
    uint32_t  tv_millisec;
    /** 0：ALL LIGHTS OFF 4：LEFTTURN SIGNAL ON 8：RIGHT TURNSIGNAL ON */
    uint8_t   lights;
    /** 0：NONE 128：Hard Braking 2048：Disabled Vehicle */
    uint16_t  events;
    /** neutral (0), — 空挡 park (1), — 驻车挡 forwardGears (2), — 前进挡
        reverseGears (3), — 倒挡 reserved1 (4), reserved2 (5), reserved3 (6), unavailable (7) — 无效或未装配. */
    uint8_t  transmisn_state;
    /** Represent X-axis acceleration. Negative values indicate
        deceleration, and possible braking action. Range:
        -2000..2001, the value 2000 shall be used for values greater than 2000, the value
        -2000 shall be used for values less than -2000, avalue of 2001 shall be used
        for Unavailable. units: 0.01m/s^2 */
    int16_t acc_x;
    /** Represent Y-axis acceleration. Lateral acceleration is the
        acceleration along the Y axis or perpendicular to the
        vehicle’s general direction of travel in parallel with
        a left-to right centerline. Range: -2000..2001, the value 2000 shall be used for
        values greater than 2000, the value -2000 shall be used for values less than
        -2000, a value of 2001 shall be used for Unavailable. units: 0.01m/s^2 */
    int16_t acc_y;
    /** A data element representing the signed vertical
        acceleration of the vehicle along the vertical axis
        in units of 0.02 G (where 9.80665 meters per second
        squared is one G, i.e., 0.02 G = 0.1962 meters per second
        squared). Range: -127..127 (-2.52 to +2.54 G), for
        ranges >= 2.54 G(+127), for ranges < = 2.52 G(-126),unavailable(-127).
    */
    int8_t  acc_z;
    /** range：-32767..32767, unavailable(32767). yaw rate，represent a vehicle’s rotation about its vertical axis within。右转为正值。*/
    int16_t yaw_rate;
    /** range：-126..127 （-189度到189度）, -189度或超出（-126），189度或超出（126），unavailable(127)。
        方向盘转角，右转为正值。
    */
    int8_t  SteeringWheel;
    /** Represent the accuracy which can be expected from a GNSS system in 1cm steps.
        Range: 0-4095, any value equal or greater than 40.94 meter(4094),unavailable(4095), no
        error(0). units: 1cm */
    uint16_t PositionAccuracy;
    /** Represent the accuracy which can be expected from a GNSS system in 1cm steps.
        Range: 0-4095, any value equal or greater than 40.94 meter(4094),unavailable(4095), no
        error(0). units: 1cm */
    uint16_t ElevationAccuracy;
    /** reserved2 */
    uint32_t rsvd2;
#ifdef PERFORMANCE_TC_TA_TEST_ON   //for performance test
    uint16_t SecMark;
#endif
    /** reserved3 */
    uint32_t rsvd3;
    /** reserved4 */
    uint32_t rsvd4;
} PACKED_STRUCT remote_info_t;

/**
* @CN
* @brief 定义详细位置信息 use for Outside Vehicle Data and Outside Position Based pedestrian
*
* @EN
* @brief Define detailed location information use for Outside Vehicle Data and Outside Position Based pedestrian
* @END
*/
typedef struct full_position_info {
    /** target identifer */
    uint32_t id;
    /** Range: 0..3600, 3600 shall be used when unavailable. Unit: 1/10 degree */
    uint16_t heading;
    /** reserved1 */
    uint16_t rsvd1;
    /**
     * Range -1800000000..1800000000 (-180..180) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t lon;
    /**
     * Range: -900000000..900000000 (-90..90) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t lat;
    /** 0.001 m/s */
    uint16_t speed;
    /** 状态，根据不同消息有不同解释 */
    uint8_t  status;
    /** MsgCount, Range: 0...127 */
    uint8_t  MsgCount; // 更新传送 BSM消息集的 MsgCount
    /**
      * Range: -4096..61439.
      * Represents the geographic position above or below the reference ellipsoid(typically WGS-84).
      * The number has a resolution of 1 decimeter and represents an asymmetric range of positive and negative
      * values. Any elevation higher than +6143.9 meters is represented as +61439.
      * Any elevation lower than -409.5 meters is represented as -4095. If the sending device does not know its elevation,
      * it shall encode the Elevation data element with -4096.
      */
    int32_t elevation;
    /** vehicle width , Range: 0..1023 ,  unit:  1cm  */
    uint16_t width;
    /** vehicle length, Range: 0.. 4095 ,  unit:  1cm  */
    uint16_t length;
    /** the timestamp  when  IPU Identify the vehicle, Unix timestamp , Range: 0..4294967295 ,  unit:  seconds  */
    uint32_t  tv_sec;
    /** the rest miliseconds  of the IPU Unix timestamp , Range:0..999999999 ,  unit:  miliseconds   */
    uint32_t  tv_msec;
    /** Events, bit string, 0: None 1: The hazard lights2：Stop Line Violation
      * 4: 制动防抱死系统功能触发引发危险 8：牵引力控制系统功能触发引发危险
      * 16：车身稳定性系统功能触发引发危险 32: 危险品运输车 64：车道偏移预警系统功能触发引发危险
      * 128:Hard Braking are active 256:Light Changed 512: WipersChanged 1024: Flat Tire
      * 2048: 车辆故障 4096: Air BagDeployment. */
    uint16_t events;
    /** 车辆纵向加速度，加速为正值，unavailable(65535), 单位0.01m/s^2 */
    uint16_t longitudinal_acceleration;
    /** 远车BSM消息集中的时间戳，当前分钟的毫秒数 0-59999,60000:unavailable */
    uint16_t secmark;
    /** 数据来源， 0: Unknown, 1:Infomation from itself, 2:V2X, 3: Video, 4: microwaveradar, 5: loop */
    uint8_t source;
    /** reserved3 */
    uint8_t rsvd2;
    /** reserved2 */
    uint32_t rsvd3;
    /** reserved4 */
    uint32_t rsvd4;
    /** reserved5 */
    uint32_t rsvd5;
} PACKED_STRUCT full_position_info_t;

/** SPAT phase state info */
typedef struct phase_state {
    /** the basic 11 state type, see e_MovementPhaseState */
    uint8_t state;
    /** reserved1 */
    uint8_t rsvd1;
    /** duration for this state. Unit: second */
    uint16_t duration;
    /** reserved2 */
    uint32_t    rsvd2;
} PACKED_STRUCT phase_state_t;

/** SPAT signal group info */
typedef struct signal_group {
    /**
     * signal group id, used to map to lists of lanes.
     * The value 0 shall be used when the ID is not available or not known
     * the value 255 is reserved to indicate a permanent green signal state
     */
    uint8_t	id;
    /**
     * how many phases this signal group contains. range: 1..16
     */
    uint8_t  nr_phases;
    /** current active state. The basic 11 state type, see e_MovementPhaseState */
    uint8_t cur_state;
    /** reserved1 */
    uint8_t rsvd1;
    /* time remain for curent active state. Unit: 1/10 second */
    uint16_t cur_remain_time;
    /** reserved2 */
    uint16_t rsvd2;
    /** reserved3 */
    uint32_t rsvd3;
    /** reserved4 */
    uint32_t rsvd4;
    /**
     * Fixed 16 phases(only 'nr_phases' phases are used).
     * Each phase is given in turn, and it
     * may contain both active and future states.
     */
    phase_state_t phases_list[16];
} PACKED_STRUCT signal_group_t;

/** spat_reference_id include region id and intersection_id */
typedef struct spat_reference_id {
    /** a globally uniqueregional assignment value, the value zero shall be used for testing needs */
    uint16_t region_id;
    /** a unique mapping to the intersection within the above region of use */
    uint16_t intersection_id;
} PACKED_STRUCT spat_reference_id_t;

/**
* @CN
* @brief PDU of Vehicle Runtime Data (OP Code: 1)定义车辆实时数据流载荷，v2x
* service主动周期性发送，发送频率10HZ
*
* @EN
* @brief PDU of Vehicle Runtime Data (OP Code: 1) define the vehicle real-time data flow load,
* v2x service take the initiative to send periodically and the sending frequency is 10HZ.
* @END
*/
typedef struct vehicle_runtime_data {
    /** long local info */
    long_local_info_t local;
    /** number of remote */
    uint8_t           nr_remote;
    /** reseved1 */
    uint8_t           seq;
    /** reseved2 */
    uint16_t          total_remote;
    /** remote vechicles info number depends on numer_remote */
    remote_info_t     remote[];
} PACKED_STRUCT vehicle_runtime_data_t;



typedef struct ph_points_info {
    int32_t lonOffset;
    int32_t latOffset;
    int32_t elevation;
    uint16_t timeOffset;
    uint16_t speed;
    uint8_t posInPH: 4;
    uint8_t elevationInPH: 4;
    uint8_t heading;
    uint16_t rvsd0;
} PACKED_STRUCT ph_points_info_t;

typedef struct safety_ext_info {
    uint16_t events;
    uint16_t lights;
    uint16_t radiusOfCurve;
    uint8_t confidence;
    uint8_t PHPointsNr;//count of ph_point
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint16_t second;
    int16_t offset;
    uint16_t heading;
    int32_t latitude;
    int32_t longitude;
    uint32_t elevation;
    uint16_t speed;
    uint8_t transmission;
    uint8_t posInPosAcc: 4;
    uint8_t eleInPosAcc: 4;
    uint8_t timeCFD;
    uint8_t posInPosCFD: 4;
    uint8_t eleInPosCFD: 4;
    uint8_t speedCFD: 3;
    uint8_t headingCFD: 3;
    uint8_t steerCFD: 2;
    uint8_t currGNSSStatus;
    ph_points_info_t PHPointsList[];
} PACKED_STRUCT safety_ext_info_t;

typedef struct remote_bsm_pkg {
    uint8_t msgCnt;
    uint8_t ext;
    uint16_t secMark;
    uint8_t id[8];
    uint8_t timeCFD;
    uint8_t rsvd1;
    uint16_t rsvd2;
    int32_t latitude;
    int32_t longitude;
    int32_t elevation;
    uint8_t semiMajor;
    uint8_t semiMinor;
    uint16_t semiMajorOrient;
    uint8_t posCFD: 4;
    uint8_t elevationCFD: 4;
    uint8_t transmission;
    uint16_t speed;
    uint16_t heading;
    int8_t angle;
    uint8_t speedCFD: 3;
    uint8_t headingCFD: 3;
    uint8_t steeringCFD: 2;
    int16_t longAccel;
    int16_t latAccel;
    int8_t verAccel;
    int16_t yawRate;
    uint8_t brakePadel: 2;
    uint8_t wheelBrakes: 6;
    uint8_t traction: 3;
    uint8_t abs: 3; //4.0协议中笔误写成了2bit，在1.18中修改一下
    uint8_t scs: 2;
    uint8_t brakeBoost: 4;
    uint8_t auxBrakes: 4;
    uint16_t width;
    uint16_t length;
    uint8_t height;
    uint8_t classification;
    uint32_t orientation: 20;
    uint8_t fuelType: 4;
    uint8_t responseType: 3;
    uint8_t sirenUse: 2;
    uint8_t lightsUse: 3;
    uint32_t rsvd3;
    uint32_t rsvd4;
    uint32_t rsvd5;
    uint32_t rsvd6;
    safety_ext_info_t safetyExt;
} PACKED_STRUCT remote_bsm_pkg_t;

typedef struct remote_bsm_info {
    uint16_t bsm_pkg_length;
    remote_bsm_pkg_t bsm_pkg;
} PACKED_STRUCT remote_bsm_info_t;

typedef struct vehicle_runtime_bsm_data {
    /** long local info */
    long_local_info_t local;
    /** number of remote */
    uint8_t           nr_remote;
    /** sequence of remote: range:0 ~ nr_remote-1*/
    uint8_t           sq_remote;
    /** reseved1 */
    uint16_t          rsvd1;
    /** remote vechicles info number depends on numer_remote */
    remote_bsm_pkg_t     remote_bsm;
} PACKED_STRUCT vehicle_runtime_bsm_data_t;

/**
* @CN
* @brief PDU of System Info Data (OP Code: 2)定义V2X Service当前信息数据流载荷，
* 由v2x service向外界发送
*
* @EN
* @brief PDU of System Info Data (OP Code: 2) define V2X Service current message data flow load,
* Sent by the v2x service to outside.
* @END
*/
typedef struct system_info_data {
    /** v2x_service version */
    uint8_t  buf[MAX_VERSION_INFO];
    /** 1: Normal mode (Default) 2: Engineering mode */
    uint8_t  mode;
    /** 1: stopped 2: init 3:running 4: error */
    uint8_t  status;
    /** reserved1 */
    uint16_t rsvd1;
    /** reserved2 */
    uint32_t rsvd2;
    /** reserved3 */
    uint32_t rsvd3;
    /** reserved4 */
    uint32_t rsvd4;
    /** reserved5 */
    uint32_t rsvd5;
    /** reserved6 */
    uint32_t rsvd6;
    /** reserved7 */
    uint32_t rsvd7;
} PACKED_STRUCT system_info_data_t;

typedef struct rsa_information {
    /** Used when RSA function is on. 268: speed-limit(curve speedlimit) 401: road-worker 5897: slippery road
    2564: speed guidance 7203: speedy warning, warning while speed too fast */
    uint16_t    type;
    /** Limit speed value [km/s] */
    uint8_t     value;
    /** Range:0..8. 0 represents the 'background' information, not presented to the driver,
    8 represents the most urgent situation, < =2 represents slight，< =4 represents mediu，< =7 represents significant.*/
    uint8_t     priority;
    /** Used when RSA function is on. Alert coverage distance. (Engineering mode only). units: 1m*/
    uint16_t    distance;
    /** Used when RSA function is enabled. Indicate a gross range of the direction to which the
        enclosing message applies. */
    uint16_t     headingslice;
    /** an index link to any other incident information data. (internal use only) */
    uint16_t     further_id;
    /** reserved1 */
    uint16_t    rsvd1;
    /** Range: -1800000000..1800000000, 0shall be used when unavailable */
    int32_t     lon;
    /** Range: -900000000..900000000, 0 shall be used when unavailable */
    int32_t     lat;
    /** Range: -4096..61439. Represents the geographic position above or below the
        reference ellipsoid (typically WGS-84). The number has a
        resolution of 1 decimeter and represents an asymmetric range
        of positive and negative values. Any elevation higher than
        +6143.9 meters is represented as +61439. Any elevation
        lower than -409.5 meters is represented as -4095. If the
        sending device does not know its elevation, it shall encode the Elevation data element with -4096. */
    int32_t     elevation;

    uint16_t    heading;
    /** reserved2 */
    uint16_t    rsvd2;
    /** reserved3 */
    uint32_t    rsvd3;
} PACKED_STRUCT rsa_information_t;

/**
* @CN
* @brief PDU of System Function Data (OP Code: 3)定义V2X Service当前运行模式（OBU/
* RSU）以及RSU启动的功能，由v2x service向外界发送。
*
* @EN
* @brief PDU of System Function Data (OP Code: 3) define V2X Service current operating mode(OBU/RSU),
* and RSU-enabled features. Sent by the v2x service to outside.
* @END
*/
typedef struct system_function_data {
    /** 1: OBU mode 2: RSU mode */
    uint8_t     mode;
    /** Must be 0 in OBU mode. In RSU mode 0: None 1: RSA 2: SPAT 4: MAP 8: PSM 16: SIMBSM 32:SRMRx 64:BSMRx*/
    uint16_t    function;
    /** Used when RSA function is on. Number of Rsa information Range:0..8 */
    uint8_t     nr_rsa;
    /** 1: TYPE_ITISCODE 2:TYPE_GB 3:TYPE_ANYCATALOGUE  */
    uint8_t     signtype;
    /** reserved1 */
    uint8_t    rsvd1;
    /** reserved2 */
    uint16_t    rsvd2;
    /** reserved3 */
    uint32_t    rsvd3;
    /** reserved4 */
    uint32_t    rsvd4;
    /** reserved5 */
    uint32_t    rsvd5;
    /** Used when RSA function is on. include RSA type, RSA value , RSA priority Alert coverage distance */
    rsa_information_t rsa_information_list[];
} PACKED_STRUCT system_function_data_t;

typedef struct app_function_data {
    uint64    app_enabler;
} PACKED_STRUCT app_function_data_t;

/* v2x_service send simulated spat data to v2x_proxy */
#define simulated_spat_data inbound_realtime_spat_data
#define simulated_spat_data_t inbound_realtime_spat_data_t

/*  The PDU of Probe Vehicle Data is sent by the V2X service to
    external subsystems for update the surrounding vehicle information collected by a
    V2X device */
#define probe_vehicle_Data_t full_position_info_t

/** v2x_service recive inbound realtime spat data to v2x_proxy */
typedef struct inbound_realtime_spat_data {
    /** total number of intersections range: 1..32 */
    uint8_t                 nr_intersections;
    /** reserved1 */
    uint8_t                 rsvd1;
    /** reserved2 */
    uint16_t                rsvd2;
    /** see SPAT Reference ID */
    spat_reference_id_t     reference_id;
    /** reserved currently, general status of the controller */
    uint16_t                status;
    /** number of signal groups. range:1..255 */
    uint16_t                nr_signal_groups;
    /**
     * Each signal_group is given in turn and contains its signal phase state,
     * mapping to the lanes it applies to, and point in time it will end,
     * and it may contain both active and future states.
     */
    signal_group_t          signal_groups_list[];
} PACKED_STRUCT inbound_realtime_spat_data_t;

/** Pedestrian Camera Info */
typedef struct Pedestrian_info {
    /** y-axis coordinate unit: mm */
    uint32_t      dist_y;
    /** x-axis coordinate unit: mm */
    int32_t       dist_x;

} PACKED_STRUCT Pedestrian_info_t;

/**
* @CN
* @brief PDU of Outside Camera Based pedestrian Data (OP Code: 101)由外部子系统发送
* 给v2x service，用于传递pedestrian detection based on camera。基于视觉检测
* 的行人危险预警提供基于摄像头光心的局部坐标信息给v2x service
*
* @EN
* @brief PDU of Outside Camera Based pedestrian Data (OP Code: 101) is sent by the external
* subsystem to v2x service, to transfer pedestrian detection based on camera.
* Pedestrian collision warning based on visual inspection provides local coordinate information
* based on camera's optical center to v2x service
* @END
*/
typedef struct inbound_camera_pedestrian_data {
    /** total number of Pedestrians detected, range: 1..255 */
    uint8_t                 num;
    /** reserved2 */
    uint8_t                 rsvd1;
    /** reserved2 */
    uint16_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** pedestrian camera info */
    Pedestrian_info_t       pedestrian_list[];
} PACKED_STRUCT inbound_camera_pedestrian_data_t;

/**
* @CN
* @brief PDU of Outside GNSS Data (OP Code: 104)由外部子系统发送
* 给v2x service，发送频率为10hz，用于传递外部的GNSS数据。
*
* @EN
* @brief PDU of Outside GNSS Data (OP Code: 104) is sent by the external
* subsystem to v2x service, the frequency of transmission is 10hz, to transfer GNSS data.
* @END
*/
typedef struct inbound_gnss_data {
    /**
     * Range -1800000000..1800000000 (-180..180) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t lon;
    /**
     * Range: -900000000..900000000 (-90..90) Units: 1/10 micro degree
     * 0 shall be used when unavailable
     */
    int32_t lat;
    /** Altitude in meters units: 0.01 meter */
    int32_t                 alt;
    /** Longitude position uncertainty, units: 0.01meter */
    int32_t                 epx;
    /** Latitude position uncertainty, units: 0.01meter */
    int32_t                 epy;
    /** Track uncertainty, units: 0.01degree */
    int32_t                 epd;
    /** Seconds since the Epoch */
    uint32_t                timestamp_seconds;
    /** Miliseconds since the last Second */
    uint32_t                timestamp_miliseconds;
    /** Range: 0..3600, 3600 shall be used when unavailable. Unit: 1/10 degree */
    uint16_t                heading;
    /** 0.001 m/s */
    uint16_t                speed;
    /** acc_x:  X-axis acceleration units: 0.001m/s^2 */
    int32_t                 acc_x;
    /** acc_y:  Y-axis acceleration units: 0.001m/s^2 */
    int32_t                 acc_y;
    /** acc_z:  Z-axis acceleration units: 0.001m/s^2 */
    int32_t                 acc_z;
    /** gyro_x: pitch   units: 0.001rad/s */
    int32_t                 gyro_x;
    /** gyro_y: roll    units: 0.001rad/s */
    int32_t                 gyro_y;
    /** gyro_z: yaw     units: 0.001rad/s */
    int32_t                 gyro_z;
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT inbound_gnss_data_t;

/**
 * @CN
* @brief PDU of CAN Data (OP Code: 200) 由外部子系统发送给V2X SERVICE，用于提供本车
* 的CAN总线信息。当车辆CAN总线发送数据流，由外部子系统解析CAN数据，并即时以
* 此消息形式发送给V2X SERVICE。其中，speed、heading等车辆实时数据的更新频率
* 不低于10HZ。当有事件类CAN数据时，外部子系统以触发模式，即时向V2X SERVICE发
 * 送此消息
 *
 * @EN
 * @brief PDU of CAN Data (OP Code: 200) is sent to V2X SERVICE by the external subsystem to provide the car
 * CAN bus information. When the vehicle CAN bus sends the data stream, the external subsystem parses the CAN data and instantly
 * This message form is sent to V2X SERVICE. Where, speed, heading and other real-time vehicle data update frequency
 * no less than 10HZ. When there is an event class CAN data, The external subsystem sends this message to V2X SERVICE in real time in triggering mode.
 *
 *
 * @END
*/
typedef struct inbound_can_data {

    /** 车速, unavailable(65535) 0.001 m/s */
    uint16_t                speed;
    /** 后轮转数，unavailable(65535) */
    uint16_t                rear_wheel_tick;
    /** range：-32767..32766 （-3276.7度到3276.6度）, 单位0.1度， -3276.7度或超出（-32767），3276.6度或超出（-3276.6），unavailable(32767)。
        方向盘转角，右转为正值。
    */
    int16_t                steering_wheel_angle;
    /** range: 0..3600, 3600 shall be used when unavailable. 单位0.1degree */
    uint16_t                heading;
    /** neutral (0), — 空挡
        park (1), — 驻车挡
        forwardGears (2), — 前进挡
        reverseGears (3), — 倒挡
        reserved1 (4),
        reserved2 (5),
        reserved3 (6),
        unavailable (7) — 无效或未装配 【事件类数据】 */
    uint8_t                 transmisn;
    /** 车灯状态，可以多种灯同时ON.
        0:ALL LIGHTS OFF
        1:hazarlight on(车辆警示灯亮起,【汽车双跳灯闪】)
        2:brake lighton(刹车灯)
        4: left turn light on
        8: right turn light on
        16:fault light on(故障灯：发动机故障、机油灯等，提示驾驶员需要维修车辆)
        32:parking light on(停车灯)
        64:Backup Light on(倒车灯)
        128:reserved 256:reserved,
        unavailable(65535).【事件类数据】*/
    uint16_t                light;
    /** 车辆ESP状态，0: unavailable,1: off, 2: On but notEngaged, 3: Engaged.【事件类数据】 */
    uint8_t                 ESP_status;
    /** 车辆TCS状态，0: unavailable,1: off, 2: On but notEngaged, 3: Engaged.【事件类数据】 */
    uint8_t                 TCS_status;
    /** 车辆ABS状态，0: unavailable,1: off, 2: On but notEngaged, 3: Engaged.【事件类数据】 */
    uint8_t                 ABS_status;
    /** 车辆LDW状态，0: unavailable,1: off, 2: On but notEngaged, 3: Engaged.【事件类数据】 */
    uint8_t                 LDW_status;
    /** 车辆刹车踏板状态，0:unavailable, 1: not pressed, 2: Normal Breaking, 3:Emergency Breaking.【事件类数据】 */
    uint8_t                 brake_pedal;
    /** 车辆刹车踏板百分比，0-100(0%-100%), 101:unavailable.【事件类数据】 */
    uint8_t                 pedal_status;
    /** 车辆失控状态，unavailable, 1: off, 2: On【事件类数据】 */
    uint8_t                 lose_contorl;
    /** 车辆轮胎气压状态， 0x00 :all normal 0x01 :fl warning 0x02:fr warning 0x04:rl warning 0x08:rr warnin 【事件类数据】*/
    uint8_t                 tire_pressure;
    /** reserved1 */
    uint8_t                 rsvd1;
    /** 车辆X轴方向加速度，加速为正值，unavailable(32767) 单位：0.001m/s^2 */
    int32_t                acc_x;
    /** 车辆Y轴方向加速度，加速为正值，unavailable(32767) 单位：0.001m/s^2 */
    int32_t                acc_y;
    /** 车辆Z轴方向加速度，加速为正值，unavailable(32767) 单位：0.001m/s^2 */
    int32_t                acc_z;
    /** 车辆俯仰角角加速度，加速为正值，unavailable(32767) 单位：0.001rad/s */
    int32_t                gyro_x;
    /** 车辆翻滚角角加速度，加速为正值，unavailable(32767) 单位：0.001rad/s */
    int32_t                gyro_y;
    /** 车辆偏航角角加速度，加速为正值，unavailable(32767) 单位：0.001rad/s */
    int32_t                gyro_z;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
    /** reserved6 */
    uint32_t                rsvd6;
} PACKED_STRUCT inbound_can_data_t;

/** TC标定数据中包含的字段 */
typedef struct TC_calibration_data {
    /** 最大分类距离，超过该距离的车辆不分类，默认值7000，65535为unavailable，
    范围为[1000,20000]。*/
    uint16_t                tc_max_range_dm;
    /** 远车最大生命周期，远车信息中记录的时间与当前时间差超过该值，此条远车信息不处理，
    默认值5000，65535为unavailable，范围为(0,50000]。*/
    uint16_t                max_remote_age_ms;
    /** 同道同向车辆判定参数，远车与本车的横向距离绝对值需要在该值范围内，
    默认值1750，65535为unavailable，范围为(0,20000]。*/
    uint16_t                ahead_lim_mm;
    /** 同向左邻道的车辆判定参数，远车与本车的左侧横向距离需要在该值与AheadLimit_mm之间，
    默认值5250，65535为unavailable，范围为(0,40000]。*/
    uint16_t                ahead_left_lim_mm;
    /** 同向右邻道车辆判定参数，远车与本车的右侧横向距离需要在该值与AheadLimit_mm之间，
    默认值5250，65535为unavailable，范围为(0,40000]。*/
    uint16_t                ahead_right_lim_mm;
    /** 对向车辆判定参数，远车与本车的横向距离需在该值范围内，默认值1975，
    65535为unavailable，范围为(0,20000]。*/
    uint16_t                oncoming_lim_mm;
    /** 对向左邻道判定参数，远车与本车的左侧横向距离需在该值和OnComingLim_mm之间，
    默认值5475，65535为unavailable，范围为(0,40000]。*/
    uint16_t                oncoming_left_lim_mm;
    /** 海拔差，若远车与本车的海拔差超过该值，不计算远车分类，默认值2000，
    65535为unavailable，范围为(0,60000]。*/
    uint16_t                same_elev_lim_cm;
    /** 对向右邻道判定参数，远车与本车的右侧横向距离需在该值和OnComingLim_mm之间，
    默认值5475，65535为unavailable，范围为(0,40000]。*/
    uint16_t                oncoming_right_lim_mm;
    /** 同向车辆判断参数，若远车和本车的heading差绝对值在该范围内，为同向车辆，
    默认值20，255为unavailable，范围为(0,90)。*/
    uint8_t                 same_hdg_lim_deg;
    /** 逆向车辆判定参数，若远车与本车的heading差在（180-OppHdgLim_deg，180+OppHdgLim_deg）范围内，
    则为逆向车辆，默认值20，255为unavailable，范围为(0,90)。*/
    uint8_t                 opp_hdg_lim_deg;
    /** 判断车辆进入弯道的阈值，车辆在一定时间内，连续转弯超过该值，认为进入弯道，默认值100，
    65535为unavailable，范围为(0,1800]。*/
    uint16_t                turn_ang_lim_deg;
    /** 进入弯道后判断出弯的阈值，车辆在一定时间内，车辆转角小于该值，认为出弯道，默认值50，
    65535为unavailable，范围为(0,1800]。*/
    uint16_t                ang_add_lim_deg;
    /** 进入弯道和出弯道的时间阈值，该值表示最大采点个数，若数据10Hz更新，则50代表5秒，默认值50，
    255为unavailable，范围为(0,200]。*/
    uint8_t                 ang_add_cnt_lim;
    /** reserved1 */
    uint8_t                 rsvd1;
    /** reserved2 */
    uint16_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
    /** reserved6 */
    uint32_t                rsvd6;
    /** reserved7 */
    uint32_t                rsvd7;
    /** reserved8 */
    uint32_t                rsvd8;
    /** reserved9 */
    uint32_t                rsvd9;
    /** reserved10 */
    uint32_t                rsvd10;
    /** reserved11 */
    uint32_t                rsvd11;
    /** reserved12 */
    uint32_t                rsvd12;
    /** reserved13 */
    uint32_t                rsvd13;
    /** reserved14 */
    uint32_t                rsvd14;
} PACKED_STRUCT TC_calibration_data_t;

/** TA标定数据中所有应用场景通用的字段 */
typedef struct TA_Common {
    /** 最大预警距离，远车与本车的距离超过该值，不预警，默认值7000，65535为unavailable，范围为[1000,20000]。*/
    uint16_t                ta_max_range_dm;
    /** 车道宽度，默认值3600，65535为unavailable，范围为(0,40000]。*/
    uint16_t                lane_width_mm;
    /** 紧急刹车减速度，默认值-600，32767为unavailable，范围为[-2000,0)。*/
    int16_t                hard_brake_cmps2;
    /** 轻踩刹车减速度，默认值-300，32767为unavailable，范围为[-2000,0)。*/
    int16_t                soft_brake_cmps2;
    /** 反应时间，默认值16200，65535为unavailable，范围为(0,20000]。*/
    uint16_t                reaction_time_ms;
    /** 安全速度，低于该速度不预警，默认值4，65535为unavailable，范围为(0,20]。*/
    uint16_t                ta_safe_spd_mps;
    /** 自车长度，默认值4079，65535为unavailable，范围为(0,40000]。*/
    uint16_t                self_len_mm;
    /** 自车宽度，默认值1539，65535为unavailable，范围为(0,10000]。*/
    uint16_t                self_width_mm;
    /** 车道判断时，判定车道的阈值，车辆与车道中心线的距离大于该值，不识别车道，默认值500，
    65535为unavailable，范围为(0,4000]。*/
    uint16_t                lane_lv_thr_cm;
    /** 车道判断时，判定路段的阈值，车辆与路段中心线的距离大于该值，不识别路段，默认值1500，
    65535为unavailable，范围为(0,10000]。*/
    uint16_t                link_lv_thr_cm;
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT TA_Common_t;

/** TA标定数据中应用于FCW的字段 */
typedef struct TA_FCW {
    /** FCW预警距离，默认值3000，65535为unavailable，范围为[1000,20000]。*/
    uint16_t                fcw_op_range_dm;
    /** FCW制动协调时间t1，默认值为500，65535为unavailable，范围为(0,10000]。*/
    uint16_t                fcw_brake_t1_ms;
    /** FCW减速度增长时间t2，默认值为200，65535为unavailable，范围为(0,10000]。*/
    uint16_t                fcw_brake_t2_ms;
    /** FCW预设加速度as，暂时算法没有用到，后续可进行标定，65535为unavailable，范围为(0,2000]。*/
    uint16_t                fcw_deft_accel_cmps2;
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT TA_FCW_t;

/** TA标定数据中应用于EBW的字段 */
typedef struct TA_EBW {
    /** EBW预警距离，默认值300，65535为unavailable，范围为[100,2000]。*/
    uint16_t                max_eebl_range_m;
    /** reserved1 */
    uint16_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
    /** reserved6 */
    uint32_t                rsvd6;
} PACKED_STRUCT TA_EBW_t;

/** TA标定数据中应用于BSW的字段 */
typedef struct TA_BSW {
    /** 盲区范围，远车在本车后方邻道，且纵向距离小于该值，为盲区范围，默认值2000，
    65535为unavailable，范围为(0,20000]。*/
    uint16_t                bsw_thr_cm;
    /** reserved1 */
    uint16_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
    /** reserved6 */
    uint32_t                rsvd6;
} PACKED_STRUCT TA_BSW_t;

/** TA标定数据中应用于ICW的字段 */
typedef struct TA_ICW {
    /** 十字路口碰撞时间，两车达到碰撞点的时间差超过该值，不预警，默认值5000，
    65535为unavailable，范围为(0,60000]。*/
    uint16_t                icw_ttc_s;
    /** 十字路口碰撞时间，两车达到碰撞点的时间差超过该值，不预警，默认值15000，
    65535为unavailable，范围为(0,60000]。*/
    uint16_t                icw_safe_time_ms;
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT TA_ICW_t;

/** TA标定数据中应用于DNPW的字段 */
typedef struct TA_DNPW {
    /** DNPW soft危险级别， 0 <（实际距离-安全距离）/安全距离 < DnpwSoft_level，
    默认值-10，127为unavailable，范围为(-100,0]。*/
    int8_t                 dnpw_soft_lv;
    /** DNPW hard危险级别，DnpwSoft_level <（实际距离-安全距离）/安全距离 < DnpwHard_level，
    默认值-40，127为unavailable，范围为(-100,0]。*/
    int8_t                 dnpw_hard_lv;
    /** reserved1 */
    uint16_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
    /** reserved6 */
    uint32_t                rsvd6;
} PACKED_STRUCT TA_DNPW_t;

/** TA标定数据中应用于CCW的字段 */
typedef struct TA_CCW {
    /** 安全区域宽度，默认值200，65535为unavailable，范围为(0,4000]。*/
    uint16_t                safe_zone_width_cm;
    /** 安全区域长度，默认值1500，65535为unavailable，范围为(0,10000]。*/
    uint16_t                safe_zone_length_cm;
    /** 安全区域碰撞时间，默认值10000，65535为unavailable，范围为(0,60000]。*/
    uint16_t                safe_zone_collision_time_ms;
    /** 是否开启弯道预警，默认值1，0:unavailable，1:off，2:On */
    uint8_t                 ccw_switch;
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT TA_CCW_t;

/** TA标定数据中应用于LTA的字段 */
typedef struct TA_LTA {
    /** 左转辅助安全距离，本车转弯需要的距离，默认值10000，65535为unavailable，范围为(0,60000]。*/
    uint16_t                lta_safe_dist_m;
    /** 左转辅助，车辆与本车的碰撞时间小于该值，预警，默认值10000，65535为unavailable，范围为(0,60000]。*/
    uint16_t                lta_ttc_s;
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT TA_LTA_t;

/** TA标定数据中应用于GLOSA的字段 */
typedef struct TA_GLOSA {
    /** 最大的、舒适的加速度，默认值300，65535为unavailable，范围为(0,1000]。*/
    uint16_t                max_accel_cmft;
    /** 普通的、舒适的加速度，默认值150，65535为unavailable，范围为(0,1000)。*/
    uint16_t                mid_accel_cmft;
    /** 最大的、舒适的减速度，默认值-300，32767为unavailable，范围为[-2000,0)。*/
    int16_t                 max_decel_cmft;
    /** 普通的、舒适的减速度，默认值-150，32767为unavailable，范围为(-2000,0)。*/
    int16_t                 mid_decel_cmft;
    /** 速度建议时需要的反应时间，默认值4000，65535为unavailable，范围为(0,20000]。 */
    uint16_t                spd_adv_react_ms;
    /** reserved1 */
    uint8_t                 rsvd1;
    /** reserved2 */
    uint16_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
    /** reserved6 */
    uint32_t                rsvd6;
    /** reserved7 */
    uint32_t                rsvd7;
} PACKED_STRUCT TA_GLOSA_t;

/** TA标定数据中应用于RLVW的字段 */
typedef struct TA_RLVW {
    /** reserved1 */
    uint32_t                rsvd1;
    /** reserved2 */
    uint32_t                rsvd2;
    /** reserved3 */
    uint32_t                rsvd3;
    /** reserved4 */
    uint32_t                rsvd4;
    /** reserved5 */
    uint32_t                rsvd5;
} PACKED_STRUCT TA_RLVW_t;

/** TA标定数据字段，分为所有场景都用到的字段和各个应用场景用到的字段 */
typedef struct TA_calibration_data {
    TA_Common_t             ta_common;
    TA_FCW_t                ta_fcw;
    TA_EBW_t                ta_ebw;
    TA_BSW_t                ta_bsw;
    TA_ICW_t                ta_icw;
    TA_DNPW_t               ta_dnpw;
    TA_CCW_t                ta_ccw;
    TA_LTA_t                ta_lta;
    TA_GLOSA_t              ta_glosa;
    TA_RLVW_t               ta_rlvw;
} PACKED_STRUCT TA_calibration_data_t;

/**
* @CN
* @brief PDU of Calibration Data (OP Code: 201)定义V2X Service当前系统中标定数据流载荷，
* 由v2x service向外界发送
*
* @EN
* @brief PDU of Calibration Data (OP Code: 201) define V2X Service current calibration data flow load,
* Sent by the v2x service to outside.
* @END
*/
typedef struct calibration_data {
    /** 用于TC标定数据的结构体，具体内容参见TC_calibration_data_t定义*/
    TC_calibration_data_t   tc_cal_data;
    /** 用于TA标定数据的结构体，具体内容参见TA_calibration_data_t定义*/
    TA_calibration_data_t   ta_cal_data;
} PACKED_STRUCT calibration_data_t;

typedef struct calibration_data calibration_data_set_t;

/**
 * @CN
 * @brief PDU of Outside Roadside Safety Message Data (OP Code: 105)
 * 由外部子系统发送给V2X service，用于传递周边交通参与者的实时状态信息，包括
 * 路测单元本身、周围车辆、非机动车、行人等。
 * 1. 当前版本，此消息仅适用于中国标准，RSU端
 * 2. service收到消息后，即时以RSM消息集广播
 * 3. 用于一帧数据中，包含多个不同种类交通参与者的情况
 * 4. 建议频率：10HZ
 * @EN
 * @brief PDU of Outside Roadside Safety Message Data (OP Code: 105) is sent to V2X SERVICE by the external subsystem to provide the Traffic participant information.
 * data update frequency = 10HZ.
 * @END
 */
typedef struct inbound_rsm_data {
    /** 交通参与者的个数, range:1..16 */
    uint8_t                  nr_participants;
    /** msg count, range: 0…127 */
    uint8_t                  msgcount;
    /** reserved2 */
    uint16_t                 rsvd2;
    /** 交通参与者信息列表，其中"status”字段代表交通参与者类型，0: Unknown, 1:Motor, 2: Non-motor, 3:Pedestrian, 4: RSU */
    full_position_info_t     full_position_info[];
} PACKED_STRUCT inbound_rsm_data_t;

/**  v2v event data */
typedef struct v2v_event {
    /** engineering model local info */
    long_local_info_t        local;
    /** remote info */
    remote_info_t            remote;
    /** Refer to the definition of event type  */
    uint8_t                  type;
    /** Alert level 1 - SOFT; 2 - HARD; 3 -SEVERE */
    uint8_t                  level;
    /** reserved1 */
    uint16_t                 rsvd1;
    /** reserved2 */
    uint32_t                 rsvd2;
    /** reserved3 */
    uint32_t                 rsvd3;
    /** reserved4 */
    uint32_t                 rsvd4;
} PACKED_STRUCT v2v_event_t;

typedef struct speed_advice {
    /** unit：0.02m/s **/
    uint16_t speed_value;
    uint16_t type;
} PACKED_STRUCT speed_advice_t;

typedef struct lane_info {
    uint8_t                  lane_id;
    /** distance units: 0.1m  与停止线的距离      */
    uint16_t                 distance;
    /** singnal id Engineering mode */
    uint8_t                  signal_id;
    /** state   bit 4   */
    uint16_t                 state          : 4;
    /** allowedManeuver bit 12 */
    uint16_t                 allowedManeuver: 12;
    /** curremaintime units: second */
    uint16_t                 curremaintime;
    /** lane id Engineering mode */
    /** 停止线 Latitude, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t                  latitude;
    /** remote Longtitude, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t                  longtitude;
    /** speed_advice **/
    speed_advice_t           speed_advice;
    /** 0：Unknown 1：该车道为本车所在车道 2: 该车道不为本车所在车道 */
    uint8_t                  occupied;
    /** reserved1 */
    uint8_t                  rsvd1;
    /** reserved2 */
    uint16_t                 rsvd2;
    /** reserved3 */
    uint32_t                 rsvd3;
} PACKED_STRUCT lane_info_t;

/**  spat event data */
typedef struct spat_event {
    /** see local info */
    local_info_t             local;
    /** remote ID */
    uint32_t                 remote_id;
    /** vertical Engineering mode units: 0.01m/s */
    uint16_t                 vertical;
    /** intersection id Engineering mode */
    uint16_t                 intersection_id;
    /** 十字路口中心点纬度, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t                  spat_latitude;
    /** 十字路口中心点经度, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t                  spat_longtitude;
    /** red light violate**/
    uint8_t                  red_light_violate;
    /** traffic light control request result, message from signal control system in the intersection**/
    uint8_t                  request_result;
    /** 本车当前行进方向上车道的数量 */
    uint8_t                  lane_num;
    /** 闯红灯预警的level */
    uint8_t                  red_light_violate_level;
    /** Lane Info描述了当前车辆行进方向上所有车道信息以及对应的信号灯信息 */
    lane_info_t              laneinfo[];
} PACKED_STRUCT spat_event_t;

typedef struct warning_path
{
    /** warning point Longtitude, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t lon;    
    /** warning point Latitude, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t lat;
	/** elevation*/
	int32_t elev;

} PACKED_STRUCT warning_path_t;

/**  rsa event  data */
typedef struct rsa_event {
    /** see local info */
    local_info_t             local;
    /** remote ID */
    uint32_t                 remote_id;
    /** Refer to the definition of event type  */
    uint16_t                 type;
    /** Limit speed value, units: 1km/h */
    uint8_t                  value;
    /** Priority 0 represents the 'background' information, not presented to the driver,
    8 represents the most urgent situation, < =2 represents slight，< =4 represents mediu，< =7 represents significant.*/
    uint8_t                  priority;
    /** reservd2 */
    uint16_t                 rsvd1;
    /** distance  units: 1m */
    uint16_t                 distance;
    /** remote Latitude, -1800000000..1800000001 (-180..180)，0 represents unknow */
    int32_t                  latitude;
    /** remote Longtitude, -1800000000..1800000001 (-180..180)，0 represents unknow*/
    int32_t                  longtitude;
    /** description*/
    uint8_t description[256];
    /** The number of warning path position */
    uint8_t                  path_num;
    /** level */
    uint8_t                  level;
    /** reservd3 */
    uint16_t                 rsvd2;
    /** reservd4 */
    uint32_t                 rsvd3;
    /** warning path */
    warning_path_t           path[];
} PACKED_STRUCT rsa_event_t;

/**  v2p event  data */
typedef struct v2p_event {
    /** see local info */
    local_info_t             local;
    /** remote ID */
    uint32_t                 remote_id;
    /** cm mean distance between host vehicle and pedestrian */
    uint16_t                 distance;
    /** 0: unknown, 1:ahead, 2:ahead left, 3:far left, 4:ahead right, 5:far right **/
    uint8_t                  orientation;
    /** reservd1 */
    uint8_t                  level;
    /** 弱势交通参与者的纬度信息，Range:-900000000..900000001(-90..90) ，0表示未知 */
    int32_t                  lat;
    /** 弱势交通参与者的经度信息，Range:-1800000000..1800000001(-180..180)，0表示未知 */
    int32_t                  lon;
    /** 弱势交通参与者的行进方向，Range: 0..3600, 3600表示未知, unit: 0.1 deg */
    uint16_t                 heading;
    /** 弱势交通参与者的速度, unit: 0.001m/s */
    uint16_t                 speed;
    /** 弱势交通参与者的类型，1: 行人，2：自行车， 3：摩托车，4：其他 */
    uint8_t                  type;
    /** reservd2 */
    uint8_t                  rsvd1;
    /** reservd3 */
    uint16_t                 rsvd2;
} PACKED_STRUCT v2p_event_t;

/** cancel event data */
typedef struct cancel_event {
    /** see local info */
    local_info_t             local;
    /** remote ID */
    uint32_t                 remote_id;
    /** Refer to the definition of event type  */
    uint16_t                 type;
    /** reservd1 */
    uint32_t                 rsvd1;
    /** reservd2 */
    uint32_t                 rsvd2;
    /** reservd3 */
    uint32_t                 rsvd3;
} PACKED_STRUCT cancel_event_t;

/** error event data */
typedef struct error_event {
    /** error level  see DIAGNOSIS_SEVERITY */
    uint8_t    severity;
    /** error category see DIAGNOSIS_CATEGORY */
    uint8_t    category;
    /** error fault code see diag_info_table[] */
    uint16_t   fault_code;
    /** error process id  see vtk_event_diag_module_id_e*/
    uint8_t    module_id;
    uint8_t    rsvd1;
    uint16_t   rsvd2;
    /** description*/
    uint8_t    buf[MAX_ERROR_DESCRIPTION];
} PACKED_STRUCT error_event_t;

typedef struct diag_event {
    /** diag level  see DIAGNOSIS_SEVERITY */
    uint8_t    severity;
    /** diag category see DIAGNOSIS_CATEGORY */
    uint8_t    category;
    /** diag fault code see diag_info_table[] */
    uint16_t   fault_code;
    /** diag process id  see vtk_event_diag_module_id_e*/
    uint8_t    module_id;
    uint8_t    rsvd1;
    uint16_t   rsvd2;
    /** description see diag_info_table[]*/
    uint8_t    buf[MAX_ERROR_DESCRIPTION];
} PACKED_STRUCT diag_event_t;
typedef struct diag_info_data {
    uint8_t    diag_sum;
    uint8_t    reserved1;
    uint16_t   reserved2;
    diag_event_t diag_info_list[];

} PACKED_STRUCT diag_info_data_t;
/** ack event data */
typedef struct ack_event {
    /** see op_code */
    uint8_t    op_code;
    /** The command serial number uniquely identifies an executing command */
    uint16_t   op_sn;
    /** result 0 success */
    uint8_t    result;
    /** description */
    uint8_t    buf[MAX_ACK_DESCRIPTION];
} PACKED_STRUCT ack_event_t;

/** start up event data*/
typedef struct startup_state {
    /** see v2x service state vtk_service_id_e*/
    uint8_t service_id;
    /** see startup_state_type */
    uint8_t startup_type;
    /** result 0 success */
    uint8_t result;
    /** reservd1 */
    uint8_t   rsvd1;
    /** reservd2 */
    uint32_t  rsvd2;
} PACKED_STRUCT startup_state_t;

/** Set Engineer Mode data */
typedef struct engineer_mode {
    /** 1: Normal mode (Default) 2: Engineering mode*/
    uint8_t    mode;
    /** password 0xFD23A8 */
    int8_t     password[3];
} PACKED_STRUCT engineer_mode_t;

/** Set record log data */
typedef struct record_log {
    /** 1: Start recording 2: Stop recording */
    uint8_t   mode;
    /** reservd1 */
    uint8_t   rsvd1;
    /** reservd2 */
    uint16_t  rsvd2;
} PACKED_STRUCT record_log_t;

/** Set vechicle state data */
typedef struct vechicle_state {
    /** 0：ALL LIGHTS OFF 4：LEFTTURN SIGNAL ON 8：RIGHT TURNSIGNAL ON */
    uint8_t   lights;
    /** 0：NONE 128：Hard Braking 2048：Disabled Vehicle */
    uint16_t  events;
    /** reservd1 */
    uint8_t   rsvd1;
    /** Local vehicle types. 0: (default)normal vehicle 9729: defalut emergency-vehicle-units
    (to be used as unknown emergency vehicle) 9730: federal-law-enforcement */
    uint16_t  groups;
    /** reservd2 */
    uint16_t  rsvd2;
    /** srm send flag: 0 for cancel, 1 for start */
    uint8_t   signal_request;
    /** reservd3 */
    uint8_t  rsvd3;
    /** reservd4 */
    uint16_t  rsvd4;

} PACKED_STRUCT vechicle_state_t;

/** heartbeat event*/
typedef  struct rsu_config_info {
    uint8_t   service_id;
    uint8_t   module_id;
    uint8_t   result;
    uint8_t   reserved1;
    uint8_t   reserved2;
} PACKED_STRUCT rsu_config_info_t;

/** set system function data*/
typedef struct system_function_set {
    /** 1: OBU mode 2：RSU mode */
    uint8_t   mode;
    /** Must be 0 in OBU mode. In RSU mode 0: None 1: RSA 2: SPAT 4: MAP 8: PSM 16: SIMBSM */
    uint16_t  function;
    /** reservd1 */
    uint8_t   rsvd1;
} PACKED_STRUCT system_function_set_t;

typedef struct  spat_control_request {
    uint16_t region_id;
    uint16_t intersection_id;
    uint8_t  signalgroup_id;
    uint8_t  state;
    uint16_t rest_time;// 该灯的剩余时间 ：uint : 0.1s
    uint8_t staus;//signal request status, 0:cancel, 1:request
    uint8_t rsvd1;
    uint16_t rsvd2;
    uint32_t rsvd3;//Unix timestamp, Unit:second
    uint32_t rsvd4;
} PACKED_STRUCT spat_control_request_t;

typedef struct powermanager_state {
    uint8_t service_id;
    uint8_t pm_state;
    uint8_t result;
    uint8_t msg_type;
    uint32_t  rsvd;
} PACKED_STRUCT powermanager_state_t;

/** heartbeat event*/
typedef  struct heartbeat_event {
    uint8_t   service_id;
    uint8_t   module_id;
    uint8_t   result;
    uint8_t   reserved1;
    uint8_t   reserved2;
} PACKED_STRUCT heartbeat_event_t;

/** evt pkg api from evt_msg to communcation */
typedef struct v2x_event {
    uint8_t  event_type;
    uint8_t  priority;
    uint32_t local_id;
    union {
        v2v_event_t     v2v;
        spat_event_t    *spat;
        rsa_event_t     *rsa;
        v2p_event_t     v2p;
        cancel_event_t  cancel;
        error_event_t   error;
        diag_event_t    diag;
    };
} PACKED_STRUCT v2x_event_t;

/** all evt pkg api from evt_msg to communcation */
typedef struct {
    uint8_t         type;
    uint16_t        length;
    uint8_t         rsvd0;
    union {
        v2v_event_t     v2v;
        spat_event_t    *spat;
        rsa_event_t     *rsa;
        v2p_event_t     v2p;
    };
} PACKED_STRUCT alert_evt_t;

typedef struct {
    uint8_t         event_num;
    uint8_t         rsvd0;
    uint8_t         rsvd1;
    uint8_t         rsvd2;
    alert_evt_t     events[];
} PACKED_STRUCT all_alert_evt_t;

/** data pkg api from v2x_dataplane to communcation */
typedef struct v2x_data {
    uint8_t  data_type;
    uint32_t local_id;
    uint16_t data_len;
    union {
        vehicle_runtime_data_t *vehicle;
        vehicle_runtime_bsm_data_t *bsm;
        system_info_data_t     info;
        system_function_data_t *func;
        simulated_spat_data_t  *spat;
        probe_vehicle_Data_t   probe;
        char                   *direct_data;
        uint8_t               *raw_data;
        calibration_data_t    cal_data;
        app_function_data_t   *app;
        diag_info_data_t      *diag_info;
    };
} PACKED_STRUCT v2x_data_t;

typedef struct extend_outbound_communication_header {
    /** 0xFEFE */
    uint16_t proto_id;
    /** 3 */
    uint16_t verson;
    /** length of PDU */
    uint16_t msg_len;
    /** payload中包含的Vehicle数量，当前版本默认为1，每条消息仅
     含1个远车，SPaT和MAP消息集，值默认为0 */
    uint16_t num;
    /** Seconds since the Epoch*/
    uint32_t timestamp_seconds;
    /** Miliseconds since the last Second */
    uint32_t timestamp_miliseconds;
    /** 101 BSM, 102 SPaT, 103 MAP */
    uint16_t msg_type;
    /** reserved1 */
    uint16_t rsvd1;
    /** reserved2 */
    uint32_t rsvd2;
} PACKED_STRUCT extend_outbound_communication_header_t;

typedef struct extend_SPaT_data {
    /** 取值0-127，车辆当前发送SPaT信息的序列号，其中0-127为一个周期 */
    uint8_t msgcnt;
    /** 路口个数，取值为1~32 */
    uint8_t nr_intersection;
    /** reserved1 */
    uint16_t rsvd1;
    /** 数值用来表示当前年份，已经过去的总分钟数（UTC 时间）。分辨率为1 分钟。该数值配合
        DSecond 数值，则可表示全年已过去的总毫秒数。 取值为0~527040，the value 527040
        shall be used for invalid */
    uint32_t moy;
    /** reserved2 */
    uint32_t rsvd2;
    /** reserved3 */
    uint32_t rsvd3;
} PACKED_STRUCT extend_SPaT_data_t;

typedef struct extend_intersection_list {
    /** 全局唯一的地区ID，取值0~65535，详细参考国标6.2.2.22DF_NodeReferenceID定义 */
    uint16_t region_id;
    /** a unique mapping to the node，一个地区内部唯一的节点ID，详细参考国标6.2.2.22DF_NodeReferenceID定义 */
    uint16_t nodeid;
    /**  general status of the controller(s)，参考国标6.2.3.27 DE_IntersectionStatusObject定义 */
    uint16_t status;
    /** 相位列表个数，取值1~16 */
    uint16_t nr_phase;
    /** 数值用来表示当前年份，已经过去的总分钟数（UTC 时间）。
        分辨率为1 分钟。该数值配合DSecond 数值，则可表示全年
        已过去的总毫秒数。 取值为0~527040，the value 527040 shall be used for invalid */
    uint32_t moy;
    /** the mSec point in the current UTC minute，有效范围0~59999，60000及以上表示未知或无效值 */
    uint32_t dsecond;
} PACKED_STRUCT extend_intersection_list_t;

typedef struct extend_phase_list {
    /** 定义信号灯相位id，取值0~255，0表示无效id，通常用来和map中的lane进行匹配 */
    uint8_t phase_id;
    /** 相位状态列表个数，取值1~16 */
    uint8_t  nr_phase_state;
    /** reserved1 */
    uint16_t rsvd1;
} PACKED_STRUCT extend_phase_list_t;

typedef struct extend_phase_state_list {
    /** phase state list与国标定义的相同，详细参考国标文档6.2.2.50和6.2.3.40，所
        有TIME分辨率为0.1 秒。有效范围是0~35999。数值36000 表示大于1 小时的时间长
        度。数值36001 表示无效数值。 */
    uint8_t  light_state;
    /**  */
    uint8_t timeFormat;
    /** reserved2 */
    uint16_t rsvd1;
    uint16_t startTime;/* OPTIONAL */
    uint16_t minEndTime;
    uint16_t maxEndTime;/* OPTIONAL */
    uint16_t likelyTime;/* OPTIONAL */
    uint16_t confidence;/* OPTIONAL */
    uint16_t nextTime;/* OPTIONAL */
    uint16_t nextDuration;/* OPTIONAL, for GB, Relative time, Unit 0.1s*/
    uint16_t rsvd2;
} PACKED_STRUCT extend_phase_state_list_t;

typedef struct extend_MAP_data {
    /** 取值0-127，车辆当前发送MAP信息的序列号，其中0-127为一个周期 */
    uint8_t msgcnt;
    /** 地图节点列表个数，取值为1~32 */
    uint8_t nr_node;
    /** reserved1 */
    uint16_t rsvd1;
    /** 数值用来表示当前年份，已经过去的总分钟数（UTC 时间）。分辨率为1 分钟。该数值配合
        DSecond 数值，则可表示全年已过去的总毫秒数。 取值为0~527040，the value 527040
        shall be used for invalid */
    uint32_t moy;
} PACKED_STRUCT extend_MAP_data_t;

typedef struct extend_node_list {
    /** 全局唯一的地区id，取值  0~65535，详细参考国标6.2.2.22DF_NodeReferenceID定义*/
    uint16_t region_id;
    /** a unique mapping to the node，一个地区内部唯一的节点id，详细参考国标6.2.2.22DF_NodeReferenceID定义 */
    uint16_t node_id;
    /* 中心点的纬度，取值-900000000~900000000，0表示未知，正数代表北纬，负数代表南纬。*/
    int32_t lat;
    /* 中心点的纬度，取值-1800000000~1800000000，0表示未知，正数代表东经，负数代表西经。*/
    int32_t lon;
    /** 中心点的海拔，取值-4096~61439，未知海拔固定填-4096?*/
    int32_t elev;
    /** 路段的个数 */
    uint8_t nr_link;
    /** reserved1 */
    uint8_t rsvd1;
    /** reserved2 */
    uint16_t rsvd2;
} PACKED_STRUCT extend_node_list_t;

typedef struct extend_link_list {
    /** 全局唯一的地区id，来自上游link，取值 0~65535，详细参考国标6.2.2.22DF_NodeReferenceID定义*/
    uint16_t up_region_id;
    /** a unique mapping to the node，一个地区内部唯一的节点id，来自上游link详细参考国标6.2.2.22DF_NodeReferenceID定义 */
    uint16_t up_node_id;
    /** 速度限制列表的个数，取值为取值为1~9 */
    uint8_t  nr_speed_limit;
    /** 车道宽度，取值为0~32767 */
    uint16_t lane_width;
    /* 路段中点的个数，取值为2~31 */
    uint8_t  nr_point;
    /* 连接关系列表的个数，取值为1~32 */
    uint8_t  nr_movement;
    /* 车道的个数,取值为1~32 */
    uint8_t  nr_lane;
    /** reserved1 */
    uint16_t rsvd1;
} PACKED_STRUCT extend_link_list_t;

typedef struct extend_speed_limit_list {
    /** 定义限速类型，参考国标6.2.3.61*/
    uint8_t type;
    /** 限速值，取值为0~8191，数值8191表示无效数值 */
    uint16_t speed_value;
    /** reserved1 */
    uint8_t rsvd1;
} PACKED_STRUCT extend_speed_limit_list_t;

typedef struct extend_point_list {
    /** 纬度，取值-900000000~900000000，0表
        示未知，正数代表北纬，负数代
        表南纬?*/
    int32_t lat;
    /** 纬度，取值-1800000000~1800000000，0
        表示未知，正数代表东经，负数
        代表西经。 */
    int32_t lon;
    /** 海拔，取值-4096~61439，未知
        海拔固定填-4096。*/
    int32_t elev;
} PACKED_STRUCT extend_point_list_t;

typedef struct extend_movement_list {
    /** 全局唯一的地区id，来自下游link，取值 0~65535，详细参考国标6.2.2.22DF_NodeReferenceID定义*/
    uint16_t down_region_id;
    /** a unique mapping to the node，一个地区内部唯一的节点id，来自下游link详细参考国标6.2.2.22DF_NodeReferenceID定义 */
    uint16_t down_node_id;
    /** 下游信号灯相位id，0代表无效值 */
    uint8_t  down_phaseId;
    /** reserved1 */
    uint8_t rsvd1;
    /** reserved2 */
    uint16_t rsvd2;
} PACKED_STRUCT extend_movement_list_t;

typedef struct extend_lane_list {
    /** 车道ID, 取值0~255，数值0代表无效ID */
    uint8_t  laneID;
    /** 定义车道被共享的情况，参考国标6.2.3.37定义 */
    uint16_t lane_shareing;
    /** 定义不同类别车道的属性集合，取值为0~8，参考国标6.2.2.14定义 */
    uint8_t  lane_type;
    /** 根据lanetype不同，属性集合不同，代表各个属性的取值，参考国标6.2.3.28-6.2.3.35的定义 */
    uint16_t lane_type_value;
    /** 定义一个（机动车）车道的允许转向行为，参考国标6.2.3.4 */
    uint16_t maneuvers;
    /** 每条车道连接关系列表的个数，取值为1~8 */
    uint8_t  nr_connectsto;
    /** 速度限制列表的个数，取值为1~9 */
    uint8_t  nr_speed_limit;
    /** 车道中点的个数，取值为2~31 */
    uint8_t  nr_point;
    /** reserved1 */
    uint8_t  rsvd1;
} PACKED_STRUCT extend_lane_list_t;

typedef struct extend_connectsto_list {
    /** 全局唯一的地区id，来自下游link，取值 0~65535，详细参考国标6.2.2.22DF_NodeReferenceID定义*/
    uint16_t down_region_id;
    /** a unique mapping to the node，一个地区内部唯一的节点id，来自下游link详细参考国标6.2.2.22DF_NodeReferenceID定义 */
    uint16_t down_node_id;
    /** 下游信号灯相位id，0代表无效值 */
    uint8_t  down_phaseId;
    /** reserved1 */
    uint8_t down_laneID;
    /** reserved2 */
    uint16_t maneuver;
} extend_connectsto_list_t;

typedef struct extend_PSM_data {
    /** 取值0-127，车辆当前发送SPaT信息的序列号，其中0-127为一个周期 */
    uint8_t msgcnt;
    /** 交通参与者个数，取值为1~16 */
    uint8_t nr_participant;
    /** reserved1 */
    uint16_t rsvd1;
    /* RSU的id*/
    uint64_t id;
    /*RSM消息的参考坐标*/
    int32_t latitude;
    int32_t longitude;
    int32_t elevation;
} PACKED_STRUCT extend_PSM_data_t;

typedef struct extend_participant_list {
    /* 交通参与者类型*/
    uint8_t    ptcType;
    /* 由RSU设置的临时ID，0表示RSU*/
    uint16_t   ptcId;
    /* 交通参与者数据的加测其来源*/
    uint8_t    source;
    /* RSM中的ID*/
    uint64_t   ID;
    /* RSM中的时间戳*/
    uint16_t   secMark;
    /** reserved1 */
    uint16_t   rsvd1;
    /** 交通参与者的坐标 */
    int32_t    latitude;
    int32_t    longitude;
    int32_t    elevation;
    /** position confidence */
    uint8_t    posCfd: 4;
    /* elevation confidence*/
    uint8_t    eleCfd: 4;
    /* 车辆档位状态*/
    uint8_t    transmission;
    /* 速度*/
    uint16_t   speed;
    /* 朝向*/
    uint16_t   heading;
    /* 方向盘角度*/
    uint8_t    angle;
    /* speed confidence*/
    uint8_t    speedCfd: 3;
    /* heading confidence*/
    uint8_t    headingCfd: 3;
    /* 方向盘转角confidence */
    uint8_t    steerCfd: 2;
    /* 四轴加速度中的纵向加速度*/
    uint16_t   longAccel;
    /* 四轴加速度中的横向加速度*/
    uint16_t   latAccel;
    /* 四轴加速度中的垂直加速度*/
    uint8_t    vertAccel;
    /* 四轴加速度中的横摆角加速度*/
    uint16_t   yawRate;
    /* 车辆高度*/
    uint8_t    height;
    /* 车辆宽度*/
    uint16_t   width;
    /* 车辆长度*/
    uint16_t   length;
    /* 类型*/
    uint8_t    classfication;
    /* 燃料类型*/
    uint8_t    fuelType;
    /* reserved2*/
    uint16_t   rsvd2;
}PACKED_STRUCT extend_participant_list_t;

typedef struct extend_RSA_data {
    /** 取值0-127，车辆当前发送RSA信息的序列号，其中0-127为一个周期 */
    uint8_t msgcnt;
    /** 道路交通事件个数，取值为1~8 */
    uint8_t nr_rtes;
    /*道路交通标志个数，取值1~16*/
    uint8_t nr_rtss;
    /** reserved1 */
    uint8_t rsvd1;
    uint32_t moy;
    /* RSI的id*/
    uint64_t id;
    /*RSA消息的参考坐标*/
    int32_t latitude;
    int32_t longitude;
    int32_t elevation;
} PACKED_STRUCT extend_RSA_data_t;

typedef struct extend_rte_list {
    uint8_t rteId;
    uint8_t eventSource;
    uint8_t nr_referencePath;
    uint8_t nr_referenceLink;
    uint32_t eventType;
    int32_t latitude;
    int32_t longitude;
    int32_t elevation;
    uint32_t eventRadius;
    uint8_t descriptionType;
    uint8_t descriptionLength;
    uint16_t rsvd1;
    uint32_t startTime;
    uint32_t endTime;
    uint8_t endTimeConfidence;
    uint8_t priority;
    uint16_t eventConfidence;
    
} PACKED_STRUCT extend_rte_list_t;

typedef struct extend_referencePath_list {
    uint16_t pathRadius;
    uint8_t nr_pathPoint;
    uint8_t rsvd1;
} PACKED_STRUCT extend_referencePath_list_t;

typedef struct extend_pathPoint_list {
    int32_t lat;
    int32_t lon;
    int32_t elev;
} PACKED_STRUCT extend_pathPoint_list_t;

typedef struct extend_referenceLink_list {
    uint16_t upRegionID;
    uint16_t upNodeID;
    uint16_t downRegionID;
    uint16_t downNodeID;
    uint16_t referenceLanes;
} PACKED_STRUCT extend_referenceLink_list_t;

typedef struct extend_rts_list {
    uint8_t rtsId;
    uint8_t signType;
    uint8_t nr_referPath;
    uint8_t nr_referLink;
    int32_t latitude;
    int32_t longitude;
    int32_t elevation;
    uint32_t startTime;
    uint32_t endTime;
    uint8_t endTimeConfidence;
    uint8_t priority;
    uint8_t descriptionType;
    uint8_t descriptionLength;
} PACKED_STRUCT extend_rts_list_t;

typedef struct in_reload_rsu_config {

    uint8_t msg_type;
    uint8_t reseverd;
    uint16_t reseverd1;
    uint32_t reseverd2;
    uint32_t reseverd3;
    char file[128];/*数字先这样使用，后面在决定怎么定义*/

}PACKED_STRUCT in_reload_rsu_config_t;

#define RSA_DESCRIPTION_SIZE_256    256

typedef struct extend_rsa 
{
    uint32_t   id;
    uint16_t   type;
    uint8_t    value;
    uint8_t    priority;
    int32_t    lat;
    int32_t    lng;
    char       des[RSA_DESCRIPTION_SIZE_256];
    uint8_t    nrpoint;
    uint8_t    reserved;
    uint16_t   alertradius;
    tpathposition wp[];
} PACKED_STRUCT extend_rsa_t;

typedef struct raw_data
{
    char     sendTime[70];
    uint32_t msgCnt;
    char     data[100];
    uint32_t sendTimeMs;
} PACKED_STRUCT raw_data_t;

typedef struct Transparent_Transmission_Data_Header
{
    uint8_t ttdID;
    uint8_t reseverd;
    uint16_t reseverd1;
    uint32_t dataSN;
    uint16_t informationage;
    uint16_t secmark;
    uint16_t discardage;
    uint16_t maxIOsize;
    uint32_t reserved2;
    uint32_t reserved3;
} PACKED_STRUCT Transparent_Transmission_Data_Header_t;

typedef struct proprietary_vehicle_info {
    /**/
    Transparent_Transmission_Data_Header_t info_header;
    /*车辆ID*/
    int32_t    vehicle_id;
    /*车辆长度*/
    float      vehicle_length;
    /*车辆宽度*/
    float      vehicle_width;
    /*车辆高度*/
    float      vehicle_height;
    /*期望驾驶模式 0：人工 1：单车 2：车队*/
    int8_t     desire_drive_mode;
    /*实际驾驶模式*/
    int8_t     actual_drive_mode;
    int8_t     cut_in_flag;
    /*精度*/
    double     longitude;
    /*纬度*/
    double     latitude;
    /*高度*/
    float      altitude;
    /*航向角*/
    float      heading;
    /*GPS状态*/
    int8_t     gps_status;
    /*GPS时间戳*/
    double    gps_time;
    /*车辆坐标系下的x值*/
    double     relative_x;
    /*车辆坐标系下的y值*/
    double     relative_y;
    /*车辆坐标系下的相对航向角*/
    double     relative_heading;
    /*纵向加速度*/
    float      longtitude_acc;
    /*侧向加速度*/
    float      lateral_acc;
    /*速度*/
    float      speed;
    /*方向盘转角*/
    float      steering_wheel_angle;
    /*横摆角速度*/
    float      yaw_rate;
    /*理想纵向加速度*/
    float      desire_long_acc;
} PACKED_STRUCT proprietary_vehicle_info_t;

typedef struct proprietary_vehicle_info_node {

    int32_t stay_time;
    proprietary_vehicle_info_t node;
    struct proprietary_vehicle_info_node* pre;
    struct proprietary_vehicle_info_node* next;

} proprietary_vehicle_info_node_t;

typedef struct proprietary_vehicle_info_list {

    int32_t node_count;
    proprietary_vehicle_info_node_t head;

} proprietary_vehicle_info_list_t;

#endif

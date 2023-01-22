#ifndef __GSW_MB_DATA_H
#define __GSW_MB_DATA_H

/*
#define U8        unsigned char
#define U16       unsigned short
#define U32       unsigned int
#define U64       unsigned long long
*/

#define APN_STRING_MAX  (64+6)
#define MB_INVALID_INTERFACE (0x09)
#define MB_DATA_MSG_LEN  (0X01)

#define IPV4_ADDRSTRLEN 16
#define IPV6_ADDRSTRLEN 46

#define DATA0      "/sys/gswgobi/data0"
#define DATA0_CFG   "/sys/gswgobi/data0cfg"

#define DATA1      "/sys/gswgobi/data1"
#define DATA1_CFG   "/sys/gswgobi/data1cfg"

#define DATA2      "/sys/gswgobi/data2"
#define DATA2_CFG   "/sys/gswgobi/data2cfg"

#define CALL_STATUS_IND  "/sys/gswgobi/gobi_call_status"


/*typedef struct ipv6_addr {
    unsigned char         ipv6addr[16];
    unsigned char         prefix;
} ipv6_addr;*/

typedef enum
{
    MB_FIRST_INTERFACE = 0x00,
    MB_SEC_INTERFACE   = 0x01,
    MB_THIRD_INTERFACE = 0X02,
    MB_MAX_INTERFACE   = MB_INVALID_INTERFACE
} mb_interface_id_e_type;

typedef enum
{
    MB_IPV4     = 0x00,
    MB_IPV6     = 0x01,
    MB_IPV4V6   = 0x02
} mb_pdp_type_e_type;

typedef enum
{
    MB_DATACONN_REQ_MSG    = 0x00, //0x0000,
    MB_DATACONN_RSP_MSG    = 0x01, //0x0001,
    MB_DATACONN_ERR_MSG    = 0x02, //0x0002,
    MB_DATACONN_WAIT_MSG    = 0x03, //0x0003,

    MB_DATADISCONN_REQ_MSG = 0x10, //0x0100,
    MB_DATADISCONN_RSP_MSG = 0x11, //0x0101,
    MB_DATADISCONN_WAIT_MSG = 0x11, //0x0101,

    MB_DATA_ERR_REQ_MSG    = 0x20, //0x0200,
    MB_DATA_ERR_RSP_MSG    = 0x21, // 0x0201,

    MB_DATALIST_REQ_MSG    = 0x30, //0x0300,
    MB_DATALIST_RSP_MSG    = 0x31, //0x0301,

    MB_DATAIND_REQ_MSG     = 0x40, //0x0400,
    MB_DATAIND_RSP_MSG     = 0x41, //0x0401,

    MB_MAX_REQ_MSG         = 0x90, //0x0900,
    MB_MAX_RSP_MSG         = 0x91  //0x0901
} mb_data_msg_e_type;

typedef enum
{
    MB_IPV4_DISCONNECTED    = 0x00,
    MB_IPV4_CONNECTED       = 0x01,
    MB_IPV4_DISCONNECTING   = 0x02,
    MB_IPV4_CONNECTING      = 0x03,

    MB_IPV6_DISCONNECTED    = MB_IPV4_DISCONNECTED, //= 0x10,
    MB_IPV6_CONNECTED       ,//= 0x11,
    MB_IPV6_DISCONNECTING   ,//= 0x12,
    MB_IPV6_CONNECTING      ,//= 0x13,

    MB_DORMANCY             = 0X20,

    MB_MAX_STATUS         = 0x99
} mb_pdp_state_e_type;


//request data conn
typedef struct
{
    mb_interface_id_e_type  if_id;
    int  cid;
    int  pdp_type;
    char apn[APN_STRING_MAX];
} mb_data_conn_req_s_type;   //其中int pdp_type, char apn[64] 为空。


typedef struct
{

    char   public_ip[IPV4_ADDRSTRLEN];
    char   gateway_ip[IPV4_ADDRSTRLEN];
    char   primary_dns[IPV4_ADDRSTRLEN];
    char   secondary_dns[IPV4_ADDRSTRLEN];
} v4_conf_s_type;


typedef struct
{
    char  public_ipv6[IPV6_ADDRSTRLEN];
    char  gateway_ipv6[IPV6_ADDRSTRLEN];
    char  primary_dnsv6[IPV6_ADDRSTRLEN];
    char  secondary_dnsv6[IPV6_ADDRSTRLEN];
} v6_conf_s_type;

//result
typedef struct
{
    int end_type;
    int call_end_reason;
} mb_data_conn_ret_s_type;

//rsp ipv4v6
typedef struct
{
    mb_interface_id_e_type         if_id;
    int         cid;
    int         pdp_type;

    int         v4_pdp_status;
    v4_conf_s_type   v4_addr;
    int v4_mtu;

    int         v6_pdp_status;
    v6_conf_s_type   v6_addr;
    int v6_mtu;

} mb_data_conn_rsp_s_type;

typedef struct
{
    int qmi_client_v4;
    int data_handle_v4;
    mb_data_conn_ret_s_type   v4_conn_err_code;

    int qmi_client_v6;
    int data_handle_v6;
    mb_data_conn_ret_s_type   v6_conn_err_code;

    int gobi_current_command;

    char apn[APN_STRING_MAX];

    mb_data_conn_rsp_s_type data_rsp;
} mb_data_conn_state_s_type;



// 1 拨号
//request msg - dataconn
typedef struct
{
    int msg;       // MB_DATACONN_REQ_MSG;
    mb_data_conn_req_s_type data_conn_req;
} mb_data_conn_req_msg_s_type;

//rsp msg1 - dataconn
typedef struct
{
    int msg;       // MB_DATACONN_RSP_MSG;
    mb_data_conn_rsp_s_type   data_conn_rsp;
} mb_data_conn_rsp_msg_s_type;

//rsp msg2 - dataconn
typedef struct
{
    int msg;       // MB_DATACONN_ERR_MSG;
    mb_data_conn_ret_s_type   v4_con_err_code;
    mb_data_conn_ret_s_type   v6_conn_err_code;
} mb_data_conn_err_rsp_msg_s_type;

//rsp msg3 - dataconn
typedef struct
{
    int msg;       // MB_DATACONN_WAIT_MSG;
} mb_data_conn_wait_rsp_msg_s_type;


// 2. 断开拨号
//request msg - data disconn
typedef struct
{
    int msg;       // MB_DATADISCONN_REQ_MSG;
    mb_interface_id_e_type  if_id;
} mb_data_disconn_req_msg_s_type;

//response msg - data disconn
typedef struct
{
    int msg;       //MB_DATADISCONN_RSP_MSG;
    int ret;
} mb_data_disconn_rsp_msg_s_type;

//response msg - data disconn
typedef struct
{
    int msg;       //MB_DATADISCONN_WAIT_MSG;
} mb_data_disconn_wait_msg_s_type;

// 3. 查询拨号失败原因值
typedef struct
{
    int msg; 	  // MB_DATA_ERR_REQ_MSG;
    mb_interface_id_e_type	if_id;
} mb_data_err_req_msg_s_type;

typedef struct
{
    int msg;       // MB_DATA_ERR_RSP_MSG;
    mb_data_conn_ret_s_type   v4_conn_err_code;
    mb_data_conn_ret_s_type   v6_conn_err_code;
} mb_data_err_rsp_msg_s_type;


// 4. 拨号状态查询
//request msg - data list
typedef struct
{
    int msg;       // MB_DATALIST_REQ_MSG;
    mb_interface_id_e_type  if_id;
} mb_data_list_state_req_msg_s_type;

//response msg - data list
/*typedef struct{
    int msg;       //MB_DATALIST_RSP_MSG;
    mb_data_conn_rsp_s_type   data_conn_rsp;
} mb_data_conn_rsp_msg_s_type;*/


// 5 拨号状态主动上报设置

/*typedef struct{
    int msg;       // MB_DATAIND_RSP_MSG;
    mb_data_conn_rsp_s_type   data_conn_rsp;
} mb_data_conn_rsp_msg_s_type;*/


#endif // __GSW_MB_DATA_H


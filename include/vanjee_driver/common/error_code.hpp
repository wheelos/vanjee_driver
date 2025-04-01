#pragma once
#include <string>
#include <ctime>

namespace vanjee
{
namespace lidar
{
        enum class ErrCodeType
        {
            INFO_CODE,
            WARNING_CODE,
            ERROR_CODE
        };
        enum ErrCode
        {
            ERRCODE_SUCCESS = 0x00,   
            ERRCODE_PCAPREPAT = 0x01, 
            ERRCODE_PCAPEXIT = 0x02,  

            ERRCODE_MSOPTIMEOUT = 0x40,    
            ERRCODE_NODIFOPRECV = 0x41,    
            ERRCODE_WRONGMSOPLEN = 0x42,   
            ERRCODE_WRONGMSOPBLKID = 0x44, 
            ERRCODE_WRONGDIFOPLEN = 0x45,  
            ERRCODE_WRONGDIFOPID = 0x46,   
            ERRCODE_ZEROPOINTS = 0x47,     
            ERRCODE_PKTBUFOVERFLOW = 0x48, 
            ERRCODE_CLOUDOVERFLOW = 0x49,
            ERRCODE_WRONGIMUPACKET = 0x50,  

            ERRCODE_STARTBEFOREINIT = 0x80, 
            ERRCODE_PCAPWRONGPATH = 0x81,   
            ERRCODE_POINTCLOUDNULL = 0x82,
            ERRCODE_IMUPACKETNULL = 0x83,
            ERRCODE_LASERSCANPACKETNULL = 0x84      

        };

        struct Error
        {
            ErrCode error_code;
            ErrCodeType error_code_type;

            Error()
                : error_code(ErrCode::ERRCODE_SUCCESS)
            {
            }
            explicit Error(const ErrCode &code)
                : error_code(code)
            {
                if (error_code < 0x40)
                {
                    error_code_type = ErrCodeType::INFO_CODE;
                }
                else if (error_code < 0x80)
                {
                    error_code_type = ErrCodeType::WARNING_CODE;
                }
                else
                {
                    error_code_type = ErrCodeType::ERROR_CODE;
                }
            }
            std::string toString() const
            {
                switch (error_code)
                {
                case ERRCODE_PCAPREPAT:
                    return "Info_PcapRepeat";
                case ERRCODE_PCAPEXIT:
                    return "Info_PcapExit";

                case ERRCODE_MSOPTIMEOUT:
                    return "ERRCODE_MSOPTIMEOUT";
                case ERRCODE_NODIFOPRECV:
                    return "ERRCODE_NODIFOPRECV";
                case ERRCODE_WRONGMSOPLEN:
                    return "ERRCODE_WRONGMSOPLEN";
                case ERRCODE_WRONGMSOPBLKID:
                    return "ERRCODE_WRONGMSOPBLKID";
                case ERRCODE_WRONGDIFOPID:
                    return "ERRCODE_WRONGDIFOPID";
                case ERRCODE_WRONGDIFOPLEN:
                    return "ERRCODE_WRONGDIFOPLEN";
                case ERRCODE_ZEROPOINTS:
                    return "ERRCODE_ZEROPOINTS";
                case ERRCODE_PKTBUFOVERFLOW:
                    return "ERRCODE_PKTBUFOVERFLOW";
                case ERRCODE_CLOUDOVERFLOW:
                    return "ERRCODE_CLOUDOVERFLOW";
                case ERRCODE_WRONGIMUPACKET:
                    return "ERRCODE_WRONGIMUPACKET";

                case ERRCODE_STARTBEFOREINIT:
                    return "ERRCODE_STARTBEFORINIT";
                case ERRCODE_PCAPWRONGPATH:
                    return "ERRCODE_PCAPWRONGPATH";
                case ERRCODE_POINTCLOUDNULL:
                    return "ERRCODE_POINTCLOUDNULL";
                case  ERRCODE_IMUPACKETNULL:
                    return "ERRCODE_IMUPACKETNULL";
                case ERRCODE_LASERSCANPACKETNULL:
                    return "ERRCODE_LASERSCANPACKETNULL";

                default:
                    return "ERRCODE_SUCCESS";
                }
            }
        };
#define LIMIT_CALL(func, sec)         \
    {                                 \
        static time_t prev_tm = 0;    \
        time_t cur_tm = time(NULL);   \
        if ((cur_tm - prev_tm) > sec) \
        {                             \
            func;                     \
            prev_tm = cur_tm;         \
        }			       \
    }
#define DELAY_LIMIT_CALL(func, sec)         \
    {                                       \
        static time_t prev_tm = time(NULL); \
        time_t curt_tm = time(NULL);        \
        if ((curt_tm - prev_tm) > sec)      \
        {                                   \
            func;                           \
            prev_tm = curt_tm;              \
        }                                   \
    }
}  
}  

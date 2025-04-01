#pragma once

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/decoder/wlr720_32/protocol/frames/cmd_repository_720_32.hpp>
#include <vanjee_driver/driver/decoder/wlr720_32/protocol/frames/protocol_ldangle_get.hpp>

namespace vanjee
{
    namespace lidar
    {
        #pragma pack(push, 1)
        typedef struct _Vanjee720_32Channel
        {
            uint16_t distance;      ///< 距离 缩小4倍
            uint8_t intensity;      ///< 强度值
            uint8_t reflectivity;   ///< 反射
        } Vanjee720_32Channel;

        typedef struct _Vanjee720_32Block
        {
            uint16_t azimuth;                   ///< 方位角
            Vanjee720_32Channel channel[32];    ///< 通道数据
        } Vanjee720_32Block;

        typedef struct _Vanjee720_32Difop
        {
            uint8_t mac_id[2];
            uint16_t circle_id;
            uint8_t data_time[6];
            uint8_t timestamp[4];
            int16_t line_speed_x;
            int16_t line_speed_y;
            int16_t line_speed_z;
            int16_t add_speed_x;
            int16_t add_speed_y;
            int16_t add_speed_z;
            uint8_t info[34];
        } Vanjee720_32Difop;

        typedef struct _Vanjee720_32MsopPkt
        {
            uint8_t head[2];
            uint8_t channel_num;
            uint8_t return_wave_num;
            uint8_t block_num;
            Vanjee720_32Block blocks[10]; ///< 数据块
            Vanjee720_32Difop difop;
        } Vanjee720_32MsopPkt;

        #pragma pack(pop)
        template <typename T_PointCloud>
        class DecoderVanjee720_32 : public DecoderMech<T_PointCloud>
        {
        private:
            std::vector<std::vector<double>> all_points_luminous_moment_720_32_;    // 缓存32通道一圈点云时间差
            const double luminous_period_of_ld_ = 0.00005555;                       // 相邻水平角度下时间间隔
            const double luminous_period_of_adjacent_ld_ = 0.00000160;              // 组内相邻垂直角度下时间间隔
            
            int32_t azimuth_trans_pre_ = -1.0;

            int32_t pre_frame_id_ = -1;
            int32_t pre_circle_id_ = -1;
            uint8_t publish_mode_ = 0;

            int32_t pre_pkt_hor_angle_ = -1;
            double pre_pkt_time_ = -1;
            int32_t resolution_num_offset_ = -1;

            std::shared_ptr<SplitStrategy> split_strategy_; 
            static WJDecoderMechConstParam &getConstParam(uint8_t mode);
            void initLdLuminousMoment(void);
            bool lidarParamGet(uint32_t hor_angle,uint32_t resolution, uint8_t block_num, double time);
        public:
            constexpr static double FRAME_DURATION = 0.1;
            constexpr static uint32_t SINGLE_PKT_NUM = 100;

            virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
            virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
            virtual ~DecoderVanjee720_32() = default;
            explicit DecoderVanjee720_32(const WJDecoderParam &param);
        };
    
        template<typename T_PointCloud>
        bool DecoderVanjee720_32<T_PointCloud>::lidarParamGet(uint32_t hor_angle,uint32_t resolution, uint8_t block_num, double time)
        {
            if(pre_pkt_hor_angle_ > 0 && (hor_angle + 36000 - pre_pkt_hor_angle_) % 36000 == resolution * block_num)
            {
                if(pre_pkt_time_ > 0 && fabs(time - pre_pkt_time_) > 0.001)
                {
                    resolution_num_offset_ = (hor_angle / resolution) % 180;
                    return true;
                }
            }
            pre_pkt_hor_angle_ = hor_angle;
            pre_pkt_time_ = time;
            return false;
        }
        
        template<typename T_PointCloud>
        void DecoderVanjee720_32<T_PointCloud>::initLdLuminousMoment()
        {
            double offset = 0;
            all_points_luminous_moment_720_32_.resize(4);
            all_points_luminous_moment_720_32_[0].resize(115200);
            all_points_luminous_moment_720_32_[1].resize(57600);
            all_points_luminous_moment_720_32_[2].resize(38400);
            all_points_luminous_moment_720_32_[3].resize(28800);
            for(uint16_t col = 0;col < 3600;col++)
            {
                for(uint8_t row = 0;row < 32;row++)
                {
                    if(row < 8)
                        offset = row*luminous_period_of_adjacent_ld_;
                    else if(row < 16)
                        offset = (row-8)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ / 4;
                    else if (row < 24)
                        offset =(row - 16)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 2 / 4;
                    else
                        offset = (row-24)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * 3 / 4;
                    
                    if(col < 900)
                    {
                        for(int i = 0; i < 4; i++)
                            all_points_luminous_moment_720_32_[i][col*32+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else if(col >= 900 && col < 1200)
                    {
                        for(int i = 0; i < 3; i++)
                            all_points_luminous_moment_720_32_[i][col*32+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else if(col >= 1200 && col < 1800)
                    {
                        for(int i = 0; i < 2; i++)
                            all_points_luminous_moment_720_32_[i][col*32+row] = col*luminous_period_of_ld_ + offset;
                    }
                    else
                    {
                        all_points_luminous_moment_720_32_[0][col*32+row] = col*luminous_period_of_ld_ + offset;
                    }
                }
            }
        }

        template <typename T_PointCloud>
        inline WJDecoderMechConstParam &DecoderVanjee720_32<T_PointCloud>::getConstParam(uint8_t mode)
        {
            //WJ_INFOL << "publish_mode ============mode=================" << mode << WJ_REND;
            uint16_t msop_len = 1365;
            uint16_t laser_num = 32;
            uint16_t block_num = 10;
            uint16_t chan_num = 32;
            float distance_min = 0.3f;
            float distance_max = 120.0f;
            float distance_resolution = 0.004f;
            float init_temperature = 80.0f;

            static WJDecoderMechConstParam param =
                {
                    msop_len /// msop len
                    ,
                    laser_num /// laser number
                    ,
                    block_num /// blocks per packet
                    ,
                    chan_num /// channels per block
                    ,
                    distance_min /// distance min
                    ,
                    distance_max /// distance max
                    ,
                    distance_resolution /// distance resolution
                    ,
                    init_temperature /// initial value of temperature
                };
            param.BLOCK_DURATION = 0.1 / 360;
            return param;
        }

        template <typename T_PointCloud>
        inline DecoderVanjee720_32<T_PointCloud>::DecoderVanjee720_32(const WJDecoderParam &param)
            : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param)
        {
            if(param.max_distance < param.min_distance)
                WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

            publish_mode_ = param.publish_mode;
            this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
            split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);

            if(this->param_.config_from_file)
            {
              int ret_angle = this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
            }
            initLdLuminousMoment();
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720_32<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
        {
            if(size !=  sizeof(Vanjee720_32MsopPkt))
                return false;
            static int32_t pre_pkt_resolution = -1;
            auto &packet = *(Vanjee720_32MsopPkt *)pkt;
            bool ret = false;
            double pkt_ts = 0;
            double pkt_host_ts = 0;
            double pkt_lidar_ts = 0;

            uint16_t frame_id = (packet.difop.info[30] << 8) | packet.difop.info[31];
            uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
            if(loss_packets_num > 1 && pre_frame_id_ >= 0)
                WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
            pre_frame_id_ = frame_id;

            pkt_host_ts = getTimeHost() * 1e-6;
            std::tm stm;
            memset(&stm, 0, sizeof(stm));
            stm.tm_year = packet.difop.data_time[5] + 100;
            stm.tm_mon = packet.difop.data_time[4] - 1;
            stm.tm_mday = packet.difop.data_time[3];
            stm.tm_hour = packet.difop.data_time[2];
            stm.tm_min = packet.difop.data_time[1];
            stm.tm_sec = packet.difop.data_time[0];
            double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) + ((packet.difop.timestamp[3] & 0x0F) << 24)) * 1e-8;
            pkt_lidar_ts = std::mktime(&stm) + nsec;

            if(!this->param_.use_lidar_clock)
                pkt_ts = pkt_host_ts;
            else
                pkt_ts = pkt_lidar_ts;

            int32_t resolution = 10;
            uint8_t resolution_index = 0;
            double last_point_to_first_point_time = 0;
            if(packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0)
            {
                resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }
            else
            {
                resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000)%36000;
            }

            if(resolution != pre_pkt_resolution)
            {
                resolution_num_offset_ = -1;
                pre_pkt_hor_angle_ = -1;
                pre_pkt_time_ = -1;
            }
            pre_pkt_resolution = resolution;
            if(resolution_num_offset_ < 0 && !lidarParamGet(packet.blocks[0].azimuth, resolution, packet.block_num, pkt_lidar_ts))
            {
                return false;
            }

            if(resolution == 10)
            {
                resolution_index = 0;
                last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_720_32_[resolution_index][115199];
            }
            else if(resolution == 20)
            {
                resolution_index = 1;
                last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_720_32_[resolution_index][57599];
            }
            else if(resolution == 30)
            {
                resolution_index = 2;
                last_point_to_first_point_time = (1.0 / 15.0) - all_points_luminous_moment_720_32_[resolution_index][38399];
            }
            else if(resolution == 40)
            {
                resolution_index = 3;
                last_point_to_first_point_time = (1.0 / 20.0) - all_points_luminous_moment_720_32_[resolution_index][28799];
            }
            else
            {
                return ret;
            }

            uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
            if(loss_circles_num > 1)
            {
                this->point_cloud_->points.clear();
            }
            uint16_t block_num = packet.block_num * packet.return_wave_num;
            for (uint16_t blk = 0; blk < block_num; blk++)
            {
                if(packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1))
                {
                    continue;
                }
                else if(packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0))
                {
                    continue;
                }
                
                const Vanjee720_32Block &block = packet.blocks[blk];
                int32_t azimuth = block.azimuth % 36000;
                int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

                if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
                    (loss_circles_num == 1 && blk == 0 && 
                    (packet.blocks[0].azimuth % 36000) != 0 && packet.blocks[0].azimuth < packet.blocks[block_num-1].azimuth && this->point_cloud_->points.size() != 0))
                {
                    int32_t time_reload_num = 0;
                    int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
                    if(time_change_num >= 0)
                    {
                        time_reload_num = time_change_num / 180;
                        if(time_change_num % 180 >= 0)
                        {
                            time_reload_num += 1;
                        }
                    }
                    uint32_t point_gap_num = (time_reload_num * 180 + resolution_num_offset_) * 32;
                    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720_32_[resolution_index][point_gap_num] - last_point_to_first_point_time;
                    this->first_point_ts_ =  this->last_point_ts_ - all_points_luminous_moment_720_32_[resolution_index][all_points_luminous_moment_720_32_[resolution_index].size()-1];

                    this->cb_split_frame_(32, this->cloudTs());
                    ret = true;
                }
                
                {
                    double timestamp_point;
                    for (uint16_t chan = 0; chan < packet.channel_num; chan++)
                    {
                        float x, y, z, xy;

                        uint32_t point_id = azimuth / resolution * packet.channel_num + chan;
                        if(this->param_.ts_first_point == true)
                        {
                          timestamp_point = all_points_luminous_moment_720_32_[resolution_index][point_id];
                        }
                        else
                        {
                          timestamp_point = all_points_luminous_moment_720_32_[resolution_index][point_id] - all_points_luminous_moment_720_32_[resolution_index][all_points_luminous_moment_720_32_[resolution_index].size()-1];
                        }

                        const Vanjee720_32Channel &channel = block.channel[chan];
                        
                        float distance = channel.distance * this->const_param_.DISTANCE_RES;
                        int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                        int32_t angle_horiz = this->chan_angles_.horizAdjust(chan, azimuth * 10) % 360000;

                        if (angle_horiz < 0)
                        {
                            angle_horiz += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz < this->param_.start_angle * 1000 || angle_horiz > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz > this->param_.end_angle * 1000 && angle_horiz < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }

                        int32_t azimuth_index = angle_horiz;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.reflectivity);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan + this->first_line_id_);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                float x_zero = 0;
                                float y_zero = 0;
                                float z_zero = 0;
                                this->transformPoint(x_zero, y_zero, z_zero);

                                setX(point, x_zero);
                                setY(point, y_zero);
                                setZ(point, z_zero);
                            }
                            setIntensity(point, 0);
                            setTimestamp(point, timestamp_point);
                            setRing(point, chan + this->first_line_id_);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                }
                if (azimuth_trans == 0 && 
                    (packet.return_wave_num == 1 || (packet.return_wave_num == 2 && publish_mode_ != 2) || 
                        (packet.return_wave_num == 2 && publish_mode_ == 2 && azimuth_trans_pre_ == 0)))
                {
                    uint32_t point_gap_num = resolution_num_offset_ * 32;
                    this->last_point_ts_ = pkt_ts - all_points_luminous_moment_720_32_[resolution_index][point_gap_num];
                    this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_720_32_[resolution_index][all_points_luminous_moment_720_32_[resolution_index].size()-1];
                    this->cb_split_frame_(32, this->cloudTs());
                    ret = true;
                }
                azimuth_trans_pre_ = azimuth_trans;
            }
            pre_circle_id_ = ntohs(packet.difop.circle_id);
            this->prev_pkt_ts_ = pkt_lidar_ts;
            return ret;
        }

        template<typename T_PointCloud>
        void DecoderVanjee720_32<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
        {
          std::shared_ptr<ProtocolAbstract720_32> p;
          std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

          if(*sp_cmd == *(CmdRepository720_32::CreateInstance()->sp_ld_angle_get_))
          {
            p = std::make_shared<Protocol_LDAngleGet720_32>();
          }
          else
          {
            return;
          }
          p->Load(*protocol);

          std::shared_ptr<ParamsAbstract> params = p->Params;
          if(typeid(*params) == typeid(Params_LDAngle720_32))
          {
            if(!this->param_.wait_for_difop)
            {
                if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
                {
                    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
                }
                return;
            }

            std::shared_ptr<Params_LDAngle720_32> param = std::dynamic_pointer_cast<Params_LDAngle720_32>(params);

            std::vector<double> vert_angles;
            std::vector<double> horiz_angles;

            for (int num_of_lines = 0; num_of_lines < param->num_of_lines_; num_of_lines++)
            {
              vert_angles.push_back((double)(param->ver_angle_[num_of_lines] / 1000.0));
              horiz_angles.push_back((double)(0));
            }

            this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver , param->num_of_lines_ , vert_angles , horiz_angles);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
              Decoder<T_PointCloud>::angles_ready_ = true;
            }
          }
          else
          {
            WJ_WARNING << "Unknown Params Type..." << WJ_REND;
          }

        }

    } // namespace lidar

} // namespace vanjee

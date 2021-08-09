#ifndef _ROS_statek_map_GraphNode_h
#define _ROS_statek_map_GraphNode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace statek_map
{

  class GraphNode : public ros::Msg
  {
    public:
      typedef uint32_t _id_type;
      _id_type id;
      typedef geometry_msgs::Point _point_type;
      _point_type point;
      typedef bool _isStart_type;
      _isStart_type isStart;
      typedef bool _isGoal_type;
      _isGoal_type isGoal;
      uint32_t neighbors_length;
      typedef uint32_t _neighbors_type;
      _neighbors_type st_neighbors;
      _neighbors_type * neighbors;

    GraphNode():
      id(0),
      point(),
      isStart(0),
      isGoal(0),
      neighbors_length(0), st_neighbors(), neighbors(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      offset += this->point.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isStart;
      u_isStart.real = this->isStart;
      *(outbuffer + offset + 0) = (u_isStart.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isStart);
      union {
        bool real;
        uint8_t base;
      } u_isGoal;
      u_isGoal.real = this->isGoal;
      *(outbuffer + offset + 0) = (u_isGoal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isGoal);
      *(outbuffer + offset + 0) = (this->neighbors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->neighbors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->neighbors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->neighbors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->neighbors_length);
      for( uint32_t i = 0; i < neighbors_length; i++){
      *(outbuffer + offset + 0) = (this->neighbors[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->neighbors[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->neighbors[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->neighbors[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->neighbors[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      offset += this->point.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isStart;
      u_isStart.base = 0;
      u_isStart.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isStart = u_isStart.real;
      offset += sizeof(this->isStart);
      union {
        bool real;
        uint8_t base;
      } u_isGoal;
      u_isGoal.base = 0;
      u_isGoal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isGoal = u_isGoal.real;
      offset += sizeof(this->isGoal);
      uint32_t neighbors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      neighbors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      neighbors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      neighbors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->neighbors_length);
      if(neighbors_lengthT > neighbors_length)
        this->neighbors = (uint32_t*)realloc(this->neighbors, neighbors_lengthT * sizeof(uint32_t));
      neighbors_length = neighbors_lengthT;
      for( uint32_t i = 0; i < neighbors_length; i++){
      this->st_neighbors =  ((uint32_t) (*(inbuffer + offset)));
      this->st_neighbors |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_neighbors |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_neighbors |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_neighbors);
        memcpy( &(this->neighbors[i]), &(this->st_neighbors), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "statek_map/GraphNode"; };
    virtual const char * getMD5() override { return "2ab23385384b66528e5cbe4edf3e9b98"; };

  };

}
#endif

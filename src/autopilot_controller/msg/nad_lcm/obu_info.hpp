/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __nad_lcm_obu_info_hpp__
#define __nad_lcm_obu_info_hpp__

#include <string>
#include "lane_point.hpp"

namespace nad_lcm
{

/// obu每秒上报自身信息的结构
class obu_info
{
    public:
        std::string obu_name;

        double     cur_lon;

        double     cur_lat;

        double     cur_yaw;

        nad_lcm::lane_point cur_point;

        double     cur_speed;

        double     cur_speed_lateral;

        double     cur_speed_longitudinal;

        double     cur_acceleration_pattern;

        double     cur_acceleration;

        double     cur_acceleration_lateral;

        double     cur_acceleration_longitudinal;

        double     steering_angle;

        int32_t    camera_stat[16];

        int32_t    radar_stat[16];

        int32_t    lidar_stat[16];

        int32_t    gps_stat[16];

        int32_t    cur_gears;

        int32_t    drive_button_stat;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "obu_info"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int obu_info::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int obu_info::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int obu_info::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t obu_info::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* obu_info::getTypeName()
{
    return "obu_info";
}

int obu_info::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    char* obu_name_cstr = (char*) this->obu_name.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &obu_name_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_lon, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_lat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_yaw, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->cur_point._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_speed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_speed_lateral, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_speed_longitudinal, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration_pattern, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration_lateral, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration_longitudinal, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->steering_angle, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->camera_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->radar_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->lidar_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->gps_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->cur_gears, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->drive_button_stat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int obu_info::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    int32_t __obu_name_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__obu_name_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__obu_name_len__ > maxlen - pos) return -1;
    this->obu_name.assign(((const char*)buf) + offset + pos, __obu_name_len__ - 1);
    pos += __obu_name_len__;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_lon, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_lat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_yaw, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->cur_point._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_speed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_speed_lateral, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_speed_longitudinal, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration_pattern, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration_lateral, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->cur_acceleration_longitudinal, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->steering_angle, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->camera_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->radar_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->lidar_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->gps_stat[0], 16);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->cur_gears, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->drive_button_stat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int obu_info::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->obu_name.size() + 4 + 1;
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += this->cur_point._getEncodedSizeNoHash();
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 16);
    enc_size += __int32_t_encoded_array_size(NULL, 16);
    enc_size += __int32_t_encoded_array_size(NULL, 16);
    enc_size += __int32_t_encoded_array_size(NULL, 16);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t obu_info::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == obu_info::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)obu_info::getHash };

    int64_t hash = 0x04a37bf4649f7515LL +
         nad_lcm::lane_point::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
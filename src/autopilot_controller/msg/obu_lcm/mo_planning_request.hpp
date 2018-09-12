/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __obu_lcm_mo_planning_request_hpp__
#define __obu_lcm_mo_planning_request_hpp__

#include <string>

namespace obu_lcm
{

/**
 * -------------------------------------------------------------------
 * 运动规划进程motion_planner为路径规划进程obu_planning发消息
 * 道路全堵死时请求重规划
 * -------------------------------------------------------------------
 */
class mo_planning_request
{
    public:
        std::string obu_name;

        double     starting_lon;

        double     starting_lat;

        double     ending_lon;

        double     ending_lat;

        std::string destination;

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
         * Returns "mo_planning_request"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int mo_planning_request::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int mo_planning_request::decode(const void *buf, int offset, int maxlen)
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

int mo_planning_request::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t mo_planning_request::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* mo_planning_request::getTypeName()
{
    return "mo_planning_request";
}

int mo_planning_request::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    char* obu_name_cstr = (char*) this->obu_name.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &obu_name_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->starting_lon, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->starting_lat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->ending_lon, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->ending_lat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* destination_cstr = (char*) this->destination.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &destination_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int mo_planning_request::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    int32_t __obu_name_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__obu_name_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__obu_name_len__ > maxlen - pos) return -1;
    this->obu_name.assign(((const char*)buf) + offset + pos, __obu_name_len__ - 1);
    pos += __obu_name_len__;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->starting_lon, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->starting_lat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->ending_lon, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->ending_lat, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    int32_t __destination_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__destination_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__destination_len__ > maxlen - pos) return -1;
    this->destination.assign(((const char*)buf) + offset + pos, __destination_len__ - 1);
    pos += __destination_len__;

    return pos;
}

int mo_planning_request::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->obu_name.size() + 4 + 1;
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += this->destination.size() + 4 + 1;
    return enc_size;
}

uint64_t mo_planning_request::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0xe0171819f4c1477eLL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif

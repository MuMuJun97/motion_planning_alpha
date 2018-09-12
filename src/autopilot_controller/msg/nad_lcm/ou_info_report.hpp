/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __nad_lcm_ou_info_report_hpp__
#define __nad_lcm_ou_info_report_hpp__

#include <vector>
#include "obu_info.hpp"
#include "light_info.hpp"
#include "limspeed_info.hpp"
#include "block_info.hpp"

namespace nad_lcm
{

/// obu->vui：每秒下发交通信息，也起到心跳的作用
class ou_info_report
{
    public:
        nad_lcm::obu_info obu;

        int16_t    num_of_light;

        std::vector< nad_lcm::light_info > light;

        int16_t    num_of_limspeed;

        std::vector< nad_lcm::limspeed_info > limspeed;

        int16_t    num_of_block;

        std::vector< nad_lcm::block_info > block;

        int16_t    num_of_lane;

        std::vector< int64_t > cur_lanelist;

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
         * Returns "ou_info_report"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int ou_info_report::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ou_info_report::decode(const void *buf, int offset, int maxlen)
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

int ou_info_report::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t ou_info_report::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* ou_info_report::getTypeName()
{
    return "ou_info_report";
}

int ou_info_report::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->obu._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_of_light, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_of_light; a0++) {
        tlen = this->light[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_of_limspeed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_of_limspeed; a0++) {
        tlen = this->limspeed[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_of_block, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_of_block; a0++) {
        tlen = this->block[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_of_lane, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_of_lane > 0) {
        tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->cur_lanelist[0], this->num_of_lane);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int ou_info_report::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->obu._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_of_light, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->light.resize(this->num_of_light);
    for (int a0 = 0; a0 < this->num_of_light; a0++) {
        tlen = this->light[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_of_limspeed, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->limspeed.resize(this->num_of_limspeed);
    for (int a0 = 0; a0 < this->num_of_limspeed; a0++) {
        tlen = this->limspeed[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_of_block, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->block.resize(this->num_of_block);
    for (int a0 = 0; a0 < this->num_of_block; a0++) {
        tlen = this->block[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_of_lane, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_of_lane) {
        this->cur_lanelist.resize(this->num_of_lane);
        tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->cur_lanelist[0], this->num_of_lane);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int ou_info_report::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->obu._getEncodedSizeNoHash();
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_of_light; a0++) {
        enc_size += this->light[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_of_limspeed; a0++) {
        enc_size += this->limspeed[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_of_block; a0++) {
        enc_size += this->block[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, this->num_of_lane);
    return enc_size;
}

int64_t ou_info_report::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == ou_info_report::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)ou_info_report::getHash };

    int64_t hash = 0x4321be60d1230440LL +
         nad_lcm::obu_info::_computeHash(&cp) +
         nad_lcm::light_info::_computeHash(&cp) +
         nad_lcm::limspeed_info::_computeHash(&cp) +
         nad_lcm::block_info::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif

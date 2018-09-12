/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __nad_lcm_cr_start_auto_respond_hpp__
#define __nad_lcm_cr_start_auto_respond_hpp__

#include <string>

namespace nad_lcm
{

/// csu->rsu：返回开始自动驾驶结果
class cr_start_auto_respond
{
    public:
        std::string obu_name;

        int32_t    retcode;

        int32_t    start_reason;

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
         * Returns "cr_start_auto_respond"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int cr_start_auto_respond::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int cr_start_auto_respond::decode(const void *buf, int offset, int maxlen)
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

int cr_start_auto_respond::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t cr_start_auto_respond::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* cr_start_auto_respond::getTypeName()
{
    return "cr_start_auto_respond";
}

int cr_start_auto_respond::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    char* obu_name_cstr = (char*) this->obu_name.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &obu_name_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->retcode, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->start_reason, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int cr_start_auto_respond::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    int32_t __obu_name_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__obu_name_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__obu_name_len__ > maxlen - pos) return -1;
    this->obu_name.assign(((const char*)buf) + offset + pos, __obu_name_len__ - 1);
    pos += __obu_name_len__;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->retcode, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->start_reason, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int cr_start_auto_respond::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->obu_name.size() + 4 + 1;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    return enc_size;
}

int64_t cr_start_auto_respond::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0x7cb7fff6afff325cLL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif

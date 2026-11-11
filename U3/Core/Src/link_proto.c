/**
  ******************************************************************************
  * @file    link_proto.c
  * @brief   U2 <-> U3 内部链路的快照编解码
  *
  * 本文件在 U2 和 U3 两个工程里各放一份**完全相同**的副本。
  * 打包顺序必须与 PC 侧 MRE_QUERY 的 D[1..21] 一致，这样两边的读取代码
  * 可以共用同一套字段偏移。
  ******************************************************************************
  */

#include "link_proto.h"

static uint16_t snap_get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void LINK_SnapPack(uint8_t *out, const LINK_Snap_t *s)
{
    out[0]  = s->flags;
    out[1]  = (uint8_t)(s->mre_last >> 8);
    out[2]  = (uint8_t)(s->mre_last);
    out[3]  = (uint8_t)(s->mre_min >> 8);
    out[4]  = (uint8_t)(s->mre_min);
    out[5]  = (uint8_t)(s->mre_max >> 8);
    out[6]  = (uint8_t)(s->mre_max);
    out[7]  = (uint8_t)(s->mre_mean >> 8);
    out[8]  = (uint8_t)(s->mre_mean);
    out[9]  = (uint8_t)(s->mre_mv >> 8);
    out[10] = (uint8_t)(s->mre_mv);
    out[11] = (uint8_t)(s->vref_last >> 8);
    out[12] = (uint8_t)(s->vref_last);
    out[13] = (uint8_t)(s->vref_mv >> 8);
    out[14] = (uint8_t)(s->vref_mv);
    out[15] = (uint8_t)(s->window_size >> 8);
    out[16] = (uint8_t)(s->window_size);
    out[17] = (uint8_t)(s->total_samples >> 24);
    out[18] = (uint8_t)(s->total_samples >> 16);
    out[19] = (uint8_t)(s->total_samples >> 8);
    out[20] = (uint8_t)(s->total_samples);
}

void LINK_SnapUnpack(LINK_Snap_t *s, const uint8_t *in)
{
    s->flags        = in[0];
    s->mre_last     = snap_get_u16(&in[1]);
    s->mre_min      = snap_get_u16(&in[3]);
    s->mre_max      = snap_get_u16(&in[5]);
    s->mre_mean     = snap_get_u16(&in[7]);
    s->mre_mv       = snap_get_u16(&in[9]);
    s->vref_last    = snap_get_u16(&in[11]);
    s->vref_mv      = snap_get_u16(&in[13]);
    s->window_size  = snap_get_u16(&in[15]);
    s->total_samples = ((uint32_t)in[17] << 24) | ((uint32_t)in[18] << 16) |
                       ((uint32_t)in[19] << 8)  |  (uint32_t)in[20];
}

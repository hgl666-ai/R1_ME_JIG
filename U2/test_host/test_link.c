/* 主机侧验证: U2<->U3 内部链路的快照编解码
 * link_proto.c 由 run.ps1 每次从 Core/Src 原样拷进来，测的就是固件里那份。 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "link_proto.h"
#include "link_proto.c"

static int fails = 0;

static void ck(const char *name, int cond)
{
    printf("  %-56s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) { fails++; }
}

int main(void)
{
    printf("== 1. 常量 ==\n");
    ck("LINK_SNAP_LEN == 21", LINK_SNAP_LEN == 21u);
    ck("链路命令码不与 PC 侧 0x00xx 段冲突",
       ((LINK_CMD_PING | LINK_CMD_GET_SNAP) & 0xFF00u) == 0x0100u);

    printf("== 2. 打包/解包的字节序与偏移 ==\n");
    {
        LINK_Snap_t s, t;
        uint8_t buf[LINK_SNAP_LEN];

        memset(&s, 0, sizeof(s));
        s.flags         = 0xA1;
        s.mre_last      = 0x1234;
        s.mre_min       = 0x2345;
        s.mre_max       = 0x3456;
        s.mre_mean      = 0x4567;
        s.mre_mv        = 0x5678;
        s.vref_last     = 0x6789;
        s.vref_mv       = 0x789A;
        s.window_size   = 0x0100;
        s.total_samples = 0xDEADBEEF;

        LINK_SnapPack(buf, &s);

        ck("flags 在 [0]",            buf[0]  == 0xA1);
        ck("mre_last 大端在 [1..2]",  buf[1]  == 0x12 && buf[2]  == 0x34);
        ck("mre_min  大端在 [3..4]",  buf[3]  == 0x23 && buf[4]  == 0x45);
        ck("mre_max  大端在 [5..6]",  buf[5]  == 0x34 && buf[6]  == 0x56);
        ck("mre_mean 大端在 [7..8]",  buf[7]  == 0x45 && buf[8]  == 0x67);
        ck("mre_mv   大端在 [9..10]", buf[9]  == 0x56 && buf[10] == 0x78);
        ck("vref_last 大端在 [11..12]", buf[11] == 0x67 && buf[12] == 0x89);
        ck("vref_mv  大端在 [13..14]", buf[13] == 0x78 && buf[14] == 0x9A);
        ck("window_size 大端在 [15..16]", buf[15] == 0x01 && buf[16] == 0x00);
        ck("total_samples 大端在 [17..20]",
           buf[17] == 0xDE && buf[18] == 0xAD && buf[19] == 0xBE && buf[20] == 0xEF);

        memset(&t, 0, sizeof(t));
        LINK_SnapUnpack(&t, buf);
        ck("往返后所有字段一致", memcmp(&s, &t, sizeof(s)) == 0);
    }

    printf("== 3. 与 PC 侧 MRE_QUERY 布局等价（用一帧实机应答验证）==\n");
    {
        /* 2026-11-06 实机 MRE_QUERY 应答的数据段 D[0..21]：
         *   D[0]=状态码 0x00，D[1..21] 就是链路快照的 21 字节 */
        static const uint8_t d[22] = {
            0x00, 0x03, 0x0A, 0x3B, 0x0A, 0x37, 0x0A, 0x3D,
            0x0A, 0x3A, 0x08, 0x3E, 0x07, 0xFB, 0x06, 0x6E,
            0x01, 0x00, 0x00, 0x16, 0x83, 0x00
        };
        LINK_Snap_t s;

        LINK_SnapUnpack(&s, &d[1]);

        printf("    解出: flags=0x%02X last=%u min=%u max=%u mean=%u mv=%u vref=%u/%umV win=%u total=%lu\n",
               s.flags, s.mre_last, s.mre_min, s.mre_max, s.mre_mean, s.mre_mv,
               s.vref_last, s.vref_mv, s.window_size, (unsigned long)s.total_samples);

        ck("flags == 0x03",             s.flags        == 0x03);
        ck("mre_last == 2619",          s.mre_last     == 2619);
        ck("mre_min  == 2615",          s.mre_min      == 2615);
        ck("mre_max  == 2621",          s.mre_max      == 2621);
        ck("mre_mean == 2618",          s.mre_mean     == 2618);
        ck("mre_mv   == 2110",          s.mre_mv       == 2110);
        ck("vref_last == 2043",         s.vref_last    == 2043);
        ck("vref_mv  == 1646",          s.vref_mv      == 1646);
        ck("window_size == 256",        s.window_size  == 256);
        ck("total_samples == 1475328",  s.total_samples == 1475328u);
    }

    printf("\n%s  (fails=%d)\n", fails ? "*** FAILURES ***" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}

/* 主机侧验证: 帧 CRC16 / 组帧 / 解帧 / 字节序
 * protocol_frame.c 与 tmc_crc8() 都由 run.ps1 每次从 Core/Src 原样抽取,
 * 这里测的是固件里跑的同一份代码。 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "crc8_extract.c"       /* 从 bsp_tmc2209.c 抽出的 tmc_crc8() */

#include "protocol_frame.h"
#include "protocol_frame.c"

static int fails = 0;

static void ck(const char *name, int cond)
{
    printf("  %-56s %s\n", name, cond ? "PASS" : "FAIL");
    if (!cond) { fails++; }
}

int main(void)
{
    printf("== 1. CRC16/MODBUS 标准向量 ==\n");
    {
        const uint8_t *v = (const uint8_t *)"123456789";
        uint16_t c = Frame_CRC16(v, 9);
        printf("  CRC16(\"123456789\") = 0x%04X  期望 0x4B37\n", c);
        ck("CRC16 MODBUS 标准向量", c == 0x4B37);
    }

    printf("== 2. CRC8 (Trinamic) 自检向量 ==\n");
    {
        uint8_t a[1] = {0x05};
        uint8_t b[3] = {0x05, 0x00, 0x00};
        uint8_t ca = tmc_crc8(a, 1);
        uint8_t cb = tmc_crc8(b, 3);
        printf("  CRC8([05])=0x%02X (期望 0x69)   CRC8([05,00,00])=0x%02X (期望 0x48)\n", ca, cb);
        ck("CRC8 Trinamic 自检向量", (ca == 0x69u) && (cb == 0x48u));
    }

    printf("== 3. 组帧/解帧往返 (L=0/1/7/256) ==\n");
    {
        uint16_t lens[4] = {0, 1, 7, 256};
        int k;

        for (k = 0; k < 4; k++) {
            uint8_t data[256];
            uint8_t out[256];
            uint8_t buf[FRAME_BUF_SIZE];
            uint16_t n, cmd = 0, seq = 0, len = 0;
            uint16_t i;
            char name[96];
            int ok_rt;

            for (i = 0; i < lens[k]; i++) { data[i] = (uint8_t)(i * 37u + 11u); }

            n = Frame_Build(buf, 0x0020, 0x1234, data, lens[k]);
            sprintf(name, "L=%u 整帧长度 = 10+L", lens[k]);
            ck(name, n == (uint16_t)(10u + lens[k]));

            memset(out, 0, sizeof(out));
            if (Frame_Parse(buf, n, &cmd, &seq, out, &len)) {
                ok_rt = (cmd == 0x0020) && (seq == 0x1234) && (len == lens[k]) &&
                        ((lens[k] == 0) || (memcmp(out, data, lens[k]) == 0));
            } else {
                ok_rt = 0;
            }
            sprintf(name, "L=%u 往返: cmd/seq/len/data 全一致", lens[k]);
            ck(name, ok_rt);
        }
    }

    printf("== 4. 错误帧必须被拒绝 ==\n");
    {
        uint8_t data[4] = {1, 2, 3, 4};
        uint16_t cmd, seq, len;
        uint8_t out[256];
        uint8_t buf[FRAME_BUF_SIZE];
        uint16_t n = Frame_Build(buf, 0x0001, 0x0101, data, 4);

        buf[0] = 0xA4u;
        ck("帧头错误 -> 拒绝", Frame_Parse(buf, n, &cmd, &seq, out, &len) == 0);
        buf[0] = 0xA5u;

        buf[10] ^= 0xFFu;
        ck("CRC 错误 -> 拒绝", Frame_Parse(buf, n, &cmd, &seq, out, &len) == 0);
        buf[10] ^= 0xFFu;

        ck("长度字段与实到字节数不符 -> 拒绝",
           Frame_Parse(buf, (uint16_t)(n - 1u), &cmd, &seq, out, &len) == 0);
        ck("原帧 -> 接受", Frame_Parse(buf, n, &cmd, &seq, out, &len) == 1);
    }

    printf("== 5. 字节序 / 与文档样例帧一致 ==\n");
    {
        static const uint8_t want_move[] = {
            0xA5, 0x5B, 0x00, 0x10, 0x00, 0x01, 0x00, 0x07,
            0x00, 0x00, 0x00, 0x0C, 0x80, 0x0F, 0xA0, 0xB3, 0xA9
        };
        static const uint8_t want_ping[] = {
            0xA5, 0x5B, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x89, 0x0C
        };
        uint8_t buf[FRAME_BUF_SIZE];
        uint8_t move_data[7] = {0x00, 0x00, 0x00, 0x0C, 0x80, 0x0F, 0xA0};
        uint16_t n, i;

        n = Frame_Build(buf, 0x0010, 0x0001, move_data, 7);
        printf("  MOVE 3200步/4000Hz/正转: ");
        for (i = 0; i < n; i++) { printf("%02X ", buf[i]); }
        printf("\n");
        ck("与 TESTING.md / PROTOCOL.md 的 MOVE 样例帧逐字节一致",
           (n == sizeof(want_move)) && (memcmp(buf, want_move, sizeof(want_move)) == 0));

        n = Frame_Build(buf, 0x0001, 0x0001, NULL, 0);
        printf("  PING: ");
        for (i = 0; i < n; i++) { printf("%02X ", buf[i]); }
        printf("\n");
        ck("与文档里的 PING 样例帧逐字节一致",
           (n == sizeof(want_ping)) && (memcmp(buf, want_ping, sizeof(want_ping)) == 0));

        ck("CMD/SEQ/LEN/CRC 均为高字节在前",
           (buf[0] == 0xA5) && (buf[1] == 0x5B) &&
           (buf[2] == 0x00) && (buf[3] == 0x01) &&
           (buf[4] == 0x00) && (buf[5] == 0x01));
    }

    printf("\n%s  (fails=%d)\n", fails ? "*** FAILURES ***" : "ALL TESTS PASSED", fails);
    return fails ? 1 : 0;
}

static uint8_t tmc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t cur;
    uint8_t i, j;

    for (i = 0; i < len; i++) {
        cur = data[i];
        for (j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (cur & 0x01)) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
            cur >>= 1;
        }
    }
    return crc;
}


#include "spi.h"
#include "main.h"
#include "ksz8851.h"

static inline void ksz_cs_low(void)  { HAL_GPIO_WritePin(KSZ_CS_GPIO_Port, KSZ_CS_Pin, GPIO_PIN_RESET); }
static inline void ksz_cs_high(void) { HAL_GPIO_WritePin(KSZ_CS_GPIO_Port, KSZ_CS_Pin, GPIO_PIN_SET); }


static void ksz_cmd_build(uint8_t *cmd, uint8_t opcode, uint16_t reg)
{
    // opcode in [7:6], byte-enable in [5:2], A[7:6] in [1:0]
    cmd[0] = ((opcode & 0x03) << 6)
           | ((KSZ_BYTE_EN_2B & 0x0F) << 2)
           | ((reg >> 6) & 0x03);

    // second byte = reg[5:0] -> [7:2], low 2 bits don’t care
    cmd[1] = (uint8_t)(reg << 2);
}

HAL_StatusTypeDef ksz_read16(uint16_t reg, uint16_t *out)
{
    uint8_t cmd[2];
    uint8_t rx[2];
    uint8_t dummy[2] = {0xFF, 0xFF};
    ksz_cmd_build(cmd, KSZ_OP_REG_READ, reg);

    ksz_cs_low();
    HAL_StatusTypeDef s = HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);
    if (s == HAL_OK) {
        // clock out two data bytes
        s = HAL_SPI_TransmitReceive(&hspi1, dummy, rx, 2, HAL_MAX_DELAY);
    }
    ksz_cs_high();
    if (s == HAL_OK) *out = (uint16_t)(rx[1] << 8 | rx[0]); // byte0 first, then byte1
    return s;
}

HAL_StatusTypeDef ksz_write16(uint16_t reg, uint16_t val)
{
    uint8_t cmd[2];
    uint8_t tx[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    ksz_cmd_build(cmd, KSZ_OP_REG_WRITE, reg);

    ksz_cs_low();
    HAL_StatusTypeDef s = HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);
    if (s == HAL_OK) s = HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    ksz_cs_high();
    return s;
}

// Optional: quick probe & basic enable
HAL_StatusTypeDef ksz_probe_and_enable(void)
{
    // If you control RSTN pin, hold low then high; after power-up wait >=10 ms
    HAL_Delay(10);

    uint16_t cider = 0;
    HAL_StatusTypeDef s = ksz_read16(KSZ_REG_CIDER, &cider);
    if (s != HAL_OK) return s;

    // Family ID should be 0x88, Chip ID nibble 0x7 for KSZ8851SNL
    if ( (cider & 0xFF00) != 0x8800 ) return HAL_ERROR;

    // Example: set a MAC (01:23:45:67:89:AB)
    ksz_write16(KSZ_REG_MARH, 0x0123);
    ksz_write16(KSZ_REG_MARM, 0x4567);
    ksz_write16(KSZ_REG_MARL, 0x89AB);

    // Enable RX: unicast + multicast + RX engine
    uint16_t rxcr1 = RXCR1_RXUE | RXCR1_RXME | RXCR1_RXE;
    return ksz_write16(KSZ_REG_RXCR1, rxcr1);
}

// --- MAC helpers ------------------------------------------------------------
HAL_StatusTypeDef ksz_set_mac(const uint8_t mac[6])
{
    // KSZ stores MAC in 3 x 16-bit little-endian registers:
    // MARH = MAC[1:0], MARM = MAC[3:2], MARL = MAC[5:4]
    uint16_t h = ((uint16_t)mac[1] << 8) | mac[0];
    uint16_t m = ((uint16_t)mac[3] << 8) | mac[2];
    uint16_t l = ((uint16_t)mac[5] << 8) | mac[4];

    HAL_StatusTypeDef s;
    s = ksz_write16(KSZ_REG_MARH, h); if (s != HAL_OK) return s;
    s = ksz_write16(KSZ_REG_MARM, m); if (s != HAL_OK) return s;
    s = ksz_write16(KSZ_REG_MARL, l); return s;
}

HAL_StatusTypeDef ksz_get_mac(uint8_t mac[6])
{
    uint16_t h, m, l;
    HAL_StatusTypeDef s;
    s = ksz_read16(KSZ_REG_MARH, &h); if (s != HAL_OK) return s;
    s = ksz_read16(KSZ_REG_MARM, &m); if (s != HAL_OK) return s;
    s = ksz_read16(KSZ_REG_MARL, &l); if (s != HAL_OK) return s;

    mac[0] = (uint8_t)(h & 0xFF);
    mac[1] = (uint8_t)(h >> 8);
    mac[2] = (uint8_t)(m & 0xFF);
    mac[3] = (uint8_t)(m >> 8);
    mac[4] = (uint8_t)(l & 0xFF);
    mac[5] = (uint8_t)(l >> 8);
    return HAL_OK;
}

// --- Probe & basic enable (refactored) --------------------------------------
HAL_StatusTypeDef ksz_probe(uint16_t *cider_out)
{
    uint16_t cider = 0;
    HAL_StatusTypeDef s = ksz_read16(KSZ_REG_CIDER, &cider);
    if (s != HAL_OK) return s;

    if ((cider & 0xFF00u) != 0x8800u) return HAL_ERROR;  // family ID 0x88
    if (cider_out) *cider_out = cider;
    return HAL_OK;
}

HAL_StatusTypeDef ksz_enable_rx_basic(void)
{
    uint16_t rxcr1 = RXCR1_RXUE | RXCR1_RXME | RXCR1_RXE;
    return ksz_write16(KSZ_REG_RXCR1, rxcr1);
}

void ksz_init(void)
{
    uint16_t cider;
    if (ksz_probe(&cider) == HAL_OK) {
        printf("KSZ8851 detected, CIDER=0x%04X\r\n", cider);

        // Set your real MAC
        const uint8_t my_mac[6] = {0x02,0x11,0x22,0x33,0x44,0x55}; // LAA/unicast
        if (ksz_set_mac(my_mac) == HAL_OK) {
            uint8_t rd[6];
            ksz_get_mac(rd);
            printf("MAC set to %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                rd[0], rd[1], rd[2], rd[3], rd[4], rd[5]);
        }

        // Turn on receiver (simple defaults)
        ksz_enable_rx_basic();
    } else {
        printf("KSZ8851 not responding\r\n");
    }

}

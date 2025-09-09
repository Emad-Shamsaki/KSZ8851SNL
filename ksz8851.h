#include "spi.h"
#include "main.h"

// --- KSZ8851 SPI opcodes / bit positions (from datasheet) ---
#define KSZ_OP_REG_READ    0x00  // opcode bits [7:6] = 00
#define KSZ_OP_REG_WRITE   0x40  // opcode bits [7:6] = 01
#define KSZ_OP_RXFIFO_READ 0x80  // opcode bits [7:6] = 10
#define KSZ_OP_TXFIFO_WR   0xC0  // opcode bits [7:6] = 11

#define KSZ_BYTE_EN_2B     0x03  // enable bytes 0 and 1
#define KSZ_BYTE_EN_SHIFT  2
#define KSZ_ADDR_SHIFT     2     // A[7:2] total 6 bits
#define KSZ_ADDR_MSB_POS   6     // A[7:6] go in cmd byte0 bits [1:0]
#define KSZ_ADDR_MSB_MASK  0x03

// A few useful register addresses
#define KSZ_REG_MARL       0x10  // MAC Addr Low  (16-bit)
#define KSZ_REG_MARM       0x12  // MAC Addr Mid  (16-bit)
#define KSZ_REG_MARH       0x14  // MAC Addr High (16-bit)
#define KSZ_REG_RXCR1      0x74  // Receive Control 1 (16-bit)
#define KSZ_REG_RXCR2      0x76  // Receive Control 2 (16-bit)
#define KSZ_REG_CIDER      0xC0  // Chip ID & Enable (16-bit)

// RXCR1 bits we typically want
#define RXCR1_RXE          (1u << 0)  // RX Enable
#define RXCR1_RXINVF       (1u << 1)  // Inverse filter (0 normally)
#define RXCR1_RXAE         (1u << 4)  // Receive All
#define RXCR1_RXUE         (1u << 5)  // Unicast Enable
#define RXCR1_RXME         (1u << 6)  // Multicast Enable


static inline void ksz_cs_low(void) ;
static inline void ksz_cs_high(void);
HAL_StatusTypeDef ksz_read16(uint16_t reg, uint16_t *out);
HAL_StatusTypeDef ksz_write16(uint16_t reg, uint16_t val);
HAL_StatusTypeDef ksz_probe_and_enable(void);


HAL_StatusTypeDef ksz_probe(uint16_t *cider_out);
HAL_StatusTypeDef ksz_set_mac(const uint8_t mac[6]);
HAL_StatusTypeDef ksz_get_mac(uint8_t mac[6]);
HAL_StatusTypeDef ksz_enable_rx_basic(void);

void ksz_init(void);
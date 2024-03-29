/*
 * Copyright (c) 2013 Nuvoton Technology Corp.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 * Description:   NUC472 MAC driver source file
 */
#include "nuc472_eth.h"
#include "lwip/opt.h"
#include "lwip/def.h"


#define ETH_TRIGGER_RX()    do{EMAC->RXST = 0;}while(0)
#define ETH_TRIGGER_TX()    do{EMAC->TXST = 0;}while(0)
#define ETH_ENABLE_TX()     do{EMAC->CTL |= EMAC_CTL_TXON;}while(0)
#define ETH_ENABLE_RX()     do{EMAC->CTL |= EMAC_CTL_RXON;}while(0)
#define ETH_DISABLE_TX()    do{EMAC->CTL &= ~EMAC_CTL_TXON;}while(0)
#define ETH_DISABLE_RX()    do{EMAC->CTL &= ~EMAC_CTL_RXON;}while(0)

#ifdef __ICCARM__
#pragma data_alignment=4
struct eth_descriptor rx_desc[RX_DESCRIPTOR_NUM];
struct eth_descriptor tx_desc[TX_DESCRIPTOR_NUM];
#else
struct eth_descriptor rx_desc[RX_DESCRIPTOR_NUM] __attribute__ ((aligned(4)));
struct eth_descriptor tx_desc[TX_DESCRIPTOR_NUM] __attribute__ ((aligned(4)));
#endif
struct eth_descriptor volatile *cur_tx_desc_ptr, *cur_rx_desc_ptr, *fin_tx_desc_ptr;

u8_t rx_buf[RX_DESCRIPTOR_NUM][PACKET_BUFFER_SIZE];
u8_t tx_buf[TX_DESCRIPTOR_NUM][PACKET_BUFFER_SIZE];

extern void ethernetif_input(u16_t len, u8_t *buf, u32_t s, u32_t ns);
extern void ethernetif_loopback_input(struct pbuf *p);

// PTP source clock is 84MHz (Real chip using PLL). Each tick is 11.90ns
// Assume we want to set each tick to 100ns.
// Increase register = (100 * 2^31) / (10^9) = 214.71 =~ 215 = 0xD7
// Addend register = 2^32 * tick_freq / (84MHz), where tick_freq = (2^31 / 215) MHz
// From above equation, addend register = 2^63 / (84M * 215) ~= 510707200 = 0x1E70C600



static void mdio_write(u8_t addr, u8_t reg, u16_t val)
{

    EMAC->MIIMDAT = val;
    EMAC->MIIMCTL = (addr << EMAC_MIIMCTL_PHYADDR_Pos) | reg | EMAC_MIIMCTL_BUSY_Msk | EMAC_MIIMCTL_WRITE_Msk | EMAC_MIIMCTL_MDCON_Msk;

    while (EMAC->MIIMCTL & EMAC_MIIMCTL_BUSY_Msk);

}


static u16_t mdio_read(u8_t addr, u8_t reg)
{
    EMAC->MIIMCTL = (addr << EMAC_MIIMCTL_PHYADDR_Pos) | reg | EMAC_MIIMCTL_BUSY_Msk | EMAC_MIIMCTL_MDCON_Msk;
    while (EMAC->MIIMCTL & EMAC_MIIMCTL_BUSY_Msk);

    return(EMAC->MIIMDAT);
}

static int reset_phy(void)
{

    u16_t reg;
    u32_t delay;


    mdio_write(CONFIG_PHY_ADDR, MII_BMCR, BMCR_RESET);

    delay = 2000;
    while(delay-- > 0) {
        if((mdio_read(CONFIG_PHY_ADDR, MII_BMCR) & BMCR_RESET) == 0)
            break;

    }

    if(delay == 0) {
        printf("Reset phy failed\n");
        return(-1);
    }

    mdio_write(CONFIG_PHY_ADDR, MII_ADVERTISE, ADVERTISE_CSMA |
               ADVERTISE_10HALF |
               ADVERTISE_10FULL |
               ADVERTISE_100HALF |
               ADVERTISE_100FULL);

    reg = mdio_read(CONFIG_PHY_ADDR, MII_BMCR);
    mdio_write(CONFIG_PHY_ADDR, MII_BMCR, reg | BMCR_ANRESTART);

    delay = 200000;
    while(delay-- > 0) {
        if((mdio_read(CONFIG_PHY_ADDR, MII_BMSR) & (BMSR_ANEGCOMPLETE | BMSR_LSTATUS))
                == (BMSR_ANEGCOMPLETE | BMSR_LSTATUS))
            break;
    }

    if(delay == 0) {
        printf("AN failed. Set to 100 FULL\n");
        EMAC->CTL |= (EMAC_CTL_OPMODE_Msk | EMAC_CTL_FUDUP_Msk);
        return(-1);
    } else {
        reg = mdio_read(CONFIG_PHY_ADDR, MII_LPA);

        if(reg & ADVERTISE_100FULL) {
            printf("100 full\n");
            EMAC->CTL |= (EMAC_CTL_OPMODE_Msk | EMAC_CTL_FUDUP_Msk);
        } else if(reg & ADVERTISE_100HALF) {
            printf("100 half\n");
            EMAC->CTL = (EMAC->CTL & ~EMAC_CTL_FUDUP_Msk) | EMAC_CTL_OPMODE_Msk;
        } else if(reg & ADVERTISE_10FULL) {
            printf("10 full\n");
            EMAC->CTL = (EMAC->CTL & ~EMAC_CTL_OPMODE_Msk) | EMAC_CTL_FUDUP_Msk;
        } else {
            printf("10 half\n");
            EMAC->CTL &= ~(EMAC_CTL_OPMODE_Msk | EMAC_CTL_FUDUP_Msk);
        }
    }

    return(0);
}


static void init_tx_desc(void)
{
    u32_t i;


    cur_tx_desc_ptr = fin_tx_desc_ptr = &tx_desc[0];

    for(i = 0; i < TX_DESCRIPTOR_NUM; i++) {
        tx_desc[i].status1 = TXFD_PADEN | TXFD_CRCAPP | TXFD_INTEN;
        tx_desc[i].buf = &tx_buf[i][0];
        tx_desc[i].status2 = 0;
        tx_desc[i].next = &tx_desc[(i + 1) % TX_DESCRIPTOR_NUM];

    }
    EMAC->TXDSA = (unsigned int)&tx_desc[0];
    return;
}

static void init_rx_desc(void)
{
    u32_t i;


    cur_rx_desc_ptr = &rx_desc[0];

    for(i = 0; i < RX_DESCRIPTOR_NUM; i++) {
        rx_desc[i].status1 = OWNERSHIP_EMAC;
        rx_desc[i].buf = &rx_buf[i][0];
        rx_desc[i].status2 = 0;
        rx_desc[i].next = &rx_desc[(i + 1) % TX_DESCRIPTOR_NUM];
    }
    EMAC->RXDSA = (unsigned int)&rx_desc[0];
    return;
}

static void set_mac_addr(u8_t *addr)
{

    EMAC->CAM0M = (addr[0] << 24) |
                  (addr[1] << 16) |
                  (addr[2] << 8) |
                  addr[3];

    EMAC->CAM0L = (addr[4] << 24) |
                  (addr[5] << 16);

    EMAC->CAMCTL = EMAC_CAMCTL_CMPEN_Msk | EMAC_CAMCTL_AMP_Msk | EMAC_CAMCTL_ABP_Msk;
    EMAC->CAMEN = 1;    // Enable CAM entry 0

}

static void __eth_clk_pin_init()
{
	 /* Enable IP clock */
    CLK_EnableModuleClock(EMAC_MODULE);
    // Configure MDC clock rate to HCLK / (127 + 1) = 656 kHz if system is running at 84 MHz
    CLK_SetModuleClock(EMAC_MODULE, 0, CLK_CLKDIV3_EMAC(127));
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    // Configure RMII pins
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_EMAC_REFCLK |
                    SYS_GPC_MFPL_PC1MFP_EMAC_MII_RXERR |
                    SYS_GPC_MFPL_PC2MFP_EMAC_MII_RXDV |
                    SYS_GPC_MFPL_PC3MFP_EMAC_MII_RXD1 |
                    SYS_GPC_MFPL_PC4MFP_EMAC_MII_RXD0 |
                    SYS_GPC_MFPL_PC6MFP_EMAC_MII_TXD0 |
                    SYS_GPC_MFPL_PC7MFP_EMAC_MII_TXD1;


    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC8MFP_EMAC_MII_TXEN;
    // Enable high slew rate on all RMII pins
    PC->SLEWCTL |= 0x1DF;

    // Configure MDC, MDIO at PB14 & PB15
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_EMAC_MII_MDC | SYS_GPB_MFPH_PB15MFP_EMAC_MII_MDIO;

}

void ETH_init(u8_t *mac_addr)
{
	// init CLK & pins
	__eth_clk_pin_init();
	
    // Reset MAC
    EMAC->CTL = EMAC_CTL_RST_Msk;

    init_tx_desc();
    init_rx_desc();

    set_mac_addr(mac_addr);  // need to reconfigure hardware address 'cos we just RESET emc...
    reset_phy();

    EMAC->CTL |= EMAC_CTL_STRIPCRC_Msk | EMAC_CTL_RXON_Msk | EMAC_CTL_TXON_Msk | EMAC_CTL_RMIIEN_Msk | EMAC_CTL_RMIIRXCTL_Msk;
    EMAC->INTEN |= EMAC_INTEN_RXIEN_Msk |
                   EMAC_INTEN_RXGDIEN_Msk |
                   EMAC_INTEN_RDUIEN_Msk |
                   EMAC_INTEN_RXBEIEN_Msk |
                   EMAC_INTEN_TXIEN_Msk |
                   EMAC_INTEN_TXABTIEN_Msk |
                   EMAC_INTEN_TXCPIEN_Msk |
                   EMAC_INTEN_TXBEIEN_Msk;
    EMAC->RXST = 0;  // trigger Rx
}



void  ETH_halt(void)
{

    EMAC->CTL &= ~(EMAC_CTL_RXON_Msk | EMAC_CTL_TXON_Msk);
}



void EMAC_RX_IRQHandler(void)
{
    unsigned int cur_entry, status;

    status = EMAC->INTSTS & 0xFFFF;
    EMAC->INTSTS = status;
    if (status & EMAC_INTSTS_RXBEIF_Msk) {
        // Shouldn't goes here, unless descriptor corrupted
    }

    do {

        cur_entry = EMAC->CRXDSA;

        if ((cur_entry == (u32_t)cur_rx_desc_ptr) && (!(status & EMAC_INTSTS_RDUIF_Msk)))  // cur_entry may equal to cur_rx_desc_ptr if RDU occures
            break;
        status = cur_rx_desc_ptr->status1;

        if(status & OWNERSHIP_EMAC)
            break;

        if (status & RXFD_RXGD) {


            ethernetif_input(status & 0xFFFF, cur_rx_desc_ptr->buf, cur_rx_desc_ptr->status2, (u32_t)cur_rx_desc_ptr->next);


        }

        cur_rx_desc_ptr->status1 = OWNERSHIP_EMAC;
        cur_rx_desc_ptr = cur_rx_desc_ptr->next;

    } while (1);

    ETH_TRIGGER_RX();
	eth_arch_tcpip_thread();
}

void EMAC_TX_IRQHandler(void)
{
    unsigned int cur_entry, status;

    status = EMAC->INTSTS & 0xFFFF0000;
    EMAC->INTSTS = status;
    if(status & EMAC_INTSTS_TXBEIF_Msk) {
        // Shouldn't goes here, unless descriptor corrupted
        return;
    }

    cur_entry = EMAC->CTXDSA;

    while (cur_entry != (u32_t)fin_tx_desc_ptr) {

        fin_tx_desc_ptr = fin_tx_desc_ptr->next;
    }

}

u8_t *ETH_get_tx_buf(void)
{
    if(cur_tx_desc_ptr->status1 & OWNERSHIP_EMAC)
        return(NULL);
    else
        return(cur_tx_desc_ptr->buf);
}

void ETH_trigger_tx(u16_t length, struct pbuf *p)
{
    struct eth_descriptor volatile *desc;
    cur_tx_desc_ptr->status2 = (unsigned int)length;
    desc = cur_tx_desc_ptr->next;    // in case TX is transmitting and overwrite next pointer before we can update cur_tx_desc_ptr
    cur_tx_desc_ptr->status1 |= OWNERSHIP_EMAC;
    cur_tx_desc_ptr = desc;

    ETH_TRIGGER_TX();

}

int ETH_link_ok()
{
	/* first, a dummy read to latch */
	mdio_read(CONFIG_PHY_ADDR, MII_BMSR);
	if(mdio_read(CONFIG_PHY_ADDR, MII_BMSR) & BMSR_LSTATUS)
		return 1;
	return 0;	
}
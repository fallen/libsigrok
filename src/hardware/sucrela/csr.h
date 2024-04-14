#define CSR_BASE 0x0L
#define CSR_ANALYZER_BASE (CSR_BASE + 0x0L)
#define CSR_ANALYZER_MUX_VALUE_ADDR (CSR_BASE + 0x0L)
#define CSR_ANALYZER_MUX_VALUE_SIZE 1
#define CSR_ANALYZER_TRIGGER_ENABLE_ADDR (CSR_BASE + 0x4L)
#define CSR_ANALYZER_TRIGGER_ENABLE_SIZE 1
#define CSR_ANALYZER_TRIGGER_DONE_ADDR (CSR_BASE + 0x8L)
#define CSR_ANALYZER_TRIGGER_DONE_SIZE 1
#define CSR_ANALYZER_TRIGGER_MEM_WRITE_ADDR (CSR_BASE + 0xcL)
#define CSR_ANALYZER_TRIGGER_MEM_WRITE_SIZE 1
#define CSR_ANALYZER_TRIGGER_MEM_MASK_ADDR (CSR_BASE + 0x10L)
#define CSR_ANALYZER_TRIGGER_MEM_MASK_SIZE 1
#define CSR_ANALYZER_TRIGGER_MEM_VALUE_ADDR (CSR_BASE + 0x14L)
#define CSR_ANALYZER_TRIGGER_MEM_VALUE_SIZE 1
#define CSR_ANALYZER_TRIGGER_MEM_FULL_ADDR (CSR_BASE + 0x18L)
#define CSR_ANALYZER_TRIGGER_MEM_FULL_SIZE 1
#define CSR_ANALYZER_SUBSAMPLER_VALUE_ADDR (CSR_BASE + 0x1cL)
#define CSR_ANALYZER_SUBSAMPLER_VALUE_SIZE 1
#define CSR_ANALYZER_STORAGE_ENABLE_ADDR (CSR_BASE + 0x20L)
#define CSR_ANALYZER_STORAGE_ENABLE_SIZE 1
#define CSR_ANALYZER_STORAGE_DONE_ADDR (CSR_BASE + 0x24L)
#define CSR_ANALYZER_STORAGE_DONE_SIZE 1
#define CSR_ANALYZER_STORAGE_LENGTH_ADDR (CSR_BASE + 0x28L)
#define CSR_ANALYZER_STORAGE_LENGTH_SIZE 1
#define CSR_ANALYZER_STORAGE_OFFSET_ADDR (CSR_BASE + 0x2cL)
#define CSR_ANALYZER_STORAGE_OFFSET_SIZE 1
#define CSR_ANALYZER_STORAGE_MEM_LEVEL_ADDR (CSR_BASE + 0x30L)
#define CSR_ANALYZER_STORAGE_MEM_LEVEL_SIZE 1
#define CSR_ANALYZER_STORAGE_MEM_DATA_ADDR (CSR_BASE + 0x34L)
#define CSR_ANALYZER_STORAGE_MEM_DATA_SIZE 1
#define CSR_CTRL_BASE (CSR_BASE + 0x800L)
#define CSR_CTRL_RESET_ADDR (CSR_BASE + 0x800L)
#define CSR_CTRL_RESET_SIZE 1
#define CSR_CTRL_RESET_SOC_RST_OFFSET 0
#define CSR_CTRL_RESET_SOC_RST_SIZE 1
#define CSR_CTRL_RESET_CPU_RST_OFFSET 1
#define CSR_CTRL_RESET_CPU_RST_SIZE 1
#define CSR_CTRL_SCRATCH_ADDR (CSR_BASE + 0x804L)
#define CSR_CTRL_SCRATCH_SIZE 1
#define CSR_CTRL_BUS_ERRORS_ADDR (CSR_BASE + 0x808L)
#define CSR_CTRL_BUS_ERRORS_SIZE 1
#define CSR_DDRPHY_BASE (CSR_BASE + 0x1000L)
#define CSR_DDRPHY_DLY_SEL_ADDR (CSR_BASE + 0x1000L)
#define CSR_DDRPHY_DLY_SEL_SIZE 1
#define CSR_DDRPHY_RDLY_DQ_RST_ADDR (CSR_BASE + 0x1004L)
#define CSR_DDRPHY_RDLY_DQ_RST_SIZE 1
#define CSR_DDRPHY_RDLY_DQ_INC_ADDR (CSR_BASE + 0x1008L)
#define CSR_DDRPHY_RDLY_DQ_INC_SIZE 1
#define CSR_DDRPHY_RDLY_DQ_BITSLIP_RST_ADDR (CSR_BASE + 0x100cL)
#define CSR_DDRPHY_RDLY_DQ_BITSLIP_RST_SIZE 1
#define CSR_DDRPHY_RDLY_DQ_BITSLIP_ADDR (CSR_BASE + 0x1010L)
#define CSR_DDRPHY_RDLY_DQ_BITSLIP_SIZE 1
#define CSR_DDRPHY_BURSTDET_CLR_ADDR (CSR_BASE + 0x1014L)
#define CSR_DDRPHY_BURSTDET_CLR_SIZE 1
#define CSR_DDRPHY_BURSTDET_SEEN_ADDR (CSR_BASE + 0x1018L)
#define CSR_DDRPHY_BURSTDET_SEEN_SIZE 1
#define CSR_IDENTIFIER_MEM_BASE (CSR_BASE + 0x1800L)
#define CSR_LA_BASE (CSR_BASE + 0x2000L)
#define CSR_LA_HSPI_TEST_PATTERN_ADDR (CSR_BASE + 0x2000L)
#define CSR_LA_HSPI_TEST_PATTERN_SIZE 1
#define CSR_LA_TRIGGER_ENABLE_ADDR (CSR_BASE + 0x2004L)
#define CSR_LA_TRIGGER_ENABLE_SIZE 1
#define CSR_LA_TRIGGER_DONE_ADDR (CSR_BASE + 0x2008L)
#define CSR_LA_TRIGGER_DONE_SIZE 1
#define CSR_LA_TRIGGER_MEM_WRITE_ADDR (CSR_BASE + 0x200cL)
#define CSR_LA_TRIGGER_MEM_WRITE_SIZE 1
#define CSR_LA_TRIGGER_MEM_MASK_ADDR (CSR_BASE + 0x2010L)
#define CSR_LA_TRIGGER_MEM_MASK_SIZE 1
#define CSR_LA_TRIGGER_MEM_VALUE_ADDR (CSR_BASE + 0x2014L)
#define CSR_LA_TRIGGER_MEM_VALUE_SIZE 1
#define CSR_LA_TRIGGER_MEM_FULL_ADDR (CSR_BASE + 0x2018L)
#define CSR_LA_TRIGGER_MEM_FULL_SIZE 1
#define CSR_LA_SUBSAMPLER_VALUE_ADDR (CSR_BASE + 0x201cL)
#define CSR_LA_SUBSAMPLER_VALUE_SIZE 1
#define CSR_LA_STORAGE_ENABLE_ADDR (CSR_BASE + 0x2020L)
#define CSR_LA_STORAGE_ENABLE_SIZE 1
#define CSR_LA_STORAGE_DONE_ADDR (CSR_BASE + 0x2024L)
#define CSR_LA_STORAGE_DONE_SIZE 1
#define CSR_LA_STORAGE_FSM_STATE_R_ADDR (CSR_BASE + 0x2028L)
#define CSR_LA_STORAGE_FSM_STATE_R_SIZE 1
#define CSR_LA_STORAGE_LENGTH_ADDR (CSR_BASE + 0x202cL)
#define CSR_LA_STORAGE_LENGTH_SIZE 1
#define CSR_LA_STORAGE_OFFSET_ADDR (CSR_BASE + 0x2030L)
#define CSR_LA_STORAGE_OFFSET_SIZE 1
#define CSR_LA_STORAGE_MEM_LEVEL_ADDR (CSR_BASE + 0x2034L)
#define CSR_LA_STORAGE_MEM_LEVEL_SIZE 1
#define CSR_LA_STORAGE_MEM_DATA_ADDR (CSR_BASE + 0x2038L)
#define CSR_LA_STORAGE_MEM_DATA_SIZE 1
#define CSR_LA_STORAGE_REASON_CSR_ADDR (CSR_BASE + 0x203cL)
#define CSR_LA_STORAGE_REASON_CSR_SIZE 1
#define CSR_LA_HSPI_TX_STATE_R_ADDR (CSR_BASE + 0x2040L)
#define CSR_LA_HSPI_TX_STATE_R_SIZE 1
#define CSR_LA_HSPI_TX_REASON_R_ADDR (CSR_BASE + 0x2044L)
#define CSR_LA_HSPI_TX_REASON_R_SIZE 1
#define CSR_LA_HSPI_TX_ENABLE_ADDR (CSR_BASE + 0x2048L)
#define CSR_LA_HSPI_TX_ENABLE_SIZE 1
#define CSR_LA_HSPI_TX_MAX_PACKET_SIZE_ADDR (CSR_BASE + 0x204cL)
#define CSR_LA_HSPI_TX_MAX_PACKET_SIZE_SIZE 1
#define CSR_LA_HSPI_TX_LAST_CRC_SENT_CSR_ADDR (CSR_BASE + 0x2050L)
#define CSR_LA_HSPI_TX_LAST_CRC_SENT_CSR_SIZE 1
#define CSR_LA_HSPI_TX_MAX_PACKET_NUM_R_ADDR (CSR_BASE + 0x2054L)
#define CSR_LA_HSPI_TX_MAX_PACKET_NUM_R_SIZE 1
#define CSR_LA_HSPI_TX_HSPI_PADS_CSR_ADDR (CSR_BASE + 0x2058L)
#define CSR_LA_HSPI_TX_HSPI_PADS_CSR_SIZE 1
#define CSR_LA_HSPI_TX_WAIT_INPUT_CC_CSR_ADDR (CSR_BASE + 0x205cL)
#define CSR_LA_HSPI_TX_WAIT_INPUT_CC_CSR_SIZE 1
#define CSR_LA_HSPI_TX_WAIT_TX_READY_CC_CSR_ADDR (CSR_BASE + 0x2060L)
#define CSR_LA_HSPI_TX_WAIT_TX_READY_CC_CSR_SIZE 1
#define CSR_LA_HSPI_TX_TX_HEADER_CC_CSR_ADDR (CSR_BASE + 0x2064L)
#define CSR_LA_HSPI_TX_TX_HEADER_CC_CSR_SIZE 1
#define CSR_LA_HSPI_TX_TX_DATA_CC_CSR_ADDR (CSR_BASE + 0x2068L)
#define CSR_LA_HSPI_TX_TX_DATA_CC_CSR_SIZE 1
#define CSR_LEDS_BASE (CSR_BASE + 0x2800L)
#define CSR_LEDS_OUT_ADDR (CSR_BASE + 0x2800L)
#define CSR_LEDS_OUT_SIZE 1
#define CSR_SDRAM_BASE (CSR_BASE + 0x3000L)
#define CSR_SDRAM_DFII_CONTROL_ADDR (CSR_BASE + 0x3000L)
#define CSR_SDRAM_DFII_CONTROL_SIZE 1
#define CSR_SDRAM_DFII_CONTROL_SEL_OFFSET 0
#define CSR_SDRAM_DFII_CONTROL_SEL_SIZE 1
#define CSR_SDRAM_DFII_CONTROL_CKE_OFFSET 1
#define CSR_SDRAM_DFII_CONTROL_CKE_SIZE 1
#define CSR_SDRAM_DFII_CONTROL_ODT_OFFSET 2
#define CSR_SDRAM_DFII_CONTROL_ODT_SIZE 1
#define CSR_SDRAM_DFII_CONTROL_RESET_N_OFFSET 3
#define CSR_SDRAM_DFII_CONTROL_RESET_N_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_ADDR (CSR_BASE + 0x3004L)
#define CSR_SDRAM_DFII_PI0_COMMAND_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_CS_OFFSET 0
#define CSR_SDRAM_DFII_PI0_COMMAND_CS_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_WE_OFFSET 1
#define CSR_SDRAM_DFII_PI0_COMMAND_WE_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_CAS_OFFSET 2
#define CSR_SDRAM_DFII_PI0_COMMAND_CAS_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_RAS_OFFSET 3
#define CSR_SDRAM_DFII_PI0_COMMAND_RAS_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_WREN_OFFSET 4
#define CSR_SDRAM_DFII_PI0_COMMAND_WREN_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_RDEN_OFFSET 5
#define CSR_SDRAM_DFII_PI0_COMMAND_RDEN_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_CS_TOP_OFFSET 6
#define CSR_SDRAM_DFII_PI0_COMMAND_CS_TOP_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_CS_BOTTOM_OFFSET 7
#define CSR_SDRAM_DFII_PI0_COMMAND_CS_BOTTOM_SIZE 1
#define CSR_SDRAM_DFII_PI0_COMMAND_ISSUE_ADDR (CSR_BASE + 0x3008L)
#define CSR_SDRAM_DFII_PI0_COMMAND_ISSUE_SIZE 1
#define CSR_SDRAM_DFII_PI0_ADDRESS_ADDR (CSR_BASE + 0x300cL)
#define CSR_SDRAM_DFII_PI0_ADDRESS_SIZE 1
#define CSR_SDRAM_DFII_PI0_BADDRESS_ADDR (CSR_BASE + 0x3010L)
#define CSR_SDRAM_DFII_PI0_BADDRESS_SIZE 1
#define CSR_SDRAM_DFII_PI0_WRDATA_ADDR (CSR_BASE + 0x3014L)
#define CSR_SDRAM_DFII_PI0_WRDATA_SIZE 2
#define CSR_SDRAM_DFII_PI0_RDDATA_ADDR (CSR_BASE + 0x301cL)
#define CSR_SDRAM_DFII_PI0_RDDATA_SIZE 2
#define CSR_SDRAM_DFII_PI1_COMMAND_ADDR (CSR_BASE + 0x3024L)
#define CSR_SDRAM_DFII_PI1_COMMAND_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_CS_OFFSET 0
#define CSR_SDRAM_DFII_PI1_COMMAND_CS_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_WE_OFFSET 1
#define CSR_SDRAM_DFII_PI1_COMMAND_WE_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_CAS_OFFSET 2
#define CSR_SDRAM_DFII_PI1_COMMAND_CAS_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_RAS_OFFSET 3
#define CSR_SDRAM_DFII_PI1_COMMAND_RAS_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_WREN_OFFSET 4
#define CSR_SDRAM_DFII_PI1_COMMAND_WREN_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_RDEN_OFFSET 5
#define CSR_SDRAM_DFII_PI1_COMMAND_RDEN_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_CS_TOP_OFFSET 6
#define CSR_SDRAM_DFII_PI1_COMMAND_CS_TOP_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_CS_BOTTOM_OFFSET 7
#define CSR_SDRAM_DFII_PI1_COMMAND_CS_BOTTOM_SIZE 1
#define CSR_SDRAM_DFII_PI1_COMMAND_ISSUE_ADDR (CSR_BASE + 0x3028L)
#define CSR_SDRAM_DFII_PI1_COMMAND_ISSUE_SIZE 1
#define CSR_SDRAM_DFII_PI1_ADDRESS_ADDR (CSR_BASE + 0x302cL)
#define CSR_SDRAM_DFII_PI1_ADDRESS_SIZE 1
#define CSR_SDRAM_DFII_PI1_BADDRESS_ADDR (CSR_BASE + 0x3030L)
#define CSR_SDRAM_DFII_PI1_BADDRESS_SIZE 1
#define CSR_SDRAM_DFII_PI1_WRDATA_ADDR (CSR_BASE + 0x3034L)
#define CSR_SDRAM_DFII_PI1_WRDATA_SIZE 2
#define CSR_SDRAM_DFII_PI1_RDDATA_ADDR (CSR_BASE + 0x303cL)
#define CSR_SDRAM_DFII_PI1_RDDATA_SIZE 2
#define CSR_TIMER0_BASE (CSR_BASE + 0x3800L)
#define CSR_TIMER0_LOAD_ADDR (CSR_BASE + 0x3800L)
#define CSR_TIMER0_LOAD_SIZE 1
#define CSR_TIMER0_RELOAD_ADDR (CSR_BASE + 0x3804L)
#define CSR_TIMER0_RELOAD_SIZE 1
#define CSR_TIMER0_EN_ADDR (CSR_BASE + 0x3808L)
#define CSR_TIMER0_EN_SIZE 1
#define CSR_TIMER0_UPDATE_VALUE_ADDR (CSR_BASE + 0x380cL)
#define CSR_TIMER0_UPDATE_VALUE_SIZE 1
#define CSR_TIMER0_VALUE_ADDR (CSR_BASE + 0x3810L)
#define CSR_TIMER0_VALUE_SIZE 1
#define CSR_TIMER0_EV_STATUS_ADDR (CSR_BASE + 0x3814L)
#define CSR_TIMER0_EV_STATUS_SIZE 1
#define CSR_TIMER0_EV_STATUS_ZERO_OFFSET 0
#define CSR_TIMER0_EV_STATUS_ZERO_SIZE 1
#define CSR_TIMER0_EV_PENDING_ADDR (CSR_BASE + 0x3818L)
#define CSR_TIMER0_EV_PENDING_SIZE 1
#define CSR_TIMER0_EV_PENDING_ZERO_OFFSET 0
#define CSR_TIMER0_EV_PENDING_ZERO_SIZE 1
#define CSR_TIMER0_EV_ENABLE_ADDR (CSR_BASE + 0x381cL)
#define CSR_TIMER0_EV_ENABLE_SIZE 1
#define CSR_TIMER0_EV_ENABLE_ZERO_OFFSET 0
#define CSR_TIMER0_EV_ENABLE_ZERO_SIZE 1

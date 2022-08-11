
/*
 * BST SoC pinctrl driver
 *
 * Copyright (C) 2014 Bst, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

//#include <linux/gpio.h>
//#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "pinctrl-bst.h"
//#include "pinctrl-utils.h"

static const struct pinctrl_pin_desc bst_iomux_pins[] = {
	PINCTRL_PIN(15, "gpio_15"),
	PINCTRL_PIN(24, "gpio_24"),
	PINCTRL_PIN(25, "gpio_25"),
	PINCTRL_PIN(26, "gpio_26"),
	PINCTRL_PIN(27, "gpio_27"),
	PINCTRL_PIN(28, "gpio_28"),
	PINCTRL_PIN(29, "gpio_29"),
	PINCTRL_PIN(30, "gpio_30"),
	PINCTRL_PIN(31, "gpio_31"),
	PINCTRL_PIN(32, "jtag_tck"),
	PINCTRL_PIN(33, "jtag_trst"),
	PINCTRL_PIN(34, "jtag_tms"),
	PINCTRL_PIN(35, "jtag_tdi"),
	PINCTRL_PIN(36, "jtag_tdo"),
	PINCTRL_PIN(37, "uart0_txd"),
	PINCTRL_PIN(38, "uart0_rxd"),
	PINCTRL_PIN(39, "uart0_cts"),
	PINCTRL_PIN(40, "uart0_rts"),
	PINCTRL_PIN(41, "uart1_txd"),
	PINCTRL_PIN(42, "uart1_rxd"),
	PINCTRL_PIN(43, "uart1_cts"),
	PINCTRL_PIN(44, "uart1_rts"),
	PINCTRL_PIN(45, "uart2_txd"),
	PINCTRL_PIN(46, "uart2_rxd"),
	PINCTRL_PIN(47, "uart2_cts"),
	PINCTRL_PIN(48, "uart2_rts"),
	PINCTRL_PIN(49, "uart3_txd"),
	PINCTRL_PIN(50, "uart3_rxd"),
	PINCTRL_PIN(51, "uart3_cts"),
	PINCTRL_PIN(52, "uart3_rts"),
	PINCTRL_PIN(53, "i2s1_ck"),
	PINCTRL_PIN(54, "i2s1_ws"),
	PINCTRL_PIN(55, "i2s1_sd_in"),
	PINCTRL_PIN(56, "pwm0"),
	PINCTRL_PIN(57, "pwm1"),
	PINCTRL_PIN(58, "pwm2"),
	PINCTRL_PIN(59, "pwm3"),
	PINCTRL_PIN(60, "qspi0_sclk"),
	PINCTRL_PIN(61, "qspi0_cs0"),
	PINCTRL_PIN(62, "qspi0_cs1"),
	PINCTRL_PIN(63, "qspi0_io0"),
	PINCTRL_PIN(64, "qspi0_io1"),
	PINCTRL_PIN(65, "qspi0_io2"),
	PINCTRL_PIN(66, "qspi0_io3"),
	PINCTRL_PIN(67, "qspi0_io4"),
	PINCTRL_PIN(68, "qspi0_io5"),
	PINCTRL_PIN(69, "qspi0_io6"),
	PINCTRL_PIN(70, "qspi0_io7"),
	PINCTRL_PIN(71, "qspi1_sclk"),
	PINCTRL_PIN(72, "qspi1_cs0"),
	PINCTRL_PIN(73, "qspi1_cs1"),
	PINCTRL_PIN(74, "qspi1_io0"),
	PINCTRL_PIN(75, "qspi1_io1"),
	PINCTRL_PIN(76, "qspi1_io2"),
	PINCTRL_PIN(77, "qspi1_io3"),
	PINCTRL_PIN(78, "qspi1_io4"),
	PINCTRL_PIN(79, "qspi1_io5"),
	PINCTRL_PIN(80, "qspi1_io6"),
	PINCTRL_PIN(81, "qspi1_io7"),
	PINCTRL_PIN(82, "can_tx0"),
	PINCTRL_PIN(83, "can_rx0"),
	PINCTRL_PIN(84, "can_tx1"),
	PINCTRL_PIN(85, "can_rx1"),
	PINCTRL_PIN(86, "gpio_107"),
	PINCTRL_PIN(87, "gpio_108"),
	PINCTRL_PIN(88, "sdemmc0_clk"),
	PINCTRL_PIN(89, "sdemmc0_cmd"),
	PINCTRL_PIN(90, "sdemmc0_dat0"),
	PINCTRL_PIN(91, "sdemmc0_dat1"),
	PINCTRL_PIN(92, "sdemmc0_dat2"),
	PINCTRL_PIN(93, "sdemmc0_dat3"),
	PINCTRL_PIN(94, "sdemmc0_dat4"),
	PINCTRL_PIN(95, "sdemmc0_dat5"),
	PINCTRL_PIN(96, "sdemmc0_dat6"),
	PINCTRL_PIN(97, "sdemmc0_dat7"),
	PINCTRL_PIN(98, "sdemmc0_rstb"),
	PINCTRL_PIN(99, "sdemmc0_cdn"),
	PINCTRL_PIN(100, "sdemmc0_wp"),
	PINCTRL_PIN(101, "rptl0_p"),
	PINCTRL_PIN(102, "rptl0_n"),
	PINCTRL_PIN(103, "sdemmc1_clk"),
	PINCTRL_PIN(104, "sdemmc1_cmd"),
	PINCTRL_PIN(105, "sdemmc1_dat0"),
	PINCTRL_PIN(106, "sdemmc1_dat1"),
	PINCTRL_PIN(107, "sdemmc1_dat2"),
	PINCTRL_PIN(108, "sdemmc1_dat3"),
	PINCTRL_PIN(109, "sdemmc1_dat4"),
	PINCTRL_PIN(110, "sdemmc1_dat5"),
	PINCTRL_PIN(111, "sdemmc1_dat6"),
	PINCTRL_PIN(112, "sdemmc1_dat7"),
	PINCTRL_PIN(113, "sdemmc1_rstb"),
	PINCTRL_PIN(114, "sdemmc1_cdn"),
	PINCTRL_PIN(115, "sdemmc1_wp"),
	PINCTRL_PIN(116, "rptl1_p"),
	PINCTRL_PIN(117, "rptl1_n"),
	PINCTRL_PIN(118, "rgmii0_txd0"),
	PINCTRL_PIN(119, "rgmii0_txd1"),
	PINCTRL_PIN(120, "rgmii0_txd2"),
	PINCTRL_PIN(121, "rgmii0_txd3"),
	PINCTRL_PIN(122, "gmii0_txd4"),
	PINCTRL_PIN(123, "gmii0_txd5"),
	PINCTRL_PIN(124, "gmii0_txd6"),
	PINCTRL_PIN(125, "gmii0_txd7"),
	PINCTRL_PIN(126, "gmii0_txer"),
	PINCTRL_PIN(127, "sdemmc0_pvdd18pocsd0"),
	PINCTRL_PIN(128, "sdemmc0_pvdd18pocsd1"),
	PINCTRL_PIN(129, "sdemmc0_pvdd18pocsd2"),
	PINCTRL_PIN(130, "sdemmc1_pvdd18pocsd0"),
	PINCTRL_PIN(131, "sdemmc1_pvdd18pocsd1"),
	PINCTRL_PIN(132, "sdemmc1_pvdd18pocsd2"),
	PINCTRL_PIN(133, "pcie0_rstb"),
	PINCTRL_PIN(134, "pcie1_rstb"),
	PINCTRL_PIN(135, "isp_fsync0"),
	PINCTRL_PIN(136, "isp_fsync1"),
	PINCTRL_PIN(137, "isp_fsync2"),
	PINCTRL_PIN(138, "isp_fsync3"),
	PINCTRL_PIN(139, "isp_fsync4"),
	PINCTRL_PIN(140, "isp_fsync5"),
	PINCTRL_PIN(141, "isp_fsync6"),
	PINCTRL_PIN(142, "isp_fsync7"),
	PINCTRL_PIN(143, "sdemmc0_led_ctl"),
	PINCTRL_PIN(144, "sdemmc1_led_ctl"),
	PINCTRL_PIN(145, "debug0"),
	PINCTRL_PIN(146, "debug1"),
	PINCTRL_PIN(147, "debug2"),
	PINCTRL_PIN(148, "debug3"),
	PINCTRL_PIN(149, "debug4"),
	PINCTRL_PIN(150, "debug5"),
	PINCTRL_PIN(151, "debug6"),
	PINCTRL_PIN(152, "debug7"),
	PINCTRL_PIN(153, "dsp_jtag_tck"),	
	PINCTRL_PIN(154, "dsp_jtag_trst"),	
	PINCTRL_PIN(155, "dsp_jtag_tms"),	
	PINCTRL_PIN(156, "dsp_jtag_tdi"),	
	PINCTRL_PIN(157, "dsp_jtag_tdo"),	
	PINCTRL_PIN(158, "vin_r0"),	
	PINCTRL_PIN(159, "vin_r1"),	
	PINCTRL_PIN(160, "vin_r2"),	
	PINCTRL_PIN(161, "vin_r3"),	
	PINCTRL_PIN(162, "vin_r4"),	
	PINCTRL_PIN(163, "vin_g0"),	
	PINCTRL_PIN(164, "vin_g1"),	
	PINCTRL_PIN(165, "vin_g2"),	
	PINCTRL_PIN(166, "vin_g3"),	
	PINCTRL_PIN(167, "vin_g4"),	
	PINCTRL_PIN(168, "vin_g5"),	
	PINCTRL_PIN(169, "vin_b0"),	
	PINCTRL_PIN(170, "vin_b1"),	
	PINCTRL_PIN(171, "vin_b2"),	
	PINCTRL_PIN(172, "vin_b3"),	
	PINCTRL_PIN(173, "vin_b4"),	
	PINCTRL_PIN(174, "vin_hs"),	
	PINCTRL_PIN(175, "vin_vs"),	
	PINCTRL_PIN(176, "vin_de"),	
	PINCTRL_PIN(177, "vin_llc"),
	PINCTRL_PIN(178, "rgmii0_txctrl"),
	PINCTRL_PIN(179, "mii0_txclk"),	       
	PINCTRL_PIN(180, "rgmii0_gtxclk"),    
	PINCTRL_PIN(181, "rgmii0_rxd0"),    
	PINCTRL_PIN(182, "rgmii0_rxd1"),    
	PINCTRL_PIN(183, "rgmii0_rxd2"), 
	PINCTRL_PIN(184, "rgmii0_rxd3"),	  
	PINCTRL_PIN(185, "gmii0_rxd4"),	       
	PINCTRL_PIN(186, "gmii0_rxd5"),	 
	PINCTRL_PIN(187, "gmii0_rxd6"),	       
	PINCTRL_PIN(188, "gmii0_rxd7"),	       
	PINCTRL_PIN(189, "gmii0_rxer"),	      
	PINCTRL_PIN(190, "rgmii0_rxctrl"),
	PINCTRL_PIN(191, "rgmii0_rxclk"),
	PINCTRL_PIN(192, "rgmii0_mdc"),	 
	PINCTRL_PIN(193, "rgmii0_mdio"),	 
	PINCTRL_PIN(194, "rgmii1_txd0"),	 
	PINCTRL_PIN(195, "rgmii1_txd1"),	 
	PINCTRL_PIN(196, "rgmii1_txd2"),	 
	PINCTRL_PIN(197, "rgmii1_txd3"),	 
	PINCTRL_PIN(198, "mii1_txer"),	 
	PINCTRL_PIN(199, "rgmii1_txctrl"),
	PINCTRL_PIN(200, "mii1_txclk"),	 
	PINCTRL_PIN(201, "rgmii1_gtxclk"),
	PINCTRL_PIN(202, "rgmii1_rxd0"),
	PINCTRL_PIN(203, "rgmii1_rxd1"),
	PINCTRL_PIN(204, "rgmii1_rxd2"),
	PINCTRL_PIN(205, "rgmii1_rxd3"),
	PINCTRL_PIN(206, "mii1_rxer"),
	PINCTRL_PIN(207, "rgmii1_rxctrl"),
	PINCTRL_PIN(208, "rgmii1_rxclk"),
	PINCTRL_PIN(209, "rgmii1_mdc"),	 
	PINCTRL_PIN(210, "rgmii1_mdio"),	 
	PINCTRL_PIN(211, "ptp_clk"),
	PINCTRL_PIN(212, "rgmii0_intr"),
	PINCTRL_PIN(213, "rgmii1_intr"),
	PINCTRL_PIN(214, "vout_r0"),
	PINCTRL_PIN(215, "vout_r1"),
	PINCTRL_PIN(216, "vout_r2"),
	PINCTRL_PIN(217, "vout_r3"),
	PINCTRL_PIN(218, "vout_r4"),
	PINCTRL_PIN(219, "vout_r5"),
	PINCTRL_PIN(220, "vout_r6"),
	PINCTRL_PIN(221, "vout_r7"),
	PINCTRL_PIN(222, "vout_g0"),
	PINCTRL_PIN(223, "vout_g1"),
	PINCTRL_PIN(224, "vout_g2"),
	PINCTRL_PIN(225, "vout_g3"),
	PINCTRL_PIN(226, "vout_g4"),
	PINCTRL_PIN(227, "vout_g5"),
	PINCTRL_PIN(228, "vout_g6"),
	PINCTRL_PIN(229, "vout_g7"),
	PINCTRL_PIN(230, "vout_b0"),
	PINCTRL_PIN(231, "vout_b1"),
	PINCTRL_PIN(232, "vout_b2"),
	PINCTRL_PIN(233, "vout_b3"),
	PINCTRL_PIN(234, "vout_b4"),
	PINCTRL_PIN(235, "vout_b5"),
	PINCTRL_PIN(236, "vout_b6"),
	PINCTRL_PIN(237, "vout_b7"),
	PINCTRL_PIN(238, "vout_hs"),
	PINCTRL_PIN(239, "vout_vs"),
	PINCTRL_PIN(240, "vout_de"),
	PINCTRL_PIN(241, "vout_pclk"),
	PINCTRL_PIN(242, "vout_pdb"),
	PINCTRL_PIN(243, "ts_trig_in00"),
	PINCTRL_PIN(244, "ts_trig_in01"),
	PINCTRL_PIN(245, "ts_trig_in10"),
	PINCTRL_PIN(246, "ts_trig_in11"),
	PINCTRL_PIN(247, "ptp_pps0"),	
	PINCTRL_PIN(248, "ptp_pps1"),
	PINCTRL_PIN(249, "i2c0_scl"),
	PINCTRL_PIN(250, "i2c0_sda"),
	PINCTRL_PIN(251, "i2c1_scl"),
	PINCTRL_PIN(252, "i2c1_sda"),
	PINCTRL_PIN(253, "i2c2_scl"),
	PINCTRL_PIN(254, "i2c2_sda"),
	PINCTRL_PIN(255, "i2c3_scl"),
	PINCTRL_PIN(256, "i2c3_sda"),
	PINCTRL_PIN(257, "i2c4_scl"),
	PINCTRL_PIN(258, "i2c4_sda"),
	PINCTRL_PIN(259, "i2c5_scl"),
	PINCTRL_PIN(260, "i2c5_sda"),
	PINCTRL_PIN(261, "spi0_sclk"),
	PINCTRL_PIN(262, "spi0_cs"),
	PINCTRL_PIN(263, "spi0_mosi"),
	PINCTRL_PIN(264, "spi0_miso"),
	PINCTRL_PIN(265, "spi1_sclk"),
	PINCTRL_PIN(266, "spi1_cs"),
	PINCTRL_PIN(267, "spi1_mosi"),
	PINCTRL_PIN(268, "spi1_miso"),
	PINCTRL_PIN(269, "i2s0_mck"),
	PINCTRL_PIN(270, "i2s0_ck"),
	PINCTRL_PIN(271, "i2s0_sd_out"),
	PINCTRL_PIN(272, "i2s0_ws"),
};                    

#define DECLARE_BST_PINS(name, pin) static const unsigned int name##_pins[] = { pin }
DECLARE_BST_PINS(gpio_15, 15);
DECLARE_BST_PINS(gpio_24, 24);
DECLARE_BST_PINS(gpio_25, 25);
DECLARE_BST_PINS(gpio_26, 26);
DECLARE_BST_PINS(gpio_27, 27);
DECLARE_BST_PINS(gpio_28, 28);
DECLARE_BST_PINS(gpio_29, 29 );
DECLARE_BST_PINS(gpio_30, 30);
DECLARE_BST_PINS(gpio_31, 31);
DECLARE_BST_PINS(jtag_tck, 32);
DECLARE_BST_PINS(jtag_trst, 33);
DECLARE_BST_PINS(jtag_tms, 34);
DECLARE_BST_PINS(jtag_tdi, 35);
DECLARE_BST_PINS(jtag_tdo, 36);
DECLARE_BST_PINS(uart0_txd, 37);
DECLARE_BST_PINS(uart0_rxd, 38);
DECLARE_BST_PINS(uart0_cts, 39);
DECLARE_BST_PINS(uart0_rts, 40);
DECLARE_BST_PINS(uart1_txd, 41);
DECLARE_BST_PINS(uart1_rxd, 42);
DECLARE_BST_PINS(uart1_cts, 43);
DECLARE_BST_PINS(uart1_rts, 44);
DECLARE_BST_PINS(uart2_txd, 45);
DECLARE_BST_PINS(uart2_rxd, 46);
DECLARE_BST_PINS(uart2_cts, 47);
DECLARE_BST_PINS(uart2_rts, 48);
DECLARE_BST_PINS(uart3_txd, 49);
DECLARE_BST_PINS(uart3_rxd, 50);
DECLARE_BST_PINS(uart3_cts, 51);
DECLARE_BST_PINS(uart3_rts, 52);
DECLARE_BST_PINS(i2s1_ck, 53);
DECLARE_BST_PINS(i2s1_sd_in, 54);
DECLARE_BST_PINS(i2s1_ws, 55);
DECLARE_BST_PINS(pwm0, 56);
DECLARE_BST_PINS(pwm1, 57);
DECLARE_BST_PINS(pwm2, 58);
DECLARE_BST_PINS(pwm3, 59);
DECLARE_BST_PINS(qspi0_sclk, 60);
DECLARE_BST_PINS(qspi0_cs0, 61);
DECLARE_BST_PINS(qspi0_cs1, 62);
DECLARE_BST_PINS(qspi0_io0, 63);
DECLARE_BST_PINS(qspi0_io1, 64);
DECLARE_BST_PINS(qspi0_io2, 65);
DECLARE_BST_PINS(qspi0_io3, 66);
DECLARE_BST_PINS(qspi0_io4, 67);
DECLARE_BST_PINS(qspi0_io5, 68);
DECLARE_BST_PINS(qspi0_io6, 69);
DECLARE_BST_PINS(qspi0_io7, 70);
DECLARE_BST_PINS(qspi1_sclk, 71);
DECLARE_BST_PINS(qspi1_cs0, 72);
DECLARE_BST_PINS(qspi1_cs1, 73);
DECLARE_BST_PINS(qspi1_io0, 74);
DECLARE_BST_PINS(qspi1_io1, 75);
DECLARE_BST_PINS(qspi1_io2, 76);
DECLARE_BST_PINS(qspi1_io3, 77);
DECLARE_BST_PINS(qspi1_io4, 78);
DECLARE_BST_PINS(qspi1_io5, 79);
DECLARE_BST_PINS(qspi1_io6, 80);
DECLARE_BST_PINS(qspi1_io7, 81);
DECLARE_BST_PINS(can_tx0, 82);
DECLARE_BST_PINS(can_rx0, 83);
DECLARE_BST_PINS(can_tx1, 84);
DECLARE_BST_PINS(can_rx1, 85);
DECLARE_BST_PINS(gpio_107, 86);
DECLARE_BST_PINS(gpio_108,87);
DECLARE_BST_PINS(sdemmc0_clk	,88);
DECLARE_BST_PINS(sdemmc0_cmd	,89);
DECLARE_BST_PINS(sdemmc0_dat0,90);
DECLARE_BST_PINS(sdemmc0_dat1,91);
DECLARE_BST_PINS(sdemmc0_dat2,92);
DECLARE_BST_PINS(sdemmc0_dat3,93);
DECLARE_BST_PINS(sdemmc0_dat4,94);
DECLARE_BST_PINS(sdemmc0_dat5,95);
DECLARE_BST_PINS(sdemmc0_dat6,96);
DECLARE_BST_PINS(sdemmc0_dat7,97);
DECLARE_BST_PINS(sdemmc0_rstb,98);
DECLARE_BST_PINS(sdemmc0_cdn	,99);
DECLARE_BST_PINS(sdemmc0_wp	,100);
DECLARE_BST_PINS(err_rpt_l0_p	,101);
DECLARE_BST_PINS(err_rpt_l0_n	,102);
DECLARE_BST_PINS(sdemmc1_clk	,103);
DECLARE_BST_PINS(sdemmc1_cmd	,104);
DECLARE_BST_PINS(sdemmc1_dat0,105);
DECLARE_BST_PINS(sdemmc1_dat1,106);
DECLARE_BST_PINS(sdemmc1_dat2,107);
DECLARE_BST_PINS(sdemmc1_dat3,108);
DECLARE_BST_PINS(sdemmc1_dat4,109);
DECLARE_BST_PINS(sdemmc1_dat5,110);
DECLARE_BST_PINS(sdemmc1_dat6,111);
DECLARE_BST_PINS(sdemmc1_dat7,112);
DECLARE_BST_PINS(sdemmc1_rstb,113);
DECLARE_BST_PINS(sdemmc1_cdn	,114);
DECLARE_BST_PINS(sdemmc1_wp	,115);
DECLARE_BST_PINS(err_rpt_l1_p	,116);
DECLARE_BST_PINS(err_rpt_l1_n	,117);
DECLARE_BST_PINS(rgmii0_txd0	,118);
DECLARE_BST_PINS(rgmii0_txd1	,119);
DECLARE_BST_PINS(rgmii0_txd2	,120);
DECLARE_BST_PINS(rgmii0_txd3	,121);
DECLARE_BST_PINS(gmii0_txd4	,122);
DECLARE_BST_PINS(gmii0_txd5	,123);
DECLARE_BST_PINS(gmii0_txd6	,124);
DECLARE_BST_PINS(gmii0_txd7	,125);
DECLARE_BST_PINS(gmii0_txer	,126);
DECLARE_BST_PINS(sdemmc0_pvdd18pocsd0, 127);
DECLARE_BST_PINS(sdemmc0_pvdd18pocsd1, 128);
DECLARE_BST_PINS(sdemmc0_pvdd18pocsd2, 129);
DECLARE_BST_PINS(sdemmc1_pvdd18pocsd0, 130);
DECLARE_BST_PINS(sdemmc1_pvdd18pocsd1, 131);
DECLARE_BST_PINS(sdemmc1_pvdd18pocsd2, 132);
DECLARE_BST_PINS(pcie0_rstb	,133);
DECLARE_BST_PINS(pcie1_rstb	,134);
DECLARE_BST_PINS(isp_fsync0, 135);
DECLARE_BST_PINS(isp_fsync1, 136);
DECLARE_BST_PINS(isp_fsync2, 137);
DECLARE_BST_PINS(isp_fsync3, 138);
DECLARE_BST_PINS(isp_fsync4, 139);
DECLARE_BST_PINS(isp_fsync5, 140);
DECLARE_BST_PINS(isp_fsync6	,141);
DECLARE_BST_PINS(isp_fsync7	,142);
DECLARE_BST_PINS(sdemmc0_led_ctl,143);
DECLARE_BST_PINS(sdemmc1_led_ctl,144);
DECLARE_BST_PINS(debug0, 145);
DECLARE_BST_PINS(debug1, 146);
DECLARE_BST_PINS(debug2, 147);
DECLARE_BST_PINS(debug3, 148);
DECLARE_BST_PINS(debug4, 149);
DECLARE_BST_PINS(debug5, 150);
DECLARE_BST_PINS(debug6	,151);
DECLARE_BST_PINS(debug7	,152);
DECLARE_BST_PINS(dsp_jtag_tck,153);	
DECLARE_BST_PINS(dsp_jtag_trst,154);	
DECLARE_BST_PINS(dsp_jtag_tms,155);	
DECLARE_BST_PINS(dsp_jtag_tdi,156);	
DECLARE_BST_PINS(dsp_jtag_tdo,157);	
DECLARE_BST_PINS(vin_r0, 158);	
DECLARE_BST_PINS(vin_r1, 159);	
DECLARE_BST_PINS(vin_r2, 160);	
DECLARE_BST_PINS(vin_r3, 161);	
DECLARE_BST_PINS(vin_r4, 162);	
DECLARE_BST_PINS(vin_g0, 163);	
DECLARE_BST_PINS(vin_g1, 164);	
DECLARE_BST_PINS(vin_g2, 165);	
DECLARE_BST_PINS(vin_g3, 166);	
DECLARE_BST_PINS(vin_g4, 167);	
DECLARE_BST_PINS(vin_g5, 168);	
DECLARE_BST_PINS(vin_b0, 169);	
DECLARE_BST_PINS(vin_b1, 170);	
DECLARE_BST_PINS(vin_b2, 171);	
DECLARE_BST_PINS(vin_b3, 172);	
DECLARE_BST_PINS(vin_b4, 173);	
DECLARE_BST_PINS(vin_hs, 174);	
DECLARE_BST_PINS(vin_vs, 175);	
DECLARE_BST_PINS(vin_de, 176);	
DECLARE_BST_PINS(vin_llc, 177);
DECLARE_BST_PINS(rgmii0_txctrl	,178);
DECLARE_BST_PINS(mii0_txclk,179);	          
DECLARE_BST_PINS(rgmii0_gtxclk, 180);    
DECLARE_BST_PINS(rgmii0_rxd0,181);    
DECLARE_BST_PINS(rgmii0_rxd1,182);    
DECLARE_BST_PINS(rgmii0_rxd2,183); 
DECLARE_BST_PINS(rgmii0_rxd3,184);	  
DECLARE_BST_PINS(gmii0_rxd4,185);	         
DECLARE_BST_PINS(gmii0_rxd5,186);	 
DECLARE_BST_PINS(gmii0_rxd6,187);	        
DECLARE_BST_PINS(gmii0_rxd7,188);	        
DECLARE_BST_PINS(gmii0_rxer,189);	      
DECLARE_BST_PINS(rgmii0_rxctrl,190);
DECLARE_BST_PINS(rgmii0_rxclk,191);
DECLARE_BST_PINS(rgmii0_mdc,192);	 
DECLARE_BST_PINS(rgmii0_mdio,193);	 
DECLARE_BST_PINS(rgmii1_txd0,194);	 
DECLARE_BST_PINS(rgmii1_txd1,195);	 
DECLARE_BST_PINS(rgmii1_txd2,196);	 
DECLARE_BST_PINS(rgmii1_txd3,197);	 
DECLARE_BST_PINS(mii1_txer,198);	 
DECLARE_BST_PINS(rgmii1_txctrl,199);
DECLARE_BST_PINS(mii1_txclk,200);	 
DECLARE_BST_PINS(rgmii1_gtxclk,201);
DECLARE_BST_PINS(rgmii1_rxd0,202);
DECLARE_BST_PINS(rgmii1_rxd1,203);
DECLARE_BST_PINS(rgmii1_rxd2,204);
DECLARE_BST_PINS(rgmii1_rxd3,205);
DECLARE_BST_PINS(mii1_rxer,206);
DECLARE_BST_PINS(rgmii1_rxctrl,207);
DECLARE_BST_PINS(rgmii1_rxclk,208);
DECLARE_BST_PINS(rgmii1_mdc,209);	 
DECLARE_BST_PINS(rgmii1_mdio,210);	 
DECLARE_BST_PINS(ptp_clk, 211);
DECLARE_BST_PINS(rgmii0_intr,212);
DECLARE_BST_PINS(rgmii1_intr,213);
DECLARE_BST_PINS(vout_r0,214);
DECLARE_BST_PINS(vout_r1,215);
DECLARE_BST_PINS(vout_r2,216);
DECLARE_BST_PINS(vout_r3,217);
DECLARE_BST_PINS(vout_r4,218);
DECLARE_BST_PINS(vout_r5,219);
DECLARE_BST_PINS(vout_r6,220);
DECLARE_BST_PINS(vout_r7,221);
DECLARE_BST_PINS(vout_g0,222);
DECLARE_BST_PINS(vout_g1,223);
DECLARE_BST_PINS(vout_g2,224);
DECLARE_BST_PINS(vout_g3,225);
DECLARE_BST_PINS(vout_g4,226);
DECLARE_BST_PINS(vout_g5,227);
DECLARE_BST_PINS(vout_g6,228);
DECLARE_BST_PINS(vout_g7,229);
DECLARE_BST_PINS(vout_b0,230);
DECLARE_BST_PINS(vout_b1,231);
DECLARE_BST_PINS(vout_b2,232);
DECLARE_BST_PINS(vout_b3,233);
DECLARE_BST_PINS(vout_b4,234);
DECLARE_BST_PINS(vout_b5,235);
DECLARE_BST_PINS(vout_b6,236);
DECLARE_BST_PINS(vout_b7,237);
DECLARE_BST_PINS(vout_hs,238);
DECLARE_BST_PINS(vout_vs,239);
DECLARE_BST_PINS(vout_de,240);
DECLARE_BST_PINS(vout_pclk,241);
DECLARE_BST_PINS(vout_pdb,242);
DECLARE_BST_PINS(ts_trig_in00,243);
DECLARE_BST_PINS(ts_trig_in01,244);
DECLARE_BST_PINS(ts_trig_in10,245);
DECLARE_BST_PINS(ts_trig_in11,246);
DECLARE_BST_PINS(ptp_pps0,247);	
DECLARE_BST_PINS(ptp_pps1,248);
DECLARE_BST_PINS(i2c0_scl, 249);
DECLARE_BST_PINS(i2c0_sda, 250);
DECLARE_BST_PINS(i2c1_scl, 251);
DECLARE_BST_PINS(i2c1_sda, 252);
DECLARE_BST_PINS(i2c2_scl, 253);
DECLARE_BST_PINS(i2c2_sda, 254);
DECLARE_BST_PINS(i2c3_scl, 255);
DECLARE_BST_PINS(i2c3_sda, 256);
DECLARE_BST_PINS(i2c4_scl, 257);
DECLARE_BST_PINS(i2c4_sda, 258);
DECLARE_BST_PINS(i2c5_scl, 259);
DECLARE_BST_PINS(i2c5_sda, 260);
DECLARE_BST_PINS(spi0_sclk, 261);
DECLARE_BST_PINS(spi0_cs, 262);
DECLARE_BST_PINS(spi0_mosi, 263);
DECLARE_BST_PINS(spi0_miso, 264);
DECLARE_BST_PINS(spi1_sclk, 265);
DECLARE_BST_PINS(spi1_cs, 266);
DECLARE_BST_PINS(spi1_mosi, 267);
DECLARE_BST_PINS(spi1_miso, 268);
DECLARE_BST_PINS(i2s0_mck, 269);
DECLARE_BST_PINS(i2s0_ck, 270);
DECLARE_BST_PINS(i2s0_sd_out, 271);
DECLARE_BST_PINS(i2s0_ws, 272);
	
#define FUNCTION(fname)					\
	[BST_FUNC_##fname] = {				\
		.name = #fname,				\
		.groups = fname##_groups,		\
		.ngroups = ARRAY_SIZE(fname##_groups),	\
	}

static const char * const gpio_groups[] = {
	"debug0", "debug1", "debug2", "debug3", "debug4", "debug5", "debug6", "debug7",
	"dsp_jtag_tck", "dsp_jtag_trst", "dsp_jtag_tms", "dsp_jtag_tdi", "dsp_jtag_tdo", 
    "sdemmc0_led_ctl", "sdemmc1_led_ctl", "gpio_15", "isp_fsync0", "isp_fsync1", "isp_fsync2", 
    "isp_fsync3", "isp_fsync4", "isp_fsync5","isp_fsync6", "isp_fsync7", "gpio_29","gpio_30","gpio_31",
    "jtag_tck", "jtag_trst", "jtag_tms", "jtag_tdi", "jtag_tdo", "uart0_txd", "uart0_rxd", "uart0_cts", 
    "uart0_rts", "uart1_txd", "uart1_rxd", "uart1_cts", "uart1_rts", "uart2_txd", "uart2_rxd", "uart2_cts", "uart2_rts",
    "uart3_txd", "uart3_rxd", "uart3_cts", "uart3_rts", "i2c0_scl", "i2c0_sda", "i2c1_scl", "i2c1_sda", 
    "i2c2_scl", "i2c2_sda", "i2c3_scl", "i2c3_sda", "i2c4_scl", "i2c4_sda", "i2c5_scl", "i2c5_sda",
    "spi0_cs", "spi0_miso", "spi1_cs", "spi1_miso", "i2s0_sd_out", "i2s0_ws", "i2s1_ck", "i2s1_sd_in", 
    "i2s1_ws", "pwm0", "pwm1", "pwm2", "pwm3", "qspi0_cs0", "qspi0_cs1", "qspi0_io0", "qspi0_io1", "qspi0_io2", 
    "qspi0_io3", "qspi0_io4", "qspi0_io5", "qspi0_io6", "qspi0_io7", "qspi0_sclk", "qspi1_cs0", "qspi1_cs1", "qspi1_io0", 
    "qspi1_io1", "qspi1_io2", "qspi1_io3", "qspi1_io4", "qspi1_io5", "qspi1_io6", "qspi1_io7", "qspi1_sclk", 
    "can_rx0", "can_rx1", "can_tx0", "can_tx1", "gpio_107","gpio_108", "err_rpt_l0_n", "err_rpt_l0_p", 
    "err_rpt_l1_n", "err_rpt_l1_p", "rgmii0_intr", "rgmii0_mdc", "rgmii0_mdio", "rgmii1_intr", "rgmii1_mdc", "rgmii1_mdio", 
    "ts_trig_in00", "ts_trig_in01", "ts_trig_in10", "ts_trig_in11", "ptp_pps0", "ptp_pps1", "pcie0_rstb", "pcie1_rstb", "gpio_24",
    "gpio_25", "gpio_26", "gpio_27", "gpio_28"};

static const char *const bist_groups[] = {"spi1_mosi"};
static const char *const boot_groups[] = {"gpio_31"};
static const char *const can0_groups[] = {"can_rx0", "can_tx0"};
static const char *const can1_groups[] = {"can_rx1", "can_tx1"};
static const char *const i2c0_groups[] = {"i2c0_scl", "i2c0_sda"};
static const char *const i2c1_groups[] = {"i2c1_scl", "i2c1_sda"};
static const char *const i2c2_groups[] = {"i2c2_scl", "i2c2_sda"};
static const char *const i2c3_groups[] = {"i2c3_scl", "i2c3_sda"};
static const char *const i2c4_groups[] = {"i2c4_scl", "i2c4_sda"};
static const char *const i2c5_groups[] = {"i2c5_scl", "i2c5_sda"};
static const char *const i2s0_groups[] = {"i2s0_ck", "i2s0_mck", "i2s0_sd_out", "i2s0_ws"};
static const char *const i2s1_groups[] = {"i2s1_ck", "i2s1_sd_in", "i2s1_ws"};
static const char *const isp_groups[] = {"isp_fsync0", "isp_fsync1", "isp_fsync2", "isp_fsync3", "isp_fsync4", 
"isp_fsync5", "isp_fsync6", "isp_fsync7"};
static const char *const jtag_groups[] = {"jtag_tck", "jtag_tdi", "jtag_tdo", "jtag_tms", "jtag_trst"};
static const char *const mem_groups[] = {"spi0_sclk"};
static const char *const pcie0_groups[] = {"pcie0_rstb"};
static const char *const pcie1_groups[] = {"pcie1_rstb"};
static const char *const ptp_groups[] = {"ptp_pps0", "ptp_pps1"};
static const char *const qspi0_groups[] = {"qspi0_cs0", "qspi0_cs1", "qspi0_io0", "qspi0_io1", "qspi0_io2", 
"qspi0_io3", "qspi0_io4", "qspi0_io5", "qspi0_io6", "qspi0_io7", "qspi0_sclk"};
static const char *const qspi1_groups[] = {"qspi1_cs0", "qspi1_cs1", "qspi1_io0", "qspi1_io1", "qspi1_io2", 
"qspi1_io3", "qspi1_io4", "qspi1_io5", "qspi1_io6", "qspi1_io7", "qspi1_sclk"};
static const char *const spi0_groups[] = {"spi0_cs", "spi0_miso", "spi0_mosi", "spi0_sclk"};
static const char *const spi1_groups[] = {"spi1_cs", "spi1_miso", "spi1_mosi", "spi1_sclk"};
static const char *const strap_groups[] = {"gpio_24","gpio_25","gpio_26","gpio_27","gpio_28",
"gpio_29","gpio_30","spi1_sclk","i2s0_mck","i2s0_ck","gpio_107","gpio_108"};
static const char *const ts_groups[] = {"ts_trig_in00","ts_trig_in01","ts_trig_in10","ts_trig_in11"};
static const char *const uart0_groups[] = {"uart0_cts","uart0_rts","uart0_rxd","uart0_txd"};
static const char *const uart1_groups[] = {"uart1_cts","uart1_rts","uart1_rxd","uart1_txd"};
static const char *const uart2_groups[] = {"uart2_cts","uart2_rts","uart2_rxd","uart2_txd"};
static const char *const uart3_groups[] = {"uart3_cts","uart3_rts","uart3_rxd","uart3_txd"};
static const char *const dsp_jtag_groups[] = {"dsp_jtag_tck","dsp_jtag_trst","dsp_jtag_tms","dsp_jtag_tdi","dsp_jtag_tdo"};
static const char *const err_rpt_l0_groups[] = {"err_rpt_l0_n","err_rpt_l0_p"};
static const char *const err_rpt_l1_groups[] = {"err_rpt_l1_n","err_rpt_l1_p"};
static const char *const rgmii0_groups[] = {"rgmii0_intr","rgmii0_mdc","rgmii0_mdio"};
static const char *const rgmii1_groups[] = {"rgmii1_intr","rgmii1_mdc","rgmii1_mdio"};

static const char* const otp_groups[] = {"spi0_mosi"};
static const char* const sdemmc0_groups[] = {"sdemmc0_led_ctr"};
static const char* const sdemmc1_groups[] = {"sdemmc1_led_ctr"};
static const char *const debug_groups[] = {"debug0","debug1","debug2","debug3","debug4","debug5","debug6","debug7"};
static const char *const pwm_groups[] = {"pwm0","pwm1","pwm2","pwm3"};
static const char *const null_groups[] = {};

static struct bst_pinctrl_function bst_iomux_funcs[] = {
	FUNCTION(gpio),
    FUNCTION(spi0),
	FUNCTION(spi1),   
	FUNCTION(bist),
	FUNCTION(boot),
	FUNCTION(can0),
    FUNCTION(can1),
	FUNCTION(i2c0),
	FUNCTION(i2c1),
	FUNCTION(i2c2),
	FUNCTION(i2c3),
	FUNCTION(i2c4),
	FUNCTION(i2c5),
	FUNCTION(i2s0),
	FUNCTION(i2s1),
	FUNCTION(isp),
	FUNCTION(jtag),
	FUNCTION(mem),
	FUNCTION(otp),
	FUNCTION(pcie0),
	FUNCTION(pcie1),
	FUNCTION(ptp),
	FUNCTION(qspi0),
	FUNCTION(qspi1),
	FUNCTION(rgmii0),
	FUNCTION(rgmii1),
	FUNCTION(err_rpt_l0),
   	FUNCTION(err_rpt_l1),
	FUNCTION(strap),
	FUNCTION(ts),
	FUNCTION(uart0),
	FUNCTION(uart1),
	FUNCTION(uart2),
	FUNCTION(uart3),
	FUNCTION(dsp_jtag),
    FUNCTION(sdemmc0),
    FUNCTION(sdemmc1),
    FUNCTION(debug),
    FUNCTION(pwm),
    FUNCTION(null),   
};

static struct bst_pinmux_pingroup bst_iomux_groups[] = {
	AON_MUX_GROUP(  gpio_24     ,   gpio    ,   strap   ,   0x0 ,   0,      0x50    ,   24  ,   0x200   ,   0   ),
    AON_MUX_GROUP(	spi0_sclk	,	spi0	,	mem	    ,	0x4	,	9	,	0x78	,	16	,	0x204	,	9	),
	AON_MUX_GROUP(	spi0_cs	    ,	spi0	,	gpio	,	0x4	,	10	,	0x78	,	8	,	0x204	,	10	),
	AON_MUX_GROUP(	spi0_mosi	,	spi0	,	otp	    ,	0x4	,	11	,	0x78	,	0	,	0x204	,	11	),
	AON_MUX_GROUP(	spi0_miso	,	spi0	,	gpio	,	0x4	,	12	,	0x7C	,	24	,	0x204	,	12	),
	AON_MUX_GROUP(	spi1_sclk	,	spi1	,	strap	,	0x4	,	13	,	0x7C	,	16	,	0x204	,	13	),
	AON_MUX_GROUP(	spi1_cs	    ,	spi1	,	gpio	,	0x4	,	14	,	0x7C	,	8	,	0x204	,	14	),
	AON_MUX_GROUP(	spi1_mosi	,	spi1	,	bist	,	0x4	,	15	,	0x7C	,	0	,	0x204	,	15	),
	AON_MUX_GROUP(	spi1_miso	,	spi1	,	gpio	,	0x4	,	16	,	0x80	,	24	,	0x204	,	16	),
    AON_MUX_GROUP(	gpio_25	    ,	gpio	,	strap	,	0x0	,	1	,	0x50	,	16	,	0x200	,	1	),
	AON_MUX_GROUP(	gpio_26	    ,	gpio	,	strap	,	0x0	,	2	,	0x50	,	8	,	0x200	,	2	),
	AON_MUX_GROUP(	gpio_27	    ,	gpio	,	strap	,	0x0	,	3	,	0x50	,	0	,	0x200	,	3	),
	AON_MUX_GROUP(	gpio_28	    ,	gpio	,	strap	,	0x0	,	4	,	0x54	,	24	,	0x200	,	4	),
	AON_MUX_GROUP(	gpio_29 	,	gpio	,	strap	,	0x0	,	5	,	0x54	,	16	,	0x200	,	5	),
	AON_MUX_GROUP(	gpio_30	    ,	gpio	,	strap	,	0x0	,	6	,	0x54	,	8	,	0x200	,	6	),
	AON_MUX_GROUP(	gpio_31	    ,	gpio	,	boot	,	0x0	,	7	,	0x54	,	0	,	0x200	,	7	),
	AON_MUX_GROUP(	jtag_tck	,	jtag	,	gpio	,	0x0	,	8	,	0x58	,	24	,	0x200	,	8	),
	AON_MUX_GROUP(	jtag_trst	,	jtag	,	gpio	,	0x0	,	9	,	0x58	,	16	,	0x200	,	9	),
	AON_MUX_GROUP(	jtag_tms	,	jtag	,	gpio	,	0x0	,	10	,	0x58	,	8	,	0x200	,	10	),
	AON_MUX_GROUP(	jtag_tdi	,	jtag	,	gpio	,	0x0	,	11	,	0x58	,	0	,	0x200	,	11	),
	AON_MUX_GROUP(	jtag_tdo	,	jtag	,	gpio	,	0x0	,	12	,	0x5C	,	24	,	0x200	,	12	),
	AON_MUX_GROUP(	uart0_txd	,	uart0	,	gpio	,	0x0	,	13	,	0x5C	,	16	,	0x200	,	13	),
	AON_MUX_GROUP(	uart0_rxd	,	uart0	,	gpio	,	0x0	,	14	,	0x5C	,	8	,	0x200	,	14	),
	AON_MUX_GROUP(	uart0_cts	,	uart0	,	gpio	,	0x0	,	15	,	0x5C	,	0	,	0x200	,	15	),
	AON_MUX_GROUP(	uart0_rts	,	uart0	,	gpio	,	0x0	,	16	,	0x60	,	24	,	0x200	,	16	),
	AON_MUX_GROUP(	uart1_txd	,	uart1	,	gpio	,	0x0	,	17	,	0x60	,	16	,	0x200	,	17	),
	AON_MUX_GROUP(	uart1_rxd	,	uart1	,	gpio	,	0x0	,	18	,	0x60	,	8	,	0x200	,	18	),
	AON_MUX_GROUP(	uart1_cts	,	uart1	,	gpio	,	0x0	,	19	,	0x60	,	0	,	0x200	,	19	),
	AON_MUX_GROUP(	uart1_rts	,	uart1	,	gpio	,	0x0	,	20	,	0x64	,	24	,	0x200	,	20	),
	AON_MUX_GROUP(	uart2_txd	,	uart2	,	gpio	,	0x0	,	21	,	0x64	,	16	,	0x200	,	21	),
	AON_MUX_GROUP(	uart2_rxd	,	uart2	,	gpio	,	0x0	,	22	,	0x64	,	8	,	0x200	,	22	),
	AON_MUX_GROUP(	uart2_cts	,	uart2	,	gpio	,	0x0	,	23	,	0x64	,	0	,	0x200	,	23	),
	AON_MUX_GROUP(	uart2_rts	,	uart2	,	gpio	,	0x0	,	24	,	0x68	,	24	,	0x200	,	24	),
	AON_MUX_GROUP(	uart3_txd	,	uart3	,	gpio	,	0x0	,	25	,	0x68	,	16	,	0x200	,	25	),
	AON_MUX_GROUP(	uart3_rxd	,	uart3	,	gpio	,	0x0	,	26	,	0x68	,	8	,	0x200	,	26	),
	AON_MUX_GROUP(	uart3_cts	,	uart3	,	gpio	,	0x0	,	27	,	0x68	,	0	,	0x200	,	27	),
	AON_MUX_GROUP(	uart3_rts	,	uart3	,	gpio	,	0x0	,	28	,	0x6C	,	24	,	0x200	,	28	),
	AON_MUX_GROUP(	i2c0_scl	,	i2c0	,	gpio	,	0x0	,	29	,	0x6C	,	16	,	0x200	,	29	),
	AON_MUX_GROUP(	i2c0_sda	,	i2c0	,	gpio	,	0x0	,	30	,	0x6C	,	8	,	0x200	,	30	),
	AON_MUX_GROUP(	i2c1_scl	,	i2c1	,	gpio	,	0x0	,	31	,	0x6C	,	0	,	0x200	,	31	),
	AON_MUX_GROUP(	i2c1_sda	,	i2c1	,	gpio	,	0x4	,	0	,	0x70	,	24	,	0x204	,	0	),
	AON_MUX_GROUP(	i2c2_scl	,	i2c2	,	gpio	,	0x4	,	1	,	0x70	,	16	,	0x204	,	1	),
	AON_MUX_GROUP(	i2c2_sda	,	i2c2	,	gpio	,	0x4	,	2	,	0x70	,	8	,	0x204	,	2	),
	AON_MUX_GROUP(	i2c3_scl	,	i2c3	,	gpio	,	0x4	,	3	,	0x70	,	0	,	0x204	,	3	),
	AON_MUX_GROUP(	i2c3_sda	,	i2c3	,	gpio	,	0x4	,	4	,	0x74	,	24	,	0x204	,	4	),
	AON_MUX_GROUP(	i2c4_scl	,	i2c4	,	gpio	,	0x4	,	5	,	0x74	,	16	,	0x204	,	5	),
	AON_MUX_GROUP(	i2c4_sda	,	i2c4	,	gpio	,	0x4	,	6	,	0x74	,	8	,	0x204	,	6	),
	AON_MUX_GROUP(	i2c5_scl	,	i2c5	,	gpio	,	0x4	,	7	,	0x74	,	0	,	0x204	,	7	),
	AON_MUX_GROUP(	i2c5_sda	,	i2c5	,	gpio	,	0x4	,	8	,	0x78	,	24	,	0x204	,	8	),
	AON_MUX_GROUP(	i2s0_mck	,	i2s0	,	strap	,	0x4	,	17	,	0x80	,	16	,	0x204	,	17	),
	AON_MUX_GROUP(	i2s0_ck	    ,	i2s0	,	strap	,	0x4	,	18	,	0x80	,	8	,	0x204	,	18	),
	AON_MUX_GROUP(	i2s0_sd_out	,	i2s0	,	gpio	,	0x4	,	19	,	0x80	,	0	,	0x204	,	19	),
	AON_MUX_GROUP(	i2s0_ws	    ,	i2s0	,	gpio	,	0x4	,	20	,	0x84	,	24	,	0x204	,	20	),
	AON_MUX_GROUP(	i2s1_ck	    ,	i2s1	,	gpio	,	0x4	,	21	,	0x84	,	16	,	0x204	,	21	),
	AON_MUX_GROUP(	i2s1_sd_in	,	i2s1	,	gpio	,	0x4	,	22	,	0x84	,	8	,	0x204	,	22	),
	AON_MUX_GROUP(	i2s1_ws	    ,	i2s1	,	gpio	,	0x4	,	23	,	0x84	,	0	,	0x204	,	23	),
	AON_MUX_GROUP(	pwm0	    ,	pwm	,	gpio	,	0x4	,	24	,	0x88	,	24	,	0x204	,	24	),
	AON_MUX_GROUP(	pwm1	    ,	pwm	,	gpio	,	0x4	,	25	,	0x88	,	16	,	0x204	,	25	),
	AON_MUX_GROUP(	pwm2	    ,	pwm	,	gpio	,	0x4	,	26	,	0x88	,	8	,	0x204	,	26	),
	AON_MUX_GROUP(	pwm3	    ,	pwm	,	gpio	,	0x4	,	27	,	0x88	,	0	,	0x204	,	27	),
	AON_MUX_GROUP(	qspi0_sclk	,	qspi0	,	gpio	,	0x4	,	28	,	0x8C	,	24	,	0x204	,	28	),
	AON_MUX_GROUP(	qspi0_cs0	,	qspi0	,	gpio	,	0x4	,	29	,	0x8C	,	16	,	0x204	,	29	),
	AON_MUX_GROUP(	qspi0_cs1	,	qspi0	,	gpio	,	0x4	,	30	,	0x8C	,	8	,	0x204	,	30	),
	AON_MUX_GROUP(	qspi0_io0	,	qspi0	,	gpio	,	0x4	,	31	,	0x8C	,	0	,	0x204	,	31	),
	AON_MUX_GROUP(	qspi0_io1	,	qspi0	,	gpio	,	0x8	,	0	,	0x90	,	24	,	0x208	,	0	),
	AON_MUX_GROUP(	qspi0_io2	,	qspi0	,	gpio	,	0x8	,	1	,	0x90	,	16	,	0x208	,	1	),
	AON_MUX_GROUP(	qspi0_io3	,	qspi0	,	gpio	,	0x8	,	2	,	0x90	,	8	,	0x208	,	2	),
	AON_MUX_GROUP(	qspi0_io4	,	qspi0	,	gpio	,	0x8	,	3	,	0x90	,	0	,	0x208	,	3	),
	AON_MUX_GROUP(	qspi0_io5	,	qspi0	,	gpio	,	0x8	,	4	,	0x94	,	24	,	0x208	,	4	),
	AON_MUX_GROUP(	qspi0_io6	,	qspi0	,	gpio	,	0x8	,	5	,	0x94	,	16	,	0x208	,	5	),
	AON_MUX_GROUP(	qspi0_io7	,	qspi0	,	gpio	,	0x8	,	6	,	0x94	,	8	,	0x208	,	6	),
	AON_MUX_GROUP(	qspi1_sclk	,	qspi1	,	gpio	,	0x8	,	7	,	0x94	,	0	,	0x208	,	7	),
	AON_MUX_GROUP(	qspi1_cs0	,	qspi1	,	gpio	,	0x8	,	8	,	0x98	,	24	,	0x208	,	8	),
	AON_MUX_GROUP(	qspi1_cs1	,	qspi1	,	gpio	,	0x8	,	9	,	0x98	,	16	,	0x208	,	9	),
	AON_MUX_GROUP(	qspi1_io0	,	qspi1	,	gpio	,	0x8	,	10	,	0x98	,	8	,	0x208	,	10	),
	AON_MUX_GROUP(	qspi1_io1	,	qspi1	,	gpio	,	0x8	,	11	,	0x98	,	0	,	0x208	,	11	),
	AON_MUX_GROUP(	qspi1_io2	,	qspi1	,	gpio	,	0x8	,	12	,	0x9C	,	24	,	0x208	,	12	),
	AON_MUX_GROUP(	qspi1_io3	,	qspi1	,	gpio	,	0x8	,	13	,	0x9C	,	16	,	0x208	,	13	),
	AON_MUX_GROUP(	qspi1_io4	,	qspi1	,	gpio	,	0x8	,	14	,	0x9C	,	8	,	0x208	,	14	),
	AON_MUX_GROUP(	qspi1_io5	,	qspi1	,	gpio	,	0x8	,	15	,	0x9C	,	0	,	0x208	,	15	),
	AON_MUX_GROUP(	qspi1_io6	,	qspi1	,	gpio	,	0x8	,	16	,	0xA0	,	24	,	0x208	,	16	),
	AON_MUX_GROUP(  qspi1_io7   ,	qspi1	,	gpio	,	0x8	,	17	,	0xA0	,	16	,	0x208	,	17	),
    AON_MUX_GROUP(	can_tx0	    ,	can0	,	gpio	,	0x8	,	18	,	0xA0	,	8	,	0x208	,	18	),
	AON_MUX_GROUP(	can_rx0	    ,	can0	,	gpio	,	0x8	,	19	,	0xA0	,	0	,	0x208	,	19	),
	AON_MUX_GROUP(	can_tx1	    ,	can1	,	gpio	,	0x8	,	20	,	0xA4	,	24	,	0x208	,	20	),
	AON_MUX_GROUP(	can_rx1	    ,	can1    ,	gpio	,	0x8	,	21	,	0xA4	,	16	,	0x208	,	21	),
	AON_MUX_GROUP(	gpio_107	,	gpio	,	strap	,	0x8	,	22	,	0xA4	,	8	,	0x208	,	22	),
	AON_MUX_GROUP(	gpio_108	,	gpio	,	strap	,	0x8	,	23	,	0xA4	,	0	,	0x208	,	23	),
    TOP_MUX_GROUP(	sdemmc0_clk	,	null	,	null	,	-1	,	-1	,	0xA0	,	24	,	-1	    ,	-1	), 
	TOP_MUX_GROUP(	sdemmc0_cmd	,	null	,	null	,	-1	,	-1	,	0xA0	,	16	,	0x200	,	0	),
	TOP_MUX_GROUP(	sdemmc0_dat0,	null	,	null	,	-1	,	-1	,	0xA0	,	8	,	0x200	,	1	),
	TOP_MUX_GROUP(	sdemmc0_dat1,	null	,	null	,	-1	,	-1	,	0xA0	,	0	,	0x200	,	2	),
	TOP_MUX_GROUP(	sdemmc0_dat2,	null	,	null	,	-1	,	-1	,	0xA4	,	24	,	0x200	,	3	),
	TOP_MUX_GROUP(	sdemmc0_dat3,	null	,	null	,	-1	,	-1	,	0xA4	,	16	,	0x200	,	4	),
	TOP_MUX_GROUP(	sdemmc0_dat4,	null	,	null	,	-1	,	-1	,	0xA4	,	8	,	0x200	,	5	),
	TOP_MUX_GROUP(	sdemmc0_dat5,	null	,	null	,	-1	,	-1	,	0xA4	,	0	,	0x200	,	6	),
	TOP_MUX_GROUP(	sdemmc0_dat6,	null	,	null	,	-1	,	-1	,	0xA8	,	24	,	0x200	,	7	),
	TOP_MUX_GROUP(	sdemmc0_dat7,	null    ,	null	,	-1	,	-1	,	0xA8	,	16	,	0x200	,	8	),
	TOP_MUX_GROUP(	sdemmc0_rstb,	null	,	null	,	-1	,	-1	,	0xA8	,	8	,	-1	    ,	-1	),
	TOP_MUX_GROUP(	sdemmc0_cdn	,	null	,	null	,	-1	,	-1	,	0xA8	,	0	,	0x200	,	9	),
	TOP_MUX_GROUP(	sdemmc0_wp	,	null	,	null	,	-1	,	-1	,	0xAC	,	24	,	0x200	,	10	),
	TOP_MUX_GROUP(	err_rpt_l0_p,	err_rpt_l0,	gpio	,	0x0	,	0	,	0xAC	,	16	,	0x200	,	11	),
	TOP_MUX_GROUP(	err_rpt_l0_n,	err_rpt_l0,	gpio	,	0x0	,	1	,	0xAC	,	8	,	0x200	,	12	),
	TOP_MUX_GROUP(	sdemmc1_clk	,	null	,	null	,	-1	,	-1	,	0xAC	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc1_cmd	,	null	,	null	,	-1	,	-1	,	0xB0	,	24	,	0x200	,	13	),
	TOP_MUX_GROUP(	sdemmc1_dat0,	null	,	null	,	-1	,	-1	,	0xB0	,	16	,	0x200	,	14	),
	TOP_MUX_GROUP(	sdemmc1_dat1,	null	,	null	,	-1	,	-1	,	0xB0	,	8	,	0x200	,	15	),
	TOP_MUX_GROUP(	sdemmc1_dat2,	null	,	null	,	-1	,	-1	,	0xB0	,	0	,	0x200	,	16	),
	TOP_MUX_GROUP(	sdemmc1_dat3,	null	,	null	,	-1	,	-1	,	0xB4	,	24	,	0x200	,	17	),
	TOP_MUX_GROUP(	sdemmc1_dat4,	null	,	null	,	-1	,	-1	,	0xB4	,	16	,	0x200	,	18	),
	TOP_MUX_GROUP(	sdemmc1_dat5,	null    ,	null	,	-1	,	-1	,	0xB4	,	8	,	0x200	,	19	),
	TOP_MUX_GROUP(	sdemmc1_dat6,	null	,	null	,	-1	,	-1	,	0xB4	,	0	,	0x200	,	20	),
	TOP_MUX_GROUP(	sdemmc1_dat7,	null	,	null	,	-1	,	-1	,	0xB8	,	24	,	0x200	,	21	),
	TOP_MUX_GROUP(	sdemmc1_rstb,	null	,	null	,	-1	,	-1	,	0xB8	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc1_cdn	,	null	,	null	,	-1	,	-1	,	0xB8	,	8	,	0x200	,	22	),
	TOP_MUX_GROUP(	sdemmc1_wp	,	null	,	null	,	-1	,	-1	,	0xB8	,	0	,	0x200	,	23	),
	TOP_MUX_GROUP(	err_rpt_l1_p,	err_rpt_l1,	gpio	,	0x0	,	2	,	0xBC	,	24	,	0x200	,	24	),
	TOP_MUX_GROUP(	err_rpt_l1_n,	err_rpt_l1,	gpio	,	0x0	,	3	,	0xBC	,	16	,	0x200	,	25	),
    TOP_MUX_GROUP(	rgmii0_txd0	,	null	,	null	,	-1	,	-1	,	0xBC	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii0_txd1	,	null	,	null	,	-1	,	-1	,	0xBC	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii0_txd2	,	null	,	null	,	-1	,	-1	,	0xC0	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii0_txd3	,	null	,	null	,	-1	,	-1	,	0xC0	,	16	,	-1	,	-1	),
    TOP_MUX_GROUP(	rgmii0_txctrl,	null	,	null	,	-1	,	-1	,	0xC4	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	gmii0_txd4	,	null	,	null	,	-1	,	-1	,	0xC0	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	gmii0_txd5	,	null	,	null	,	-1	,	-1	,	0xC0	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	gmii0_txd6	,	null	,	null	,	-1	,	-1	,	0xC4	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	gmii0_txd7	,	null	,	null	,	-1	,	-1	,	0xC4	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	gmii0_txer	,	null	,	null	,	-1	,	-1	,	0xC4	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	mii0_txclk	    ,	null	,	null	,	-1	,	-1	,	0xC8	,	24	,	0x200	,	26	),
	TOP_MUX_GROUP(	rgmii0_gtxclk	,	null	,	null	,	-1	,	-1	,	0xC8	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii0_rxd0	    ,	null	,	null	,	-1	,	-1	,	0xC8	,	8	,	0x200	,	27	),
	TOP_MUX_GROUP(	rgmii0_rxd1	    ,	null	,	null	,	-1	,	-1	,	0xC8	,	0	,	0x200	,	28	),
	TOP_MUX_GROUP(	rgmii0_rxd2	    ,	null	,	null	,	-1	,	-1	,	0xCC	,	24	,	0x200	,	29	),
	TOP_MUX_GROUP(	rgmii0_rxd3	    ,	null	,	null	,	-1	,	-1	,	0xCC	,	16	,	0x200	,	30	),
	TOP_MUX_GROUP(	gmii0_rxd4	    ,	null	,	null	,	-1	,	-1	,	0xCC	,	8	,	0x200	,	31	),
	TOP_MUX_GROUP(	gmii0_rxd5	    ,	null	,	null	,	-1	,	-1	,	0xCC	,	0	,	0x204	,	0	),
	TOP_MUX_GROUP(	gmii0_rxd6	    ,	null	,	null	,	-1	,	-1	,	0xD0	,	24	,	0x204	,	1	),
	TOP_MUX_GROUP(	gmii0_rxd7	    ,	null	,	null	,	-1	,	-1	,	0xD0	,	16	,	0x204	,	2	),
	TOP_MUX_GROUP(	gmii0_rxer	    ,	null	,	null	,	-1	,	-1	,	0xD0	,	8	,	0x204	,	3	),
	TOP_MUX_GROUP(	rgmii0_rxctrl	,	null	,	null	,	-1	,	-1	,	0xD0	,	0	,	0x204	,	4	),
	TOP_MUX_GROUP(	rgmii0_rxclk	,	null	,	null	,	-1	,	-1	,	0xD4	,	24	,	0x204	,	5	),
	TOP_MUX_GROUP(	rgmii0_mdc	    ,	rgmii0	,	gpio	,	0x0	,	4	,	0xD4	,	16	,	0x204	,	6	),
	TOP_MUX_GROUP(	rgmii0_mdio	    ,	rgmii0	,	gpio	,	0x0	,	5	,	0xD4	,	8	,	0x204	,	7	),
	TOP_MUX_GROUP(	rgmii1_txd0	    ,	null	,	null	,	-1	,	-1	,	0xD4	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii1_txd1	    ,	null	,	null	,	-1	,	-1	,	0xD8	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii1_txd2	    ,	null	,	null	,	-1	,	-1	,	0xD8	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii1_txd3	    ,	null	,	null	,	-1	,	-1	,	0xD8	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	mii1_txer	    ,	null	,	null	,	-1	,	-1	,	0xD8	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii1_txctrl	,	null	,	null	,	-1	,	-1	,	0xDC	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	mii1_txclk	    ,	null	,	null	,	-1	,	-1	,	0xDC	,	16	,	0x204	,	8	),
	TOP_MUX_GROUP(	rgmii1_gtxclk	,	null	,	null	,	-1	,	-1	,	0xDC	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	rgmii1_rxd0	    ,	null	,	null	,	-1	,	-1	,	0xDC	,	0	,	0x204	,	9	),
	TOP_MUX_GROUP(	rgmii1_rxd1	    ,	null	,	null	,	-1	,	-1	,	0xE0	,	24	,	0x204	,	10	),
	TOP_MUX_GROUP(	rgmii1_rxd2	    ,	null	,	null	,	-1	,	-1	,	0xE0	,	16	,	0x204	,	11	),
	TOP_MUX_GROUP(	rgmii1_rxd3	    ,	null	,	null	,	-1	,	-1	,	0xE0	,	8	,	0x204	,	12	),
	TOP_MUX_GROUP(	mii1_rxer	    ,	null	,	null	,	-1	,	-1	,	0xE0	,	0	,	0x204	,	13	),
	TOP_MUX_GROUP(	rgmii1_rxctrl	,	null	,	null	,	-1	,	-1	,	0xE4	,	24	,	0x204	,	14	),
	TOP_MUX_GROUP(	rgmii1_rxclk	,	null	,	null	,	-1	,	-1	,	0xE4	,	16	,	0x204	,	15	),
	TOP_MUX_GROUP(	rgmii1_mdc	    ,	rgmii1	,	gpio	,	0x4	,	0	,	0xE4	,	8	,	0x204	,	16	),
	TOP_MUX_GROUP(	rgmii1_mdio	    ,	rgmii1	,	gpio	,	0x4	,	1	,	0xE4	,	0	,	0x204	,	17	),
	TOP_MUX_GROUP(	ptp_clk	        ,	null	,	null	,	-1	,	-1	,	0xE8	,	24	,	0x204	,	18	),
	TOP_MUX_GROUP(	rgmii0_intr	,	rgmii0	,	gpio	,	0x4	,	2	,	0xE8	,	16	,	0x204	,	19	),
	TOP_MUX_GROUP(	rgmii1_intr	,	rgmii1	,	gpio	,	0x4	,	3	,	0xE8	,	8	,	0x204	,	20	),	
    TOP_MUX_GROUP(	ts_trig_in00	,	ts	,	gpio	,	0x4	,	4	,	0xE8	,	0	,	0x204	,	21	),
	TOP_MUX_GROUP(	ts_trig_in01	,	ts	,	gpio	,	0x4	,	5	,	0xEC	,	24	,	0x204	,	22	),
	TOP_MUX_GROUP(	ts_trig_in10	,	ts	,	gpio	,	0x4	,	6	,	0xEC	,	16	,	0x204	,	23	),
	TOP_MUX_GROUP(	ts_trig_in11	,	ts	,	gpio	,	0x4	,	7	,	0xEC	,	8	,	0x204	,	24	),
	TOP_MUX_GROUP(	ptp_pps0	    ,	ptp	,	gpio	,	0x4	,	8	,	0xEC	,	0	,	0x204	,	25	),
	TOP_MUX_GROUP(	ptp_pps1	    ,	ptp	,	gpio	,	0x4	,	9	,	0xF0	,	24	,	0x204	,	26	),
	TOP_MUX_GROUP(	vout_r0	    ,	null	,	null	,	-1	,	-1	,	0xF0	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r1	    ,   null	,	null	,	-1	,	-1	,	0xF0	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r2	    ,	null	,	null	,	-1	,	-1	,	0xF0	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r3	    ,	null	,	null	,	-1	,	-1	,	0xF4	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r4	    ,	null	,	null	,	-1	,	-1	,	0xF4	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r5	    ,	null	,	null	,	-1	,	-1	,	0xF4	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r6	    ,	null	,	null	,	-1	,	-1	,	0xF4	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_r7	    ,	null	,	null	,	-1	,	-1	,	0xF8	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g0	    ,   null	,	null	,	-1	,	-1	,	0xF8	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g1	    ,	null	,	null	,	-1	,	-1	,	0xF8	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g2	    ,	null	,	null	,	-1	,	-1	,	0xF8	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g3	    ,	null	,	null	,	-1	,	-1	,	0xFC	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g4	    ,	null	,	null	,	-1	,	-1	,	0xFC	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g5	    ,	null	,	null	,	-1	,	-1	,	0xFC	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g6	    ,	null	,	null	,	-1	,	-1	,	0xFC	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_g7	    ,	null	,	null	,	-1	,	-1	,	0x100	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b0	    ,	null	,	null	,	-1	,	-1	,	0x100	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b1	    ,	null	,	null	,	-1	,	-1	,	0x100	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b2	    ,	null	,	null	,	-1	,	-1	,	0x100	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b3	    ,	null	,	null	,	-1	,	-1	,	0x104	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b4	    ,	null	,	null	,	-1	,	-1	,	0x104	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b5	    ,	null	,	null	,	-1	,	-1	,	0x104	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b6	    ,	null	,	null	,	-1	,	-1	,	0x104	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_b7	    ,	null	,	null	,	-1	,	-1	,	0x108	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_hs	    ,	null	,	null	,	-1	,	-1	,	0x108	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_vs	    ,	null	,	null	,	-1	,	-1	,	0x108	,	8	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_de	    ,	null	,	null	,	-1	,	-1	,	0x108	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_pclk	,	null	,	null	,	-1	,	-1	,	0x10C	,	24	,	-1	,	-1	),
	TOP_MUX_GROUP(	vout_pdb	,	null	,	null	,	-1	,	-1	,	0x10C	,	16	,	-1	,	-1	),
	TOP_MUX_GROUP(	vin_r0	,	null,	null	,	-1	,	-1	,	0x10C	,	8	,	0x204	,	27	),
	TOP_MUX_GROUP(	vin_r1	,	null,	null	,	-1	,	-1	,	0x10C	,	0	,	0x204	,	28	),
	TOP_MUX_GROUP(	vin_r2	,	null,	null	,	-1	,	-1	,	0x120	,	24	,	0x204	,	29	),
	TOP_MUX_GROUP(	vin_r3	,	null,	null	,	-1	,	-1	,	0x120	,	16	,	0x204	,	30	),
	TOP_MUX_GROUP(	vin_r4	,	null,	null	,	-1	,	-1	,	0x120	,	8	,	0x204	,	31	),
	TOP_MUX_GROUP(	vin_g0	,	null,	null	,	-1	,	-1	,	0x120	,	0	,	0x208	,	0	),
	TOP_MUX_GROUP(	vin_g1	,	null,	null	,	-1	,	-1	,	0x128	,	24	,	0x208	,	1	),
	TOP_MUX_GROUP(	vin_g2	,	null,	null	,	-1	,	-1	,	0x128	,	16	,	0x208	,	2	),
	TOP_MUX_GROUP(	vin_g3	,	null,	null	,	-1	,	-1	,	0x128	,	8	,	0x208	,	3	),
	TOP_MUX_GROUP(	vin_g4	,	null,	null	,	-1	,	-1	,	0x128	,	0	,	0x208	,	4	),
	TOP_MUX_GROUP(	vin_g5	,	null,	null	,	-1	,	-1	,	0x12C	,	16	,	0x208	,	5	),
	TOP_MUX_GROUP(	vin_b0	,	null,	null	,	-1	,	-1	,	0x12C	,	8	,	0x208	,	6	),
	TOP_MUX_GROUP(	vin_b1	,	null,	null	,	-1	,	-1	,	0x12C	,	0	,	0x208	,	7	),
	TOP_MUX_GROUP(	vin_b2	,	null,	null	,	-1	,	-1	,	0x130	,	16	,	0x208	,	8	),
	TOP_MUX_GROUP(	vin_b3	,	null,	null	,	-1	,	-1	,	0x130	,	8	,	0x208	,	9	),
	TOP_MUX_GROUP(	vin_b4	,	null,	null	,	-1	,	-1	,	0x130	,	0	,	0x208	,	10	),
	TOP_MUX_GROUP(	vin_hs	,	null,	null	,	-1	,	-1	,	0x134	,	8	,	0x208	,	11	),
	TOP_MUX_GROUP(	vin_vs	,	null,	null	,	-1	,	-1	,	0x134	,	0	,	0x208	,	12	),
	TOP_MUX_GROUP(	vin_de	,	null,	null	,	-1	,	-1	,	0x138	,	24	,	0x208	,	13	),
	TOP_MUX_GROUP(	vin_llc	,	null,	null	,	-1	,	-1	,	0x138	,	16	,	0x208	,	14	),
	TOP_MUX_GROUP(	debug0	,	debug	,	gpio	,	0x4	,	10	,	0x138	,	8	,	0x208	,	15	),
	TOP_MUX_GROUP(	debug1	,	debug	,	gpio	,	0x4	,	11	,	0x138	,	0	,	0x208	,	16	),
	TOP_MUX_GROUP(	debug2	,	debug	,	gpio	,	0x4	,	12	,	0x13C	,	24	,	0x208	,	17	),
	TOP_MUX_GROUP(	debug3	,	debug	,	gpio	,	0x4	,	13	,	0x13C	,	16	,	0x208	,	18	),
	TOP_MUX_GROUP(	debug4	,	debug	,	gpio	,	0x4	,	14	,	0x13C	,	8	,	0x208	,	19	),
	TOP_MUX_GROUP(	debug5	,	debug	,	gpio	,	0x4	,	15	,	0x13C	,	0	,	0x208	,	20	),
	TOP_MUX_GROUP(	debug6	,	debug	,	gpio	,	0x4	,	16	,	0x140	,	24	,	0x208	,	21	),
	TOP_MUX_GROUP(	debug7	,	debug	,	gpio	,	0x4	,	17	,	0x140	,	16	,	0x208	,	22	),
    TOP_MUX_GROUP(	dsp_jtag_tck		,	dsp_jtag,	gpio,	0x4	,	18	,	0x140	,	8	,	0x208	,	23	),
	TOP_MUX_GROUP(	dsp_jtag_trst		,	dsp_jtag,	gpio,	0x4	,	19	,	0x140	,	0	,	0x208	,	24	),
	TOP_MUX_GROUP(	dsp_jtag_tms		,	dsp_jtag,	gpio,	0x4	,	20	,	0x144	,	24	,	0x208	,	25	),
	TOP_MUX_GROUP(	dsp_jtag_tdi		,	dsp_jtag,	gpio,	0x4	,	21	,	0x144	,	16	,	0x208	,	26	),
	TOP_MUX_GROUP(	dsp_jtag_tdo		,	dsp_jtag,	gpio,	0x4	,	22	,	0x144	,	8	,	0x208	,	27	),
	TOP_MUX_GROUP(	sdemmc0_led_ctl,	sdemmc0,	gpio	,	0x4	,	23	,	0x144	,	0	,	0x208	,	28	),
	TOP_MUX_GROUP(	sdemmc1_led_ctl,	sdemmc1,	gpio	,	0x4	,	24	,	0x148	,	24	,	0x208	,	29	),
	TOP_MUX_GROUP(	gpio_15	    ,	null,	null	,	-1	,	-1	,	0x148	,	16	,	0x208	,	30	),
	TOP_MUX_GROUP(	isp_fsync0	,	isp	,	gpio	,	0x4	,	25	,	0x148	,	8	,	0x208	,	31	),
	TOP_MUX_GROUP(	isp_fsync1	,	isp	,	gpio	,	0x4	,	26	,	0x148	,	0	,	0x20c	,	0	),
	TOP_MUX_GROUP(	isp_fsync2	,	isp	,	gpio	,	0x4	,	27	,	0x14C	,	24	,	0x20c	,	1	),
	TOP_MUX_GROUP(	isp_fsync3	,	isp	,	gpio	,	0x4	,	28	,	0x14C	,	16	,	0x20c	,	2	),
	TOP_MUX_GROUP(	isp_fsync4	,	isp	,	gpio	,	0x4	,	29	,	0x14C	,	8	,	0x20c	,	3	),
	TOP_MUX_GROUP(	isp_fsync5	,	isp	,	gpio	,	0x4	,	30	,	0x14C	,	0	,	0x20c	,	4	),
	TOP_MUX_GROUP(	isp_fsync6	,	isp	,	gpio	,	0x4	,	31	,	0x150	,	24	,	0x20c	,	5	),
	TOP_MUX_GROUP(	isp_fsync7	,	isp	,	gpio	,	0x8	,	0	,	0x150	,	16	,	0x20c	,	6	),
	TOP_MUX_GROUP(	pcie0_rstb	,	pcie0,	gpio	,	0x8	,	1	,	0x150	,	8	,	0x20c	,	7	),
	TOP_MUX_GROUP(	pcie1_rstb	,	pcie1,	gpio	,	0x8	,	2	,	0x150	,	0	,	0x20c	,	8	), 
	TOP_MUX_GROUP(	sdemmc0_pvdd18pocsd0,	null,	null,	-1	,	-1	,	0x154	,	4	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc0_pvdd18pocsd1,	null,	null,	-1	,	-1	,	0x154	,	2	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc0_pvdd18pocsd2,	null,	null,	-1	,	-1	,	0x154	,	0	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc1_pvdd18pocsd0,	null,	null,	-1	,	-1	,	0x158	,	4	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc1_pvdd18pocsd1,	null,	null,	-1	,	-1	,	0x158	,	2	,	-1	,	-1	),
	TOP_MUX_GROUP(	sdemmc1_pvdd18pocsd2,	null,	null,	-1	,	-1	,	0x158	,	0	,	-1	,	-1	),
};
			  
static int bst_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	return pctrl->part->ngroups;
}

static const char *bst_get_group_name(struct pinctrl_dev *pctldev,
					unsigned group)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	return pctrl->part->groups[group].name;
}

static int bst_get_group_pins(struct pinctrl_dev *pctldev,
				unsigned group,
				const unsigned **pins,
				unsigned *num_pins)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = pctrl->part->groups[group].pins;
	*num_pins = pctrl->part->groups[group].npins;
	return 0;
}

static const struct pinctrl_ops bst_pinctrl_ops = {
	.get_groups_count	= bst_get_groups_count,
	.get_group_name		= bst_get_group_name,
	.get_group_pins		= bst_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinconf_generic_dt_free_map,
};

static int bst_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->part->nfunctions;
}

static const char *bst_get_function_name(struct pinctrl_dev *pctldev,
					 unsigned function)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->part->functions[function].name;
}

static int bst_get_function_groups(struct pinctrl_dev *pctldev,
				   unsigned function,
				   const char * const **groups,
				   unsigned * const num_groups)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctrl->part->functions[function].groups;
	*num_groups = pctrl->part->functions[function].ngroups;
	return 0;
}

static int bst_pinmux_set_mux(struct pinctrl_dev *pctldev,
			      unsigned function,
			      unsigned group)
{
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	const struct bst_pinmux_pingroup *g;
	unsigned long flags;
	u32 val;
	int i;
    void __iomem *base;
	
    g = &pctrl->part->groups[group];
	if(g->pmm_reg == -1){
		return -ENOTSUPP;
	}
	
	for (i = 0; i < g->nfuncs; i++) {
		if (g->funcs[i] == function)
			break;
	}

	if (WARN_ON(i == g->nfuncs)) {
	    return -EINVAL;
    }

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	if (g->reg_index == BST_PINC_INDEX_AON) {
		    base = pctrl->aon_regs;
        } else {
            base = pctrl->top_regs;
    }

	val = readl(base + g->pmm_reg);
	if (i) {
		val |= 1 << g->pmm_offset ;
	} else {
		val &= ~ (1 << g->pmm_offset );
	}

	writel(val, base + g->pmm_reg);  
    raw_spin_unlock_irqrestore(&pctrl->lock, flags); 

	return 0;
}

static const struct pinmux_ops bst_pinmux_ops = {
//	.request		= bst_pinmux_request,
	.get_functions_count = bst_get_functions_count,
	.get_function_name = bst_get_function_name,
	.get_function_groups = bst_get_function_groups,
	.set_mux = bst_pinmux_set_mux,
};

static int bst_config_group_get(struct pinctrl_dev *pctldev,
				unsigned int group,
				unsigned long *config)
{
	const struct bst_pinmux_pingroup *g;
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned param = pinconf_to_config_param(*config);
	unsigned arg = 0;
	u32 val;
    void __iomem *base;

	g = &pctrl->part->groups[group];
	if (PIN_CONFIG_INPUT_ENABLE == param) {
		if (g->io_ie_reg == -1) {
			return -ENOTSUPP;
		}
	}
 
	if (PIN_CONFIG_INPUT_ENABLE != param) {
		if (g->io_cfg_reg == -1) {
			return -ENOTSUPP;
		}
	}

	if (g->reg_index == BST_PINC_INDEX_AON) {
		base = pctrl->aon_regs;
    } else {
        base = pctrl->top_regs;
    }
    val = readl(base + g->io_cfg_reg);
	val >>= g->io_cfg_offset;
	/* Convert register value to pinconf value */
	switch (param) {
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			if (val & BST_CFG_SMT_BIT) {
				arg = 1;
			}
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			if ((val & BST_CFG_PULL_MASK) == 0) {
				arg = 1;
			}
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			if ((val & BST_CFG_PULL_MASK) == BST_CFG_PULL_DOWN_BITS) {
				arg = 1;
			}
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			if ((val & BST_CFG_PULL_MASK) == BST_CFG_PULL_UP_BITS) {
				arg = 1;
			}
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			arg = val & BST_CFG_DRIVE_STRENGTH_MASK;
			break;
		
		case PIN_CONFIG_INPUT_ENABLE:
			/* Pin is output */
			val = readl(base + g->io_ie_reg);
			val >>= g->io_ie_offset;
			if (val & 1) {
				arg = 1;
			}
			break;
		default:
			return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int bst_config_group_set(struct pinctrl_dev *pctldev,
				unsigned group,
				unsigned long *configs,
				unsigned num_configs)
{
	const struct bst_pinmux_pingroup *g;
	struct bst_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long flags;
	unsigned param;
	unsigned arg;
	int ie;
	u32 val;
	u32 tmp;
	int i;
    void __iomem *base;

	g = &pctrl->part->groups[group];
	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		if (PIN_CONFIG_INPUT_ENABLE == param) {
			if(g->io_ie_reg == -1)
				continue;
		}

		if (PIN_CONFIG_INPUT_ENABLE != param) {
			if(g->io_cfg_reg == -1)
				continue;
		}
		ie = 0;
		/* Convert pinconf values to register values */
        if (g->reg_index == BST_PINC_INDEX_AON) {
            base = pctrl->aon_regs;   
        } else {
            base = pctrl->top_regs;
        }

        val = readl(base + g->io_cfg_reg);
        switch (param) {
			case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
				if(arg){
					val |= BST_CFG_SMT_BIT << g->io_cfg_offset;
				}else{
					val &= ~(BST_CFG_SMT_BIT << g->io_cfg_offset);
				}
				break;
			case PIN_CONFIG_BIAS_PULL_DOWN:
				val &= ~(BST_CFG_PULL_MASK << g->io_cfg_offset);
				val |= BST_CFG_PULL_DOWN_BITS << g->io_cfg_offset;
				break;
			case PIN_CONFIG_BIAS_PULL_UP:
				val &= ~(BST_CFG_PULL_MASK << g->io_cfg_offset);
				val |= BST_CFG_PULL_UP_BITS << g->io_cfg_offset;
				break;
			case PIN_CONFIG_BIAS_DISABLE:
				val &= ~(BST_CFG_PULL_MASK << g->io_cfg_offset);
				break;
			case PIN_CONFIG_DRIVE_STRENGTH:
				if(arg >= 16){
					return -EINVAL;
				}
				val &= ~(BST_CFG_DRIVE_STRENGTH_MASK << g->io_cfg_offset);
				val |= (BST_CFG_DRIVE_STRENGTH_MASK & arg) << g->io_cfg_offset;
				break;

			case PIN_CONFIG_INPUT_ENABLE:
				raw_spin_lock_irqsave(&pctrl->lock, flags);
				tmp = readl(base + g->io_ie_reg);
				if(arg){
					tmp |= 1 << g->io_ie_offset;
				}else{
					tmp &= ~(1 << g->io_ie_offset);
				}
				writel(tmp, base + g->io_ie_reg);
				raw_spin_unlock_irqrestore(&pctrl->lock, flags);

				ie = 1;
				break;
			default:
				return -EINVAL;
		}

		/* Range-check user-supplied value */
		if(!ie){
			raw_spin_lock_irqsave(&pctrl->lock, flags);
			writel(val, base + g->io_cfg_reg);
			raw_spin_unlock_irqrestore(&pctrl->lock, flags);
		}
    }

	return 0;
}

static const struct pinconf_ops bst_pinconf_ops = {
	.is_generic	= true,
	.pin_config_group_get = bst_config_group_get,
	.pin_config_group_set = bst_config_group_set,
};
static const struct of_device_id bst_pinctrl_of_match[] = {
	{ .compatible = "bst,pinctrl", },
	{ },
};

static int bst_pinctrl_probe(struct platform_device *pdev, const struct bst_pinctrl_part_data *info)
{
	struct bst_pinctrl *pctl;
	struct resource *res;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	pctl->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, pctl);
    pctl->part = info;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "aon");
	pctl->aon_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctl->aon_regs)) {
    	return PTR_ERR(pctl->aon_regs);
    }

    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "top");
	pctl->top_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctl->top_regs)) {
    	return PTR_ERR(pctl->top_regs);
    }
    pctl->desc.owner = THIS_MODULE;
	pctl->desc.pctlops = &bst_pinctrl_ops;
	pctl->desc.pmxops = &bst_pinmux_ops;
	pctl->desc.confops = &bst_pinconf_ops;
	pctl->desc.name = dev_name(&pdev->dev);
    pctl->desc.pins = info->pins;
    pctl->desc.npins = info->npins;

	pctl->pctrl = devm_pinctrl_register(&pdev->dev, &pctl->desc, pctl);
	if (IS_ERR(pctl->pctrl)) {
		dev_err(&pdev->dev, "Failed to register pinctrl device\n");
		return PTR_ERR(pctl->pctrl);
	}

	return 0;
}

static const struct bst_pinctrl_part_data a1000_iomux_part = {
    .pins = bst_iomux_pins,
	.npins = ARRAY_SIZE(bst_iomux_pins),
	.functions = bst_iomux_funcs,
	.nfunctions = ARRAY_SIZE(bst_iomux_funcs),
	.groups = bst_iomux_groups,
	.ngroups = ARRAY_SIZE(bst_iomux_groups),
};

static int a1000_pinctrl_probe(struct platform_device *pdev)
{
	return bst_pinctrl_probe(pdev, &a1000_iomux_part);
}

static struct platform_driver bst_pinctrl_driver = {
	.driver = {
		.name = "bst-pinctrl",
		.of_match_table = bst_pinctrl_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = a1000_pinctrl_probe,
};

static int __init bst_pinctrl_register(void)
{
	return platform_driver_register(&bst_pinctrl_driver);
}
arch_initcall(bst_pinctrl_register);

/* generated vector source file - do not edit */
#include "bsp_api.h"
#include "vector_data.h"

extern void Default_Handler(void);

BSP_DONT_REMOVE fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION (BSP_SECTION_APPLICATION_VECTORS) =
{ 
  Default_Handler, /* IRQ:0 */
  Default_Handler, /* IRQ:1 */
  Default_Handler, /* IRQ:2 */
  Default_Handler, /* IRQ:3 */
  Default_Handler, /* IRQ:4 */
  Default_Handler, /* IRQ:5 */
  Default_Handler, /* IRQ:6 */
  Default_Handler, /* IRQ:7 */
  Default_Handler, /* IRQ:8 */
  Default_Handler, /* IRQ:9 */
  Default_Handler, /* IRQ:10 */
  Default_Handler, /* IRQ:11 */
  Default_Handler, /* IRQ:12 */
  Default_Handler, /* IRQ:13 */
  Default_Handler, /* IRQ:14 */
  Default_Handler, /* IRQ:15 */
  Default_Handler, /* IRQ:16 */
  Default_Handler, /* IRQ:17 */
  Default_Handler, /* IRQ:18 */
  Default_Handler, /* IRQ:19 */
  Default_Handler, /* IRQ:20 */
  Default_Handler, /* IRQ:21 */
  Default_Handler, /* IRQ:22 */
  Default_Handler, /* IRQ:23 */
  Default_Handler, /* IRQ:24 */
  Default_Handler, /* IRQ:25 */
  Default_Handler, /* IRQ:26 */
  Default_Handler, /* IRQ:27 */
  Default_Handler, /* IRQ:28 */
  Default_Handler, /* IRQ:29 */
  Default_Handler, /* IRQ:30 */
  Default_Handler, /* IRQ:31 */
  Default_Handler, /* IRQ:32 */
  Default_Handler, /* IRQ:33 */
  Default_Handler, /* IRQ:34 */
  Default_Handler, /* IRQ:35 */
  Default_Handler, /* IRQ:36 */
  Default_Handler, /* IRQ:37 */
  Default_Handler, /* IRQ:38 */
  Default_Handler, /* IRQ:39 */
  Default_Handler, /* IRQ:40 */
  Default_Handler, /* IRQ:41 */
  Default_Handler, /* IRQ:42 */
  Default_Handler, /* IRQ:43 */
  Default_Handler, /* IRQ:44 */
  Default_Handler, /* IRQ:45 */
  Default_Handler, /* IRQ:46 */
  Default_Handler, /* IRQ:47 */
  // Default_Handler, /* IRQ:48 */
  gtm_int_isr, /* IRQ:48 GTM2 INT (GTM2 Interrupt) */
  Default_Handler, /* IRQ:49 */
  Default_Handler, /* IRQ:50 */
  Default_Handler, /* IRQ:51 */
  Default_Handler, /* IRQ:52 */
  Default_Handler, /* IRQ:53 */
  Default_Handler, /* IRQ:54 */
  Default_Handler, /* IRQ:55 */
  Default_Handler, /* IRQ:56 */
  Default_Handler, /* IRQ:57 */
  Default_Handler, /* IRQ:58 */
  Default_Handler, /* IRQ:59 */
  Default_Handler, /* IRQ:60 */
  Default_Handler, /* IRQ:61 */
  Default_Handler, /* IRQ:62 */
  Default_Handler, /* IRQ:63 */
  Default_Handler, /* IRQ:64 */
  Default_Handler, /* IRQ:65 */
  Default_Handler, /* IRQ:66 */
  Default_Handler, /* IRQ:67 */
  Default_Handler, /* IRQ:68 */
  metal_irq_isr_wrapper, /* IRQ:69 MHU1 MSG_INT_NS (Non-Secure Message Interrupt ch1) */
  Default_Handler, /* IRQ:70 */
  Default_Handler, /* IRQ:71 */
  Default_Handler, /* IRQ:72 */
  Default_Handler, /* IRQ:73 */
  Default_Handler, /* IRQ:74 */
  Default_Handler, /* IRQ:75 */
  Default_Handler, /* IRQ:76 */
  Default_Handler, /* IRQ:77 */
  Default_Handler, /* IRQ:78 */
  Default_Handler, /* IRQ:79 */
  Default_Handler, /* IRQ:80 */
  Default_Handler, /* IRQ:81 */
  Default_Handler, /* IRQ:82 */
  Default_Handler, /* IRQ:83 */
  Default_Handler, /* IRQ:84 */
  Default_Handler, /* IRQ:85 */
  Default_Handler, /* IRQ:86 */
  Default_Handler, /* IRQ:87 */
  Default_Handler, /* IRQ:88 */
  Default_Handler, /* IRQ:89 */
  Default_Handler, /* IRQ:90 */
  Default_Handler, /* IRQ:91 */
  Default_Handler, /* IRQ:92 */
  Default_Handler, /* IRQ:93 */
  Default_Handler, /* IRQ:94 */
  Default_Handler, /* IRQ:95 */
  Default_Handler, /* IRQ:96 */
  Default_Handler, /* IRQ:97 */
  Default_Handler, /* IRQ:98 */
  Default_Handler, /* IRQ:99 */
  Default_Handler, /* IRQ:100 */
  Default_Handler, /* IRQ:101 */
  Default_Handler, /* IRQ:102 */
  Default_Handler, /* IRQ:103 */
  Default_Handler, /* IRQ:104 */
  Default_Handler, /* IRQ:105 */
  Default_Handler, /* IRQ:106 */
  Default_Handler, /* IRQ:107 */
  Default_Handler, /* IRQ:108 */
  Default_Handler, /* IRQ:109 */
  Default_Handler, /* IRQ:110 */
  Default_Handler, /* IRQ:111 */
  Default_Handler, /* IRQ:112 */
  Default_Handler, /* IRQ:113 */
  Default_Handler, /* IRQ:114 */
  Default_Handler, /* IRQ:115 */
  Default_Handler, /* IRQ:116 */
  Default_Handler, /* IRQ:117 */
  Default_Handler, /* IRQ:118 */
  Default_Handler, /* IRQ:119 */
  Default_Handler, /* IRQ:120 */
  Default_Handler, /* IRQ:121 */
  Default_Handler, /* IRQ:122 */
  Default_Handler, /* IRQ:123 */
  Default_Handler, /* IRQ:124 */
  Default_Handler, /* IRQ:125 */
  Default_Handler, /* IRQ:126 */
  Default_Handler, /* IRQ:127 */
  Default_Handler, /* IRQ:128 */
  Default_Handler, /* IRQ:129 */
  Default_Handler, /* IRQ:130 */
  Default_Handler, /* IRQ:131 */
  Default_Handler, /* IRQ:132 */
  Default_Handler, /* IRQ:133 */
  Default_Handler, /* IRQ:134 */
  Default_Handler, /* IRQ:135 */
  Default_Handler, /* IRQ:136 */
  Default_Handler, /* IRQ:137 */
  Default_Handler, /* IRQ:138 */
  Default_Handler, /* IRQ:139 */
  Default_Handler, /* IRQ:140 */
  Default_Handler, /* IRQ:141 */
  Default_Handler, /* IRQ:142 */
  Default_Handler, /* IRQ:143 */
  Default_Handler, /* IRQ:144 */
  Default_Handler, /* IRQ:145 */
  Default_Handler, /* IRQ:146 */
  Default_Handler, /* IRQ:147 */
  Default_Handler, /* IRQ:148 */
  Default_Handler, /* IRQ:149 */
  Default_Handler, /* IRQ:150 */
  Default_Handler, /* IRQ:151 */
  Default_Handler, /* IRQ:152 */
  Default_Handler, /* IRQ:153 */
  Default_Handler, /* IRQ:154 */
  Default_Handler, /* IRQ:155 */
  Default_Handler, /* IRQ:156 */
  Default_Handler, /* IRQ:157 */
  Default_Handler, /* IRQ:158 */
  Default_Handler, /* IRQ:159 */
  Default_Handler, /* IRQ:160 */
  Default_Handler, /* IRQ:161 */
  Default_Handler, /* IRQ:162 */
  Default_Handler, /* IRQ:163 */
  Default_Handler, /* IRQ:164 */
  Default_Handler, /* IRQ:165 */
  Default_Handler, /* IRQ:166 */
  Default_Handler, /* IRQ:167 */
  Default_Handler, /* IRQ:168 */
  Default_Handler, /* IRQ:169 */
  Default_Handler, /* IRQ:170 */
  Default_Handler, /* IRQ:171 */
  Default_Handler, /* IRQ:172 */
  Default_Handler, /* IRQ:173 */
  Default_Handler, /* IRQ:174 */
  Default_Handler, /* IRQ:175 */
  Default_Handler, /* IRQ:176 */
  Default_Handler, /* IRQ:177 */
  Default_Handler, /* IRQ:178 */
  Default_Handler, /* IRQ:179 */
  Default_Handler, /* IRQ:180 */
  Default_Handler, /* IRQ:181 */
  Default_Handler, /* IRQ:182 */
  Default_Handler, /* IRQ:183 */
  Default_Handler, /* IRQ:184 */
  Default_Handler, /* IRQ:185 */
  Default_Handler, /* IRQ:186 */
  Default_Handler, /* IRQ:187 */
  Default_Handler, /* IRQ:188 */
  Default_Handler, /* IRQ:189 */
  Default_Handler, /* IRQ:190 */
  Default_Handler, /* IRQ:191 */
  Default_Handler, /* IRQ:192 */
  Default_Handler, /* IRQ:193 */
  Default_Handler, /* IRQ:194 */
  Default_Handler, /* IRQ:195 */
  Default_Handler, /* IRQ:196 */
  Default_Handler, /* IRQ:197 */
  Default_Handler, /* IRQ:198 */
  Default_Handler, /* IRQ:199 */
  Default_Handler, /* IRQ:200 */
  Default_Handler, /* IRQ:201 */
  Default_Handler, /* IRQ:202 */
  Default_Handler, /* IRQ:203 */
  Default_Handler, /* IRQ:204 */
  Default_Handler, /* IRQ:205 */
  Default_Handler, /* IRQ:206 */
  Default_Handler, /* IRQ:207 */
  Default_Handler, /* IRQ:208 */
  Default_Handler, /* IRQ:209 */
  Default_Handler, /* IRQ:210 */
  Default_Handler, /* IRQ:211 */
  Default_Handler, /* IRQ:212 */
  Default_Handler, /* IRQ:213 */
  Default_Handler, /* IRQ:214 */
  Default_Handler, /* IRQ:215 */
  Default_Handler, /* IRQ:216 */
  Default_Handler, /* IRQ:217 */
  Default_Handler, /* IRQ:218 */
  Default_Handler, /* IRQ:219 */
  Default_Handler, /* IRQ:220 */
  Default_Handler, /* IRQ:221 */
  Default_Handler, /* IRQ:222 */
  Default_Handler, /* IRQ:223 */
  Default_Handler, /* IRQ:224 */
  Default_Handler, /* IRQ:225 */
  Default_Handler, /* IRQ:226 */
  Default_Handler, /* IRQ:227 */
  Default_Handler, /* IRQ:228 */
  Default_Handler, /* IRQ:229 */
  Default_Handler, /* IRQ:230 */
  Default_Handler, /* IRQ:231 */
  Default_Handler, /* IRQ:232 */
  Default_Handler, /* IRQ:233 */
  Default_Handler, /* IRQ:234 */
  Default_Handler, /* IRQ:235 */
  Default_Handler, /* IRQ:236 */
  Default_Handler, /* IRQ:237 */
  Default_Handler, /* IRQ:238 */
  Default_Handler, /* IRQ:239 */
  Default_Handler, /* IRQ:240 */
  Default_Handler, /* IRQ:241 */
  Default_Handler, /* IRQ:242 */
  Default_Handler, /* IRQ:243 */
  Default_Handler, /* IRQ:244 */
  Default_Handler, /* IRQ:245 */
  Default_Handler, /* IRQ:246 */
  Default_Handler, /* IRQ:247 */
  Default_Handler, /* IRQ:248 */
  Default_Handler, /* IRQ:249 */
  Default_Handler, /* IRQ:250 */
  Default_Handler, /* IRQ:251 */
  Default_Handler, /* IRQ:252 */
  Default_Handler, /* IRQ:253 */
  Default_Handler, /* IRQ:254 */
  Default_Handler, /* IRQ:255 */
  Default_Handler, /* IRQ:256 */
  Default_Handler, /* IRQ:257 */
  Default_Handler, /* IRQ:258 */
  Default_Handler, /* IRQ:259 */
  Default_Handler, /* IRQ:260 */
  Default_Handler, /* IRQ:261 */
  Default_Handler, /* IRQ:262 */
  Default_Handler, /* IRQ:263 */
  Default_Handler, /* IRQ:264 */
  Default_Handler, /* IRQ:265 */
  Default_Handler, /* IRQ:266 */
  Default_Handler, /* IRQ:267 */
  Default_Handler, /* IRQ:268 */
  Default_Handler, /* IRQ:269 */
  Default_Handler, /* IRQ:270 */
  Default_Handler, /* IRQ:271 */
  Default_Handler, /* IRQ:272 */
  Default_Handler, /* IRQ:273 */
  Default_Handler, /* IRQ:274 */
  Default_Handler, /* IRQ:275 */
  Default_Handler, /* IRQ:276 */
  Default_Handler, /* IRQ:277 */
  Default_Handler, /* IRQ:278 */
  Default_Handler, /* IRQ:279 */
  Default_Handler, /* IRQ:280 */
  Default_Handler, /* IRQ:281 */
  Default_Handler, /* IRQ:282 */
  Default_Handler, /* IRQ:283 */
  Default_Handler, /* IRQ:284 */
  Default_Handler, /* IRQ:285 */
  Default_Handler, /* IRQ:286 */
  Default_Handler, /* IRQ:287 */
  Default_Handler, /* IRQ:288 */
  Default_Handler, /* IRQ:289 */
  Default_Handler, /* IRQ:290 */
  Default_Handler, /* IRQ:291 */
  Default_Handler, /* IRQ:292 */
  Default_Handler, /* IRQ:293 */
  Default_Handler, /* IRQ:294 */
  Default_Handler, /* IRQ:295 */
  Default_Handler, /* IRQ:296 */
  Default_Handler, /* IRQ:297 */
  Default_Handler, /* IRQ:298 */
  Default_Handler, /* IRQ:299 */
  Default_Handler, /* IRQ:300 */
  Default_Handler, /* IRQ:301 */
  Default_Handler, /* IRQ:302 */
  Default_Handler, /* IRQ:303 */
  Default_Handler, /* IRQ:304 */
  Default_Handler, /* IRQ:305 */
  Default_Handler, /* IRQ:306 */
  Default_Handler, /* IRQ:307 */
  Default_Handler, /* IRQ:308 */
  Default_Handler, /* IRQ:309 */
  Default_Handler, /* IRQ:310 */
  Default_Handler, /* IRQ:311 */
  Default_Handler, /* IRQ:312 */
  Default_Handler, /* IRQ:313 */
  Default_Handler, /* IRQ:314 */
  Default_Handler, /* IRQ:315 */
  Default_Handler, /* IRQ:316 */
  Default_Handler, /* IRQ:317 */
  Default_Handler, /* IRQ:318 */
  Default_Handler, /* IRQ:319 */
  Default_Handler, /* IRQ:320 */
  Default_Handler, /* IRQ:321 */
  Default_Handler, /* IRQ:322 */
  Default_Handler, /* IRQ:323 */
  Default_Handler, /* IRQ:324 */
  Default_Handler, /* IRQ:325 */
  Default_Handler, /* IRQ:326 */
  Default_Handler, /* IRQ:327 */
  Default_Handler, /* IRQ:328 */
  Default_Handler, /* IRQ:329 */
  Default_Handler, /* IRQ:330 */
  Default_Handler, /* IRQ:331 */
  Default_Handler, /* IRQ:332 */
  Default_Handler, /* IRQ:333 */
  Default_Handler, /* IRQ:334 */
  Default_Handler, /* IRQ:335 */
  Default_Handler, /* IRQ:336 */
  Default_Handler, /* IRQ:337 */
  Default_Handler, /* IRQ:338 */
  Default_Handler, /* IRQ:339 */
  Default_Handler, /* IRQ:340 */
  Default_Handler, /* IRQ:341 */
  Default_Handler, /* IRQ:342 */
  Default_Handler, /* IRQ:343 */
  Default_Handler, /* IRQ:344 */
  Default_Handler, /* IRQ:345 */
  Default_Handler, /* IRQ:346 */
  Default_Handler, /* IRQ:347 */
  Default_Handler, /* IRQ:348 */
  Default_Handler, /* IRQ:349 */
  Default_Handler, /* IRQ:350 */
  Default_Handler, /* IRQ:351 */
  Default_Handler, /* IRQ:352 */
  Default_Handler, /* IRQ:353 */
  Default_Handler, /* IRQ:354 */
  Default_Handler, /* IRQ:355 */
  Default_Handler, /* IRQ:356 */
  Default_Handler, /* IRQ:357 */
  Default_Handler, /* IRQ:358 */
  Default_Handler, /* IRQ:359 */
  Default_Handler, /* IRQ:360 */
  Default_Handler, /* IRQ:361 */
  Default_Handler, /* IRQ:362 */
  Default_Handler, /* IRQ:363 */
  Default_Handler, /* IRQ:364 */
  Default_Handler, /* IRQ:365 */
  Default_Handler, /* IRQ:366 */
  Default_Handler, /* IRQ:367 */
  Default_Handler, /* IRQ:368 */
  Default_Handler, /* IRQ:369 */
  Default_Handler, /* IRQ:370 */
  Default_Handler, /* IRQ:371 */
  Default_Handler, /* IRQ:372 */
  Default_Handler, /* IRQ:373 */
  Default_Handler, /* IRQ:374 */
  Default_Handler, /* IRQ:375 */
  Default_Handler, /* IRQ:376 */
  Default_Handler, /* IRQ:377 */
  Default_Handler, /* IRQ:378 */
  Default_Handler, /* IRQ:379 */
  Default_Handler, /* IRQ:380 */
  Default_Handler, /* IRQ:381 */
  Default_Handler, /* IRQ:382 */
  Default_Handler, /* IRQ:383 */
  Default_Handler, /* IRQ:384 */
  Default_Handler, /* IRQ:385 */
  Default_Handler, /* IRQ:386 */
  Default_Handler, /* IRQ:387 */
  Default_Handler, /* IRQ:388 */
  Default_Handler, /* IRQ:389 */
  scif_uart_eri_isr, /* IRQ:390 SCIF2 ERI (Framing or Parity error) */
  scif_uart_bri_isr, /* IRQ:391 SCIF2 BRI (Break (BRK) or Overrun (ORER) error) */
  scif_uart_rxi_isr, /* IRQ:392 SCIF2 RXI (Receive data full) */
  scif_uart_txi_isr, /* IRQ:393 SCIF2 TXI (Transmit data empty) */
  scif_uart_tei_isr, /* IRQ:394 SCIF2 TEI (Transmit end) */
  Default_Handler, /* IRQ:395 */
  Default_Handler, /* IRQ:396 */
  Default_Handler, /* IRQ:397 */
  Default_Handler, /* IRQ:398 */
  Default_Handler, /* IRQ:399 */
  Default_Handler, /* IRQ:400 */
  Default_Handler, /* IRQ:401 */
  Default_Handler, /* IRQ:402 */
  Default_Handler, /* IRQ:403 */
  Default_Handler, /* IRQ:404 */
  Default_Handler, /* IRQ:405 */
  Default_Handler, /* IRQ:406 */
  Default_Handler, /* IRQ:407 */
  Default_Handler, /* IRQ:408 */
  Default_Handler, /* IRQ:409 */
  Default_Handler, /* IRQ:410 */
  Default_Handler, /* IRQ:411 */
  Default_Handler, /* IRQ:412 */
  Default_Handler, /* IRQ:413 */
  Default_Handler, /* IRQ:414 */
  Default_Handler, /* IRQ:415 */
  Default_Handler, /* IRQ:416 */
  Default_Handler, /* IRQ:417 */
  Default_Handler, /* IRQ:418 */
  Default_Handler, /* IRQ:419 */
  Default_Handler, /* IRQ:420 */
  Default_Handler, /* IRQ:421 */
  Default_Handler, /* IRQ:422 */
  Default_Handler, /* IRQ:423 */
  Default_Handler, /* IRQ:424 */
  Default_Handler, /* IRQ:425 */
  Default_Handler, /* IRQ:426 */
  Default_Handler, /* IRQ:427 */
  Default_Handler, /* IRQ:428 */
  Default_Handler, /* IRQ:429 */
  Default_Handler, /* IRQ:430 */
  Default_Handler, /* IRQ:431 */
  Default_Handler, /* IRQ:432 */
  Default_Handler, /* IRQ:433 */
  Default_Handler, /* IRQ:434 */
  Default_Handler, /* IRQ:435 */
  Default_Handler, /* IRQ:436 */
  Default_Handler, /* IRQ:437 */
  Default_Handler, /* IRQ:438 */
  Default_Handler, /* IRQ:439 */
  Default_Handler, /* IRQ:440 */
  Default_Handler, /* IRQ:441 */
  Default_Handler, /* IRQ:442 */
  Default_Handler, /* IRQ:443 */
  Default_Handler, /* IRQ:444 */
  Default_Handler, /* IRQ:445 */
  Default_Handler, /* IRQ:446 */
  Default_Handler, /* IRQ:447 */
  Default_Handler, /* IRQ:448 */
  Default_Handler, /* IRQ:449 */
  Default_Handler, /* IRQ:450 */
  Default_Handler, /* IRQ:451 */
  Default_Handler, /* IRQ:452 */
  Default_Handler, /* IRQ:453 */
  Default_Handler, /* IRQ:454 */
  Default_Handler, /* IRQ:455 */
  Default_Handler, /* IRQ:456 */
  Default_Handler, /* IRQ:457 */
  Default_Handler, /* IRQ:458 */
  Default_Handler, /* IRQ:459 */
  Default_Handler, /* IRQ:460 */
  Default_Handler, /* IRQ:461 */
  Default_Handler, /* IRQ:462 */
  Default_Handler, /* IRQ:463 */
  Default_Handler, /* IRQ:464 */
  Default_Handler, /* IRQ:465 */
  Default_Handler, /* IRQ:466 */
  Default_Handler, /* IRQ:467 */
  Default_Handler, /* IRQ:468 */
  Default_Handler, /* IRQ:469 */
  Default_Handler, /* IRQ:470 */
  Default_Handler, /* IRQ:471 */
  Default_Handler, /* IRQ:472 */
  Default_Handler, /* IRQ:473 */
  Default_Handler, /* IRQ:474 */
  Default_Handler, /* IRQ:475 */
  Default_Handler, /* IRQ:476 */
  Default_Handler, /* IRQ:477 */
  Default_Handler, /* IRQ:478 */
  Default_Handler, /* IRQ:479 */
};

/* IRQ No:0,   NMI */
/* IRQ No:1,   IRQ0 */
/* IRQ No:2,   IRQ1 */
/* IRQ No:3,   IRQ2 */
/* IRQ No:4,   IRQ3 */
/* IRQ No:5,   IRQ4 */
/* IRQ No:6,   IRQ5 */
/* IRQ No:7,   IRQ6 */
/* IRQ No:8,   IRQ7 */
/* IRQ No:9,   nCOMMIRQ[1] */
/* IRQ No:10,  nCOMMIRQ[0] */
/* IRQ No:11,  nPMUIRQ[1] */
/* IRQ No:12,  nPMUIRQ[0] */
/* IRQ No:13,  nCLUSTERPMUIRQ */
/* IRQ No:14,  nERRIRQ[2] */
/* IRQ No:15,  nERRIRQ[1] */
/* IRQ No:16,  nERRIRQ[0] */
/* IRQ No:17,  nFAULTIRQ[2] */
/* IRQ No:18,  nFAULTIRQ[1] */
/* IRQ No:19,  nFAULTIRQ[0] */
/* IRQ No:20,  CTIIRQ[1] */
/* IRQ No:21,  CTIIRQ[0] */
/* IRQ No:22,  fault_int */
/* IRQ No:23,  err_int */
/* IRQ No:24,  pmu_int */
/* IRQ No:25,  BUS_ERR_INT */
/* IRQ No:26,  TZC0INT */
/* IRQ No:27,  TZC1INT */
/* IRQ No:28,  TZC2INT */
/* IRQ No:29,  TZC3INT */
/* IRQ No:30,  ERRINT_S */
/* IRQ No:31,  ERRINT_C */
/* IRQ No:32,  Reserved */
/* IRQ No:33,  Reserved */
/* IRQ No:34,  EC7TIE1_0 */
/* IRQ No:35,  EC7TIE2_0 */
/* IRQ No:36,  EC7TIOVF_0 */
/* IRQ No:37,  EC7TIE1_1 */
/* IRQ No:38,  EC7TIE2_1 */
/* IRQ No:39,  EC7TIOVF_1 */
/* IRQ No:40,  controller_int */
/* IRQ No:41,  SPIHF */
/* IRQ No:42,  SYS_LPM_INT */
/* IRQ No:43,  SYS_CA55STBYDONE_INT */
/* IRQ No:44,  SYS_CM33STBYR_INT */
/* IRQ No:45,  SYS_CA55_DENY */
/* IRQ No:46,  OSTM0TINT */
/* IRQ No:47,  OSTM1TINT */
/* IRQ No:48,  OSTM2TINT */
/* IRQ No:49,  WDTINT_A0 */
/* IRQ No:50,  PERROUT_A0 */
/* IRQ No:51,  WDTINT_A1 */
/* IRQ No:52,  PERROUT_A1 */
/* IRQ No:53,  WDTINT_M */
/* IRQ No:54,  PERROUT_M */
/* IRQ No:55,  Reserved */
/* IRQ No:56,  msg_ch0_s */
/* IRQ No:57,  msg_ch1_s */
/* IRQ No:58,  msg_ch2_s */
/* IRQ No:59,  msg_ch3_s */
/* IRQ No:60,  msg_ch4_s */
/* IRQ No:61,  msg_ch5_s */
/* IRQ No:62,  rsp_ch0_s */
/* IRQ No:63,  rsp_ch1_s */
/* IRQ No:64,  rsp_ch2_s */
/* IRQ No:65,  rsp_ch3_s */
/* IRQ No:66,  rsp_ch4_s */
/* IRQ No:67,  rsp_ch5_s */
/* IRQ No:68,  msg_ch0_ns */
/* IRQ No:69,  msg_ch1_ns */
/* IRQ No:70,  msg_ch2_ns */
/* IRQ No:71,  msg_ch3_ns */
/* IRQ No:72,  msg_ch4_ns */
/* IRQ No:73,  msg_ch5_ns */
/* IRQ No:74,  rsp_ch0_ns */
/* IRQ No:75,  rsp_ch1_ns */
/* IRQ No:76,  rsp_ch2_ns */
/* IRQ No:77,  rsp_ch3_ns */
/* IRQ No:78,  rsp_ch4_ns */
/* IRQ No:79,  rsp_ch5_ns */
/* IRQ No:80,  sw_mhu_int_0 */
/* IRQ No:81,  sw_mhu_int_1 */
/* IRQ No:82,  sw_mhu_int_2 */
/* IRQ No:83,  sw_mhu_int_3 */
/* IRQ No:84,  pif_int_n_ch0 */
/* IRQ No:85,  int_fil_n_ch0 */
/* IRQ No:86,  int_arp_ns_n_ch0 */
/* IRQ No:87,  pif_int_n_ch1 */
/* IRQ No:88,  int_fil_n_ch1 */
/* IRQ No:89,  int_arp_ns_n_ch1 */
/* IRQ No:90,  U2H0_INT */
/* IRQ No:91,  U2H0_OHCI_INT */
/* IRQ No:92,  U2H0_EHCI_INT */
/* IRQ No:93,  U2H0_WAKEON_INT */
/* IRQ No:94,  U2H0_OBINT */
/* IRQ No:95,  U2H1_INT */
/* IRQ No:96,  U2H1_OHCI_INT */
/* IRQ No:97,  U2H1_EHCI_INT */
/* IRQ No:98,  U2H1_WAKEON_INT */
/* IRQ No:99,  U2H1_OBINT */
/* IRQ No:100, U2P_IXL_INT */
/* IRQ No:101, U2P_INT_DMA[0] */
/* IRQ No:102, U2P_INT_DMA[1] */
/* IRQ No:103, U2P_INT_DMAERR */
/* IRQ No:104, OXMNIRQ0 */
/* IRQ No:105, OXASIOIRQ0 */
/* IRQ No:106, OXMNIRQ1 */
/* IRQ No:107, OXASIOIRQ1 */
/* IRQ No:108, DMAINT0_S */
/* IRQ No:109, DMAINT1_S */
/* IRQ No:110, DMAINT2_S */
/* IRQ No:111, DMAINT3_S */
/* IRQ No:112, DMAINT4_S */
/* IRQ No:113, DMAINT5_S */
/* IRQ No:114, DMAINT6_S */
/* IRQ No:115, DMAINT7_S */
/* IRQ No:116, DMAINT8_S */
/* IRQ No:117, DMAINT9_S */
/* IRQ No:118, DMAINT10_S */
/* IRQ No:119, DMAINT11_S */
/* IRQ No:120, DMAINT12_S */
/* IRQ No:121, DMAINT13_S */
/* IRQ No:122, DMAINT14_S */
/* IRQ No:123, DMAINT15_S */
/* IRQ No:124, DMAERR_S */
/* IRQ No:125, DMAINT0_NS */
/* IRQ No:126, DMAINT1_NS */
/* IRQ No:127, DMAINT2_NS */
/* IRQ No:128, DMAINT3_NS */
/* IRQ No:129, DMAINT4_NS */
/* IRQ No:130, DMAINT5_NS */
/* IRQ No:131, DMAINT6_NS */
/* IRQ No:132, DMAINT7_NS */
/* IRQ No:133, DMAINT8_NS */
/* IRQ No:134, DMAINT9_NS */
/* IRQ No:135, DMAINT10_NS */
/* IRQ No:136, DMAINT11_NS */
/* IRQ No:137, DMAINT12_NS */
/* IRQ No:138, DMAINT13_NS */
/* IRQ No:139, DMAINT14_NS */
/* IRQ No:140, DMAINT15_NS */
/* IRQ No:141, DMAERR_NS */
/* IRQ No:142, dsi_int_seq0 */
/* IRQ No:143, dsi_int_seq1 */
/* IRQ No:144, dsi_int_vin1 */
/* IRQ No:145, dsi_int_rcv */
/* IRQ No:146, dsi_int_ferr */
/* IRQ No:147, dsi_int_ppi */
/* IRQ No:148, Reserved */
/* IRQ No:149, VSPD_INT */
/* IRQ No:150, Reserved */
/* IRQ No:151, Reserved */
/* IRQ No:152, DU_INT */
/* IRQ No:153, IRQGPU */
/* IRQ No:154, IRQJOB */
/* IRQ No:155, IRQMMU */
/* IRQ No:156, IRQEVENT */
/* IRQ No:157, ISU_INT_FRE */
/* IRQ No:158, ISU_INT_DESE */
/* IRQ No:159, ISU_INT_STOPE */
/* IRQ No:160, ISU_INT_ERR */
/* IRQ No:161, VCP.vcpl4.vint */
/* IRQ No:162, VCP.vcpl4.cint */
/* IRQ No:163, VCP.vcpl4.vedcint */
/* IRQ No:164, VCP.vcpl4.cedcint */
/* IRQ No:165, VCP.fcpcs.edcint */
/* IRQ No:166, csi2_link_int  */
/* IRQ No:167, image_conv_int */
/* IRQ No:168, image_conv_err_int */
/* IRQ No:169, axi_mst_err_int */
/* IRQ No:170, TGIA0 */
/* IRQ No:171, TGIB0 */
/* IRQ No:172, TGIC0 */
/* IRQ No:173, TGID0 */
/* IRQ No:174, TCIV0 */
/* IRQ No:175, TGIE0 */
/* IRQ No:176, TGIF0 */
/* IRQ No:177, TGIA1 */
/* IRQ No:178, TGIB1 */
/* IRQ No:179, TCIV1 */
/* IRQ No:180, TCIU1 */
/* IRQ No:181, TGIA2 */
/* IRQ No:182, TGIB2 */
/* IRQ No:183, TCIV2 */
/* IRQ No:184, TCIU2 */
/* IRQ No:185, TGIA3 */
/* IRQ No:186, TGIB3 */
/* IRQ No:187, TGIC3 */
/* IRQ No:188, TGID3 */
/* IRQ No:189, TCIV3 */
/* IRQ No:190, TGIA4 */
/* IRQ No:191, TGIB4 */
/* IRQ No:192, TGIC4 */
/* IRQ No:193, TGID4 */
/* IRQ No:194, TCIV4 */
/* IRQ No:195, TGIU5 */
/* IRQ No:196, TGIV5 */
/* IRQ No:197, TGIW5 */
/* IRQ No:198, TGIA6 */
/* IRQ No:199, TGIB6 */
/* IRQ No:200, TGIC6 */
/* IRQ No:201, TGID6 */
/* IRQ No:202, TCIV6 */
/* IRQ No:203, TGIA7 */
/* IRQ No:204, TGIB7 */
/* IRQ No:205, TGIC7 */
/* IRQ No:206, TGID7 */
/* IRQ No:207, TCIV7 */
/* IRQ No:208, TGIA8 */
/* IRQ No:209, TGIB8 */
/* IRQ No:210, TGIC8 */
/* IRQ No:211, TGID8 */
/* IRQ No:212, TCIV8 */
/* IRQ No:213, TCIU8 */
/* IRQ No:214, OEI1 */
/* IRQ No:215, OEI2 */
/* IRQ No:216, OEI3 */
/* IRQ No:217, OEI4 */
/* IRQ No:218, CCMPA0 */
/* IRQ No:219, CCMPB0 */
/* IRQ No:220, CMPC0 */
/* IRQ No:221, CMPD0 */
/* IRQ No:222, CMPE0 */
/* IRQ No:223, CMPF0 */
/* IRQ No:224, ADTRGA0 */
/* IRQ No:225, ADTRGB0 */
/* IRQ No:226, OVF0 */
/* IRQ No:227, UNF0 */
/* IRQ No:228, Reserved */
/* IRQ No:229, Reserved */
/* IRQ No:230, Reserved */
/* IRQ No:231, CCMPA1 */
/* IRQ No:232, CCMPB1 */
/* IRQ No:233, CMPC1 */
/* IRQ No:234, CMPD1 */
/* IRQ No:235, CMPE1 */
/* IRQ No:236, CMPF1 */
/* IRQ No:237, ADTRGA1 */
/* IRQ No:238, ADTRGB1 */
/* IRQ No:239, OVF1 */
/* IRQ No:240, UNF1 */
/* IRQ No:241, Reserved */
/* IRQ No:242, Reserved */
/* IRQ No:243, Reserved */
/* IRQ No:244, CCMPA2 */
/* IRQ No:245, CCMPB2 */
/* IRQ No:246, CMPC2 */
/* IRQ No:247, CMPD2 */
/* IRQ No:248, CMPE2 */
/* IRQ No:249, CMPF2 */
/* IRQ No:250, ADTRGA2 */
/* IRQ No:251, ADTRGB2 */
/* IRQ No:252, OVF2 */
/* IRQ No:253, UNF2 */
/* IRQ No:254, Reserved */
/* IRQ No:255, Reserved */
/* IRQ No:256, Reserved */
/* IRQ No:257, CCMPA3 */
/* IRQ No:258, CCMPB3 */
/* IRQ No:259, CMPC3 */
/* IRQ No:260, CMPD3 */
/* IRQ No:261, CMPE3 */
/* IRQ No:262, CMPF3 */
/* IRQ No:263, ADTRGA3 */
/* IRQ No:264, ADTRGB3 */
/* IRQ No:265, OVF3 */
/* IRQ No:266, UNF3 */
/* IRQ No:267, Reserved */
/* IRQ No:268, Reserved */
/* IRQ No:269, Reserved */
/* IRQ No:270, CCMPA4 */
/* IRQ No:271, CCMPB4 */
/* IRQ No:272, CMPC4 */
/* IRQ No:273, CMPD4 */
/* IRQ No:274, CMPE4 */
/* IRQ No:275, CMPF4 */
/* IRQ No:276, ADTRGA4 */
/* IRQ No:277, ADTRGB4 */
/* IRQ No:278, OVF4 */
/* IRQ No:279, UNF4 */
/* IRQ No:280, Reserved */
/* IRQ No:281, Reserved */
/* IRQ No:282, Reserved */
/* IRQ No:283, CCMPA5 */
/* IRQ No:284, CCMPB5 */
/* IRQ No:285, CMPC5 */
/* IRQ No:286, CMPD5 */
/* IRQ No:287, CMPE5 */
/* IRQ No:288, CMPF5 */
/* IRQ No:289, ADTRGA5 */
/* IRQ No:290, ADTRGB5 */
/* IRQ No:291, OVF5 */
/* IRQ No:292, UNF5 */
/* IRQ No:293, Reserved */
/* IRQ No:294, Reserved */
/* IRQ No:295, Reserved */
/* IRQ No:296, CCMPA6 */
/* IRQ No:297, CCMPB6 */
/* IRQ No:298, CMPC6 */
/* IRQ No:299, CMPD6 */
/* IRQ No:300, CMPE6 */
/* IRQ No:301, CMPF6 */
/* IRQ No:302, ADTRGA6 */
/* IRQ No:303, ADTRGB6 */
/* IRQ No:304, OVF6 */
/* IRQ No:305, UNF6 */
/* IRQ No:306, Reserved */
/* IRQ No:307, Reserved */
/* IRQ No:308, Reserved */
/* IRQ No:309, CCMPA7 */
/* IRQ No:310, CCMPB7 */
/* IRQ No:311, CMPC7 */
/* IRQ No:312, CMPD7 */
/* IRQ No:313, CMPE7 */
/* IRQ No:314, CMPF7 */
/* IRQ No:315, ADTRGA7 */
/* IRQ No:316, ADTRGB7 */
/* IRQ No:317, OVF7 */
/* IRQ No:318, UNF7 */
/* IRQ No:319, Reserved */
/* IRQ No:320, Reserved */
/* IRQ No:321, Reserved */
/* IRQ No:322, GROUP0 */
/* IRQ No:323, GROUP1 */
/* IRQ No:324, GROUP2 */
/* IRQ No:325, GROUP3 */
/* IRQ No:326, INT_ssif_int_req_0 */
/* IRQ No:327, INT_ssif_dma_rx_0 */
/* IRQ No:328, INT_ssif_dma_tx_0 */
/* IRQ No:329, INT_ssif_dma_rt_0 */
/* IRQ No:330, INT_ssif_int_req_1 */
/* IRQ No:331, INT_ssif_dma_rx_1 */
/* IRQ No:332, INT_ssif_dma_tx_1 */
/* IRQ No:333, INT_ssif_dma_rt_1 */
/* IRQ No:334, INT_ssif_int_req_2 */
/* IRQ No:335, INT_ssif_dma_rx_2 */
/* IRQ No:336, INT_ssif_dma_tx_2 */
/* IRQ No:337, INT_ssif_dma_rt_2 */
/* IRQ No:338, INT_ssif_int_req_3 */
/* IRQ No:339, INT_ssif_dma_rx_3 */
/* IRQ No:340, INT_ssif_dma_tx_3 */
/* IRQ No:341, INT_ssif_dma_rt_3 */
/* IRQ No:342, SRC_IDEI */
/* IRQ No:343, SRC_ODFI */
/* IRQ No:344, SRC_CEF */
/* IRQ No:345, SRC_UDF */
/* IRQ No:346, SRC_OVF */
/* IRQ No:347, INTAD */
/* IRQ No:348, INTRIICRI0 */
/* IRQ No:349, INTRIICTI0 */
/* IRQ No:350, INTRIICTEI0 */
/* IRQ No:351, INTRIICNAKI0 */
/* IRQ No:352, INTRIICSPI0 */
/* IRQ No:353, INTRIICSTI0 */
/* IRQ No:354, INTRIICALI0 */
/* IRQ No:355, INTRIICTMOI0 */
/* IRQ No:356, INTRIICRI1 */
/* IRQ No:357, INTRIICTI1 */
/* IRQ No:358, INTRIICTEI1 */
/* IRQ No:359, INTRIICNAKI1 */
/* IRQ No:360, INTRIICSPI1 */
/* IRQ No:361, INTRIICSTI1 */
/* IRQ No:362, INTRIICALI1 */
/* IRQ No:363, INTRIICTMOI1 */
/* IRQ No:364, INTRIICRI2 */
/* IRQ No:365, INTRIICTI2 */
/* IRQ No:366, INTRIICTEI2 */
/* IRQ No:367, INTRIICNAKI2 */
/* IRQ No:368, INTRIICSPI2 */
/* IRQ No:369, INTRIICSTI2 */
/* IRQ No:370, INTRIICALI2 */
/* IRQ No:371, INTRIICTMOI2 */
/* IRQ No:372, INTRIICRI3 */
/* IRQ No:373, INTRIICTI3 */
/* IRQ No:374, INTRIICTEI3 */
/* IRQ No:375, INTRIICNAKI3 */
/* IRQ No:376, INTRIICSPI3 */
/* IRQ No:377, INTRIICSTI3 */
/* IRQ No:378, INTRIICALI3 */
/* IRQ No:379, INTRIICTMOI3 */
/* IRQ No:380, ERI0 */
/* IRQ No:381, BRI0 */
/* IRQ No:382, RXI0 */
/* IRQ No:383, TXI0 */
/* IRQ No:384, TEI0_DRI0 */
/* IRQ No:385, ERI1 */
/* IRQ No:386, BRI1 */
/* IRQ No:387, RXI1 */
/* IRQ No:388, TXI1 */
/* IRQ No:389, TEI1_DRI1 */
/* IRQ No:390, ERI2 */
/* IRQ No:391, BRI2 */
/* IRQ No:392, RXI2 */
/* IRQ No:393, TXI2 */
/* IRQ No:394, TEI2_DRI2 */
/* IRQ No:395, ERI3 */
/* IRQ No:396, BRI3 */
/* IRQ No:397, RXI3 */
/* IRQ No:398, TXI3 */
/* IRQ No:399, TEI3_DRI3 */
/* IRQ No:400, ERI4 */
/* IRQ No:401, BRI4 */
/* IRQ No:402, RXI4 */
/* IRQ No:403, TXI4 */
/* IRQ No:404, TEI4_DRI4 */
/* IRQ No:405, ERI0 */
/* IRQ No:406, RXI0 */
/* IRQ No:407, TXI0 */
/* IRQ No:408, TEI0 */
/* IRQ No:409, ERI1 */
/* IRQ No:410, RXI1 */
/* IRQ No:411, TXI1 */
/* IRQ No:412, TEI1 */
/* IRQ No:413, SPRI0 */
/* IRQ No:414, SPTI0 */
/* IRQ No:415, SPEI0 */
/* IRQ No:416, SPRI1 */
/* IRQ No:417, SPTI1 */
/* IRQ No:418, SPEI1 */
/* IRQ No:419, SPRI2 */
/* IRQ No:420, SPTI2 */
/* IRQ No:421, SPEI2 */
/* IRQ No:422, INTRCAN0ERR */
/* IRQ No:423, INTRCAN1ERR */
/* IRQ No:424, INTRCAN0REC */
/* IRQ No:425, INTRCAN1REC */
/* IRQ No:426, INTRCANGERR */
/* IRQ No:427, INTRCANGRECC */
/* IRQ No:428, INTRCAN0TRX */
/* IRQ No:429, INTRCAN1TRX */
/* IRQ No:430, WRRDY1 */
/* IRQ No:431, WRRDY0 */
/* IRQ No:432, WRRDY4 */
/* IRQ No:433, RDRDY1 */
/* IRQ No:434, RDRDY0 */
/* IRQ No:435, ROMOK */
/* IRQ No:436, LONG_PLG */
/* IRQ No:437, PROC_BUSY */
/* IRQ No:438, IRDRDY */
/* IRQ No:439, IWRRDY */
/* IRQ No:440, Reserved */
/* IRQ No:441, Reserved */
/* IRQ No:442, Reserved */
/* IRQ No:443, Reserved */
/* IRQ No:444, TINT0 */
/* IRQ No:445, TINT1 */
/* IRQ No:446, TINT2 */
/* IRQ No:447, TINT3 */
/* IRQ No:448, TINT4 */
/* IRQ No:449, TINT5 */
/* IRQ No:450, TINT6 */
/* IRQ No:451, TINT7 */
/* IRQ No:452, TINT8 */
/* IRQ No:453, TINT9 */
/* IRQ No:454, TINT10 */
/* IRQ No:455, TINT11 */
/* IRQ No:456, TINT12 */
/* IRQ No:457, TINT13 */
/* IRQ No:458, TINT14 */
/* IRQ No:459, TINT15 */
/* IRQ No:460, TINT16 */
/* IRQ No:461, TINT17 */
/* IRQ No:462, TINT18 */
/* IRQ No:463, TINT19 */
/* IRQ No:464, TINT20 */
/* IRQ No:465, TINT21 */
/* IRQ No:466, TINT22 */
/* IRQ No:467, TINT23 */
/* IRQ No:468, TINT24 */
/* IRQ No:469, TINT25 */
/* IRQ No:470, TINT26 */
/* IRQ No:471, TINT27 */
/* IRQ No:472, TINT28 */
/* IRQ No:473, TINT29 */
/* IRQ No:474, TINT30 */
/* IRQ No:475, TINT31 */
/* IRQ No:476, Reserved */
/* IRQ No:477, Reserved */
/* IRQ No:478, Reserved */
/* IRQ No:479, Reserved */
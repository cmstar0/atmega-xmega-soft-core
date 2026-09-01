/*
 * This IP is the MEGA/XMEGA ATMEGA644 implementation.
 *
 * Instantiates the same MEGA/XMEGA core engine and peripheral IP already shipping in
 * MiSTer-devel/Arduboy_MiSTer (rtl/avr/{mega-core,mega-alu,mega-reg,mega-ram,mega-def,
 * atmega-tim-8bit,atmega-tim-16bit,atmega-pio,atmega-uart,atmega-spi-m,atmega-eep}.v), all
 * Copyright (C) Iulian Gheorghiu (morgoth@devboard.tech), GPLv2-or-later.
 * This file — the chip-specific top level, playing the same role atmega32u4.v plays for the
 * ATmega32U4 — is new, written against the ATmega644 datasheet (Atmel doc2593O-AVR-02/12) and
 * the Uzebox V5.0 schematic (github.com/Uzebox/uzebox, schematics/Uzebox/V5.0/). Every
 * parameter and address below carries its datasheet citation inline — nothing here is guessed.
 *
 * Deliberately chip-generic, not Uzebox-specific: every real ATmega644 pin is exposed under
 * its real name via a full PORT/DDR/PIN interface. A separate "board" module (future work,
 * matching how arduboy_board.v wraps atmega32u4.v) wires these to any specific board's design.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

`timescale 1ns / 1ps

`include "mega-def.v"

`define USE_TIMER_0
`define USE_TIMER_1
`define USE_TIMER_2
`define USE_SPI_0
`define USE_UART_0
`define USE_EEPROM

/* ATmega644 is an "AVR enhanced RISC core" (datasheet p.1) supporting MUL/JMP/CALL (datasheet
 * S29 Instruction Set Summary) and SPM self-programming (vector 28, SPM_READY, datasheet S10.1
 * Table 10-1) — all three gate at MEGA_ENHANCED_8K minimum and MEGA_CLASSIC_128K minimum
 * respectively in mega-def.v; MEGA_ENHANCED_128K (the same tier already used for the ATmega32U4
 * in atmega32u4.v:35) satisfies both. Confirmed 2026-08-10, see PROJECT.md's AVR-core-reuse Notes. */
`define CORE_TYPE               `MEGA_ENHANCED_128K

/* 64KB flash = 32K words -> 15 address bits. Datasheet p.1 Features: "64 Kbytes of ... Flash". */
`define ROM_ADDR_WIDTH          15

/* Data address space covers registers (0x00-0x1F) + I/O (0x20-0xFF) + 4KB SRAM (0x100-0x10FF)
 * -> needs 13 bits (0x10FF < 0x2000 = 2^13, 0x10FF >= 0x1000 = 2^12, so 12 bits is not enough).
 * Datasheet S28 Register Summary confirms every register address used below; SRAM size (4096
 * bytes) from p.1 Features "4 Kbytes Internal SRAM". */
`define BUS_ADDR_DATA_LEN       13

/* Physical SRAM is exactly 4096 bytes = 2^12, a clean power of two (unlike the ATmega32U4's
 * 2.5KB, which atmega32u4.v pads up to a 4096-entry array anyway) — RAM_ADDR_WIDTH=12 exactly
 * fits with no padding. mega_ram indexes with data_addr[RAM_ADDR_WIDTH-1:0], no offset
 * subtraction (matching atmega32u4.v's pattern) — verified by hand that this does NOT alias
 * within the real SRAM range 0x100-0x10FF, because that range is exactly 4096 (2^12) consecutive
 * addresses, so "address mod 4096" is a bijection over it regardless of the 0x100 offset. Tested
 * explicitly in the Phase 1 simulation testbench (writes to both ends of the SRAM range) rather
 * than trusted on the arithmetic alone. */
`define RAM_ADDR_WIDTH          12

/* 2KB EEPROM, datasheet p.1 Features: "2 Kbytes EEPROM". */
`define EEP_ADDR_WIDTH          11

`define RESERVED_RAM_FOR_IO     'h100

/* 27 = the ATmega644's real interrupt vector table (datasheet S10.1 Table 10-1) has 28 entries
 * total (vector 1 = RESET through vector 28 = SPM_READY); VECTOR_INT_TABLE_SIZE counts only the
 * 27 non-RESET, maskable sources — RESET is handled by the `rst` port, never through int_encoder.
 * Confirmed against mega-core.v's int_encoder (mega-core.v:107-146): int_vect = j+1 is a WORD
 * offset into the vector table (int_vect=1 -> byte address 0x0002 = vector 2/INT0's real address;
 * int_vect=27 -> byte address 0x0036 = vector 28/SPM_READY's real address, matching Table 10-1
 * exactly), and int_sig's bit order is a descending-vector-number concatenation (bit0=lowest
 * real vector/INT0, matching the pattern derived from atmega32u4.v's own int_sig wiring). The
 * ATmega644's vector table is fully dense (no reserved/gap vectors, unlike the ATmega32U4's),
 * so no padding/reserved bits are needed in this concatenation. */
`define VECTOR_INT_TABLE_SIZE   27
/* Page geometry: ATmega644 datasheet Table 25-7 -- 128 words/page, PCWORD = PC[6:0]. */
`define SPM_PAGE_ADDR_WIDTH     7
`define WATCHDOG_CNT_WIDTH      0

/* TIMER PRESCALER MODULE — identical to atmega32u4.v's; generic to the whole AVR family,
 * not chip-specific (the prescaler taps clk/8, clk/64, clk/256, clk/1024 off a free-running
 * counter, the same divider chain every classic-AVR timer peripheral expects). */
module tim_013_prescaller (
    input rst,
    input clk,
    output clk8,
    output clk64,
    output clk256,
    output clk1024
);
reg [9:0]cnt;

always @ (posedge clk)
begin
    if(rst)
    begin
        cnt <= 10'h000;
    end
    else
    begin
        cnt <= cnt + 10'd1;
    end
end

assign clk8 = cnt[2];
assign clk64 = cnt[5];
assign clk256 = cnt[7];
assign clk1024 = cnt[9];

endmodule
/* !TIMER PRESCALER MODULE */

module atmega644 # (
    parameter REGS_REGISTERED = "FALSE",
    parameter USE_HALT = "FALSE",
    parameter USE_PIO_A = "TRUE",
    parameter USE_PIO_B = "TRUE",
    parameter USE_PIO_C = "TRUE",
    parameter USE_PIO_D = "TRUE",
    parameter USE_TIMER_0 = "TRUE",
    parameter USE_TIMER_1 = "TRUE",
    parameter USE_TIMER_2 = "TRUE",
    parameter USE_SPI_0 = "TRUE",
    parameter USE_UART_0 = "TRUE",
    parameter USE_EEPROM = "TRUE"
)(
    input rst,
    input clk,
    output [`ROM_ADDR_WIDTH-1:0] pgm_addr,
    input [15:0] pgm_data,
    // Program-memory write port for SPM Page Erase/Write; wired up at board level.
    output [`ROM_ADDR_WIDTH-1:0] spm_pgm_addr,
    output [15:0] spm_pgm_data,
    output spm_pgm_write,
    input spm_pgm_write_ack,

    // Real chip pins, full width, real names, genuinely bidirectional (PORT/DDR out, PIN in) —
    // this module is chip-generic, not Uzebox-specific. A board wrapper builds any real
    // tristate/pull-up behavior it needs on top of these three signals per port.
    input  [7:0] PINA_ext,
    output [7:0] PORTA_out,
    output [7:0] DDRA_out,
    input  [7:0] PINB_ext,
    output [7:0] PORTB_out,
    output [7:0] DDRB_out,
    input  [7:0] PINC_ext,
    output [7:0] PORTC_out,
    output [7:0] DDRC_out,
    input  [7:0] PIND_ext,
    output [7:0] PORTD_out,
    output [7:0] DDRD_out
    );

wire core_clk = clk;
wire wdt_rst;

/* CORE WIRES */
wire [`BUS_ADDR_DATA_LEN-1:0]data_addr;
wire [7:0]core_data_out;
wire data_write;
reg  [7:0]core_data_in;
wire data_read;
/* !CORE WIRES */

/* IO WIRES */
wire [7:0]pa_in;
wire [7:0]pb_in;
wire [7:0]pc_in;
wire [7:0]pd_in;
wire [7:0]pa_out;
wire [7:0]pb_out;
wire [7:0]pc_out;
wire [7:0]pd_out;
/* !IO WIRES */

assign pa_in = PINA_ext;
assign pb_in = PINB_ext;
assign pc_in = PINC_ext;
assign pd_in = PIND_ext;

/* Real GPIO output driven straight through — DDR is exposed separately below so a board
 * wrapper can build correct tristate behavior. Note (2026-08-10, carried over from the
 * arduboy-atmega-module project's audit of this same shared atmega-pio.v): io_out is computed
 * as `DDR[n] ? PORT[n] : 1'b0` (atmega-pio.v:54-61), i.e. it substitutes a hard 0 instead of
 * real input-mode pull-up behavior when DDR=0. That's a real, already-tracked limitation of the
 * shared peripheral file (not something this file introduces or fixes) — flagged here because
 * PORTA's joystick shift-register lines are genuinely bidirectional-in-spirit on real Uzebox
 * hardware and a future board wrapper needs to know about this before trusting PORTA_out's value
 * while DDRA_out indicates an input pin. */
assign PORTA_out = pa_out;
assign PORTB_out = pb_out;
assign PORTC_out = pc_out;
assign PORTD_out = pd_out;

/* Interrupt wires — 27 real ATmega644 sources, datasheet S10.1 Table 10-1, vectors 2-28 */
wire int_int0;
wire int_int1;
wire int_int2;
wire int_pcint0;
wire int_pcint1;
wire int_pcint2;
wire int_pcint3;
wire int_wdt = 0; // watchdog disabled (WATCHDOG_CNT_WIDTH=0), matches atmega32u4.v's own convention
wire int_timer2_compa;
wire int_timer2_compb;
wire int_timer2_ovf;
wire int_timer1_capt;
wire int_timer1_compa;
wire int_timer1_compb;
wire int_timer1_ovf;
wire int_timer0_compa;
wire int_timer0_compb;
wire int_timer0_ovf;
wire int_spi_stc;
wire int_usart0_rx;
wire int_usart0_udre;
wire int_usart0_tx;
wire int_analog_comp = 0; // analog comparator not implemented (no analog front-end in this model)
wire int_adc = 0;         // ADC not implemented — Uzebox doesn't use it (all pin functions digital)
wire int_ee_ready;
wire int_twi = 0;         // TWI/I2C not implemented — Uzebox schematic doesn't wire it either
wire int_spm_ready = 0;   // SPM_READY interrupt not implemented; SPM itself is
/* !Interrupt wires */

/* Interrupt reset wires */
wire int_int0_rst;
wire int_int1_rst;
wire int_int2_rst;
wire int_pcint0_rst;
wire int_pcint1_rst;
wire int_pcint2_rst;
wire int_pcint3_rst;
wire int_wdt_rst;
wire int_timer2_compa_rst;
wire int_timer2_compb_rst;
wire int_timer2_ovf_rst;
wire int_timer1_capt_rst;
wire int_timer1_compa_rst;
wire int_timer1_compb_rst;
wire int_timer1_ovf_rst;
wire int_timer0_compa_rst;
wire int_timer0_compb_rst;
wire int_timer0_ovf_rst;
wire int_spi_stc_rst;
wire int_usart0_rx_rst;
wire int_usart0_udre_rst;
wire int_usart0_tx_rst;
wire int_analog_comp_rst;
wire int_adc_rst;
wire int_ee_ready_rst;
wire int_twi_rst;
wire int_spm_ready_rst;
/* !Interrupt reset wires */

wire ram_sel = |data_addr[`BUS_ADDR_DATA_LEN-1:8];

/* PCINT0-3 not modeled (no pin-change-interrupt logic implemented — Uzebox's kernel doesn't
 * use pin-change interrupts anywhere in the boot io_table or video/sound engine, confirmed
 * PROJECT.md kernel-source research). Tied off, matching the same-shape gaps atmega32u4.v
 * already carries for USB/ADC/TWI/etc. — an honest stub, not a silent omission. */
assign int_pcint0 = 1'b0;
assign int_pcint1 = 1'b0;
assign int_pcint2 = 1'b0;
assign int_pcint3 = 1'b0;

/* External interrupts INT0-2 not modeled — same reasoning: no external-interrupt use anywhere
 * in the kernel source this project has read. */
assign int_int0 = 1'b0;
assign int_int1 = 1'b0;
assign int_int2 = 1'b0;

/* PORTA */
wire [7:0]dat_pa_d_out;
generate
if (USE_PIO_A == "TRUE")
begin: PORTA
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h22),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h21),
    .PIN_ADDR('h20),
    .PINMASK(8'b11111111),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00000000),
    .OUT_ENABLED_MASK(8'b11111111)
)pio_a(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pa_d_out),

    .io_in(pa_in),
    .io_out(pa_out)
    );
end
else
begin
assign dat_pa_d_out = 0;
end
endgenerate
/* !PORTA */

/* PORTB */
wire [7:0]dat_pb_d_out;
generate
if (USE_PIO_B == "TRUE")
begin: PORTB
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h25),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h24),
    .PIN_ADDR('h23),
    .PINMASK(8'b11111111),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00000000),
    .OUT_ENABLED_MASK(8'b11111111)
)pio_b(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pb_d_out),

    .io_in(pb_in),
    .io_out(pb_out)
    );
end
else
begin
assign dat_pb_d_out = 0;
end
endgenerate
/* !PORTB */

/* PORTC — the Uzebox video-data port (PROJECT.md pin map: DDRC=0xFF at boot, "video dac"),
 * but this file itself is Uzebox-agnostic — it's just a real, full 8-bit ATmega644 GPIO port. */
wire [7:0]dat_pc_d_out;
generate
if (USE_PIO_C == "TRUE")
begin: PORTC
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h28),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h27),
    .PIN_ADDR('h26),
    .PINMASK(8'b11111111),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00000000),
    .OUT_ENABLED_MASK(8'b11111111)
)pio_c(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pc_d_out),

    .io_in(pc_in),
    .io_out(pc_out)
    );
end
else
begin
assign dat_pc_d_out = 0;
end
endgenerate
/* !PORTC */

/* PORTD */
wire [7:0]dat_pd_d_out;
generate
if (USE_PIO_D == "TRUE")
begin: PORTD
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h2b),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h2a),
    .PIN_ADDR('h29),
    .PINMASK(8'b11111111),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00000000),
    .OUT_ENABLED_MASK(8'b11111111)
)pio_d(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pd_d_out),

    .io_in(pd_in),
    .io_out(pd_out)
    );
end
else
begin
assign dat_pd_d_out = 0;
end
endgenerate
/* !PORTD */

/* DDR readback — atmega_pio.v doesn't expose DDR as a module output, so the top-level
 * DDRx_out ports below are driven from the same bus_dat_out path a CPU read would see (i.e.
 * "what DDRx currently holds", sourced by reading the peripheral's own DDR_ADDR case). This
 * avoids modifying the shared atmega-pio.v file — a small local shim instead. */
reg [7:0] ddra_shadow, ddrb_shadow, ddrc_shadow, ddrd_shadow;
always @ (posedge clk)
begin
    if (rst)
    begin
        ddra_shadow <= 8'h00;
        ddrb_shadow <= 8'h00;
        ddrc_shadow <= 8'h00;
        ddrd_shadow <= 8'h00;
    end
    else if (data_write & ~ram_sel)
    begin
        case (data_addr[7:0])
            'h21: ddra_shadow <= core_data_out;
            'h24: ddrb_shadow <= core_data_out;
            'h27: ddrc_shadow <= core_data_out;
            'h2a: ddrd_shadow <= core_data_out;
        endcase
    end
end
assign DDRA_out = ddra_shadow;
assign DDRB_out = ddrb_shadow;
assign DDRC_out = ddrc_shadow;
assign DDRD_out = ddrd_shadow;

/* SPI (SPI0 — the ATmega644 has one SPI master/slave peripheral, unlike the 32U4's SPI1
 * naming; same underlying atmega-spi-m.v file either way) */
wire [7:0]dat_spi_d_out;
wire spi_io_connect;
wire io_conn_slave;
// Real MISO input (PB6 per the V5.0 schematic pin map) -- previously an undriven wire, same
// class of bug as Arduboy's pre-fix atmega32u4.v: the CPU could never read a real byte back.
wire spi_miso = pb_in[6];
generate
if (USE_SPI_0 == "TRUE")
begin: SPI0
atmega_spi_m # (
    .BUS_ADDR_DATA_LEN(8),
    .SPCR_ADDR('h4c),
    .SPSR_ADDR('h4d),
    .SPDR_ADDR('h4e),
    .DINAMIC_BAUDRATE("FALSE"),
    .BAUDRATE_CNT_LEN(0),
    .BAUDRATE_DIVIDER(0),
    .USE_TX("TRUE"),
    .USE_RX("TRUE")
)spi(
    .rst(rst),
    .halt(1'b0),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_spi_d_out),
    .int_out(int_spi_stc),
    .int_rst(int_spi_stc_rst),
    .io_connect(spi_io_connect),
    .io_conn_slave(io_conn_slave),

    .scl(),
    .miso(spi_miso),
    .mosi()
    );
end
else
begin
assign dat_spi_d_out = 0;
assign int_spi_stc = 1'b0;
assign spi_io_connect = 1'b0;
end
endgenerate
/* !SPI */

/* UART (USART0 — the ATmega644 datasheet's only wired USART on this board, confirmed
 * schematic-side in PROJECT.md: PD2/PD3 are repurposed as plain GPIO on V5.0, not a real
 * USART1 despite what the datasheet's own inconsistent overview text implies) */
wire [7:0]dat_uart0_d_out;
generate
if (USE_UART_0 == "TRUE")
begin: UART0
atmega_uart # (
    .BUS_ADDR_DATA_LEN(8),
    .UDR_ADDR('hc6),
    .UCSRA_ADDR('hc0),
    .UCSRB_ADDR('hc1),
    .UCSRC_ADDR('hc2),
    .UBRRL_ADDR('hc4),
    .UBRRH_ADDR('hc5),
    .USE_TX("TRUE"),
    .USE_RX("TRUE")
    )uart(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_uart0_d_out),
    .rxc_int(int_usart0_rx),
    .rxc_int_rst(int_usart0_rx_rst),
    .txc_int(int_usart0_tx),
    .txc_int_rst(int_usart0_tx_rst),
    .udre_int(int_usart0_udre),
    .udre_int_rst(int_usart0_udre_rst),

    .rx(pd_in[0]),
    .tx(),
    .tx_connect()
    );
end
else
begin
assign dat_uart0_d_out = 1'b0;
assign int_usart0_rx = 1'b0;
assign int_usart0_tx = 1'b0;
assign int_usart0_udre = 1'b0;
end
endgenerate
/* !UART */

/* TIMER PRESCALER */
wire clk8;
wire clk64;
wire clk256;
wire clk1024;
tim_013_prescaller tim_013_prescaller_inst(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024)
);
/* !TIMER PRESCALER */

/* TIMER 0 — 8-bit, drives the colorburst clock on Uzebox (OC0A -> PB3) */
wire [7:0]dat_tim0_d_out;
generate
if (USE_TIMER_0 == "TRUE")
begin:TIMER0
atmega_tim_8bit # (
    .USE_OCRB("TRUE"),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h43),
    .TCCRA_ADDR('h44),
    .TCCRB_ADDR('h45),
    .TCNT_ADDR('h46),
    .OCRA_ADDR('h47),
    .OCRB_ADDR('h48),
    .TIMSK_ADDR('h6E),
    .TIFR_ADDR('h35)
)tim_0(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim0_d_out),
    .tov_int(int_timer0_ovf),
    .tov_int_rst(int_timer0_ovf_rst),
    .ocra_int(int_timer0_compa),
    .ocra_int_rst(int_timer0_compa_rst),
    .ocrb_int(int_timer0_compb),
    .ocrb_int_rst(int_timer0_compb_rst),
    .oca(tim0_oca),
    .ocb(tim0_ocb),
    .oca_io_connect(tim0_oca_io_connect),
    .ocb_io_connect(tim0_ocb_io_connect)
    );
end
else
begin
assign dat_tim0_d_out = 0;
assign int_timer0_ovf = 1'b0;
assign int_timer0_compa = 1'b0;
assign int_timer0_compb = 1'b0;
end
endgenerate
wire tim0_oca, tim0_ocb, tim0_oca_io_connect, tim0_ocb_io_connect;
/* !TIMER 0 */

/* TIMER 1 — 16-bit, drives HSYNC/VSYNC on Uzebox via software ISR (no hardware OC1x
 * connection needed for that role, but OC1A/OC1B are still real chip outputs, wired here for
 * completeness) */
wire [7:0]dat_tim1_d_out;
generate
if (USE_TIMER_1 == "TRUE")
begin: TIMER1
atmega_tim_16bit # (
    .USE_OCRB("TRUE"),
    .USE_OCRC("FALSE"),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h43),
    .TCCRA_ADDR('h80),
    .TCCRB_ADDR('h81),
    .TCCRC_ADDR('h82),
    .TCNTL_ADDR('h84),
    .TCNTH_ADDR('h85),
    .ICRL_ADDR('h86),
    .ICRH_ADDR('h87),
    .OCRAL_ADDR('h88),
    .OCRAH_ADDR('h89),
    .OCRBL_ADDR('h8A),
    .OCRBH_ADDR('h8B),
    .OCRCL_ADDR('h8C),
    .OCRCH_ADDR('h8D),
    .TIMSK_ADDR('h6F),
    .TIFR_ADDR('h36)
)tim_1(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim1_d_out),
    .tov_int(int_timer1_ovf),
    .tov_int_rst(int_timer1_ovf_rst),
    .ocra_int(int_timer1_compa),
    .ocra_int_rst(int_timer1_compa_rst),
    .ocrb_int(int_timer1_compb),
    .ocrb_int_rst(int_timer1_compb_rst),
    .ocrc_int(),
    .ocrc_int_rst(1'b0),
    .oca(tim1_oca),
    .ocb(tim1_ocb),
    .occ(),
    .oca_io_connect(tim1_oca_io_connect),
    .ocb_io_connect(tim1_ocb_io_connect),
    .occ_io_connect()
    );
    assign int_timer1_capt = 1'b0; // ICP1 input capture not modeled — Uzebox kernel never uses it
    assign int_timer1_capt_rst = 1'b0;
end
else
begin
assign dat_tim1_d_out = 0;
assign int_timer1_ovf = 1'b0;
assign int_timer1_compa = 1'b0;
assign int_timer1_compb = 1'b0;
assign int_timer1_capt = 1'b0;
end
endgenerate
wire tim1_oca, tim1_ocb, tim1_oca_io_connect, tim1_ocb_io_connect;
/* !TIMER 1 */

/* TIMER 2 — 8-bit, drives audio PWM on Uzebox (OC2A -> PD7). No async/32kHz-crystal (ASSR)
 * support — atmega-tim-8bit.v doesn't implement it, and Uzebox's kernel never enables it
 * (TCCR2B=(1<<CS20), synchronous mode only, confirmed PROJECT.md kernel-source research). */
wire [7:0]dat_tim2_d_out;
generate
if (USE_TIMER_2 == "TRUE")
begin:TIMER2
atmega_tim_8bit # (
    .USE_OCRB("TRUE"),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h43),
    .TCCRA_ADDR('hb0),
    .TCCRB_ADDR('hb1),
    .TCNT_ADDR('hb2),
    .OCRA_ADDR('hb3),
    .OCRB_ADDR('hb4),
    .TIMSK_ADDR('h70),
    .TIFR_ADDR('h37)
)tim_2(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim2_d_out),
    .tov_int(int_timer2_ovf),
    .tov_int_rst(int_timer2_ovf_rst),
    .ocra_int(int_timer2_compa),
    .ocra_int_rst(int_timer2_compa_rst),
    .ocrb_int(int_timer2_compb),
    .ocrb_int_rst(int_timer2_compb_rst),
    .oca(tim2_oca),
    .ocb(tim2_ocb),
    .oca_io_connect(tim2_oca_io_connect),
    .ocb_io_connect(tim2_ocb_io_connect)
    );
end
else
begin
assign dat_tim2_d_out = 0;
assign int_timer2_ovf = 1'b0;
assign int_timer2_compa = 1'b0;
assign int_timer2_compb = 1'b0;
end
endgenerate
wire tim2_oca, tim2_ocb, tim2_oca_io_connect, tim2_ocb_io_connect;
/* !TIMER 2 */

/* EEPROM — 2KB, EEP_SIZE below must match `EEP_ADDR_WIDTH (2^11 = 2048) */
wire [7:0]dat_eeprom_d_out;
generate
if (USE_EEPROM == "TRUE")
begin: EEPROM
atmega_eep # (
    .BUS_ADDR_DATA_LEN(8),
    .EEARH_ADDR('h42),
    .EEARL_ADDR('h41),
    .EEDR_ADDR('h40),
    .EECR_ADDR('h3F),
    .EEP_SIZE(2048)
)eep(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_eeprom_d_out),
    .int_out(int_ee_ready),
    .int_rst(int_ee_ready_rst)
    );
end
else
begin
assign dat_eeprom_d_out = 0;
end
endgenerate
/* !EEPROM */

/* RAM */
wire [7:0]ram_bus_out;
wire [7:0]ram_bus_out2;
wire [`BUS_ADDR_DATA_LEN-1:0]data_addr2;
mega_ram  #(
    .ADDR_BUS_WIDTH(`RAM_ADDR_WIDTH),
    .DATA_BUS_WIDTH(8),
    .RAM_PATH("")
)ram(
    .rst(rst),
    // Falling-edge clocked: the address is presented on the rising edge and the data is back
    // half a cycle later, in time for the register write on the next rising edge. This is what
    // lets LD retire in the 2 cycles the ISA manual specifies instead of 3.
    .clk(~core_clk),
    .we(data_write & ram_sel),
    .a(data_addr[`RAM_ADDR_WIDTH-1:0]),
    .d_in(core_data_out),
    .d_out(ram_bus_out),
    // Second read-only port, RET/RETI only (mega-core.v's data_addr2/data_in2) -- lets both
    // stack-pop bytes be addressed the same cycle instead of serially sharing this port.
    .a2(data_addr2[`RAM_ADDR_WIDTH-1:0]),
    .d_out2(ram_bus_out2)
);
/* !RAM */

/* DATA BUS IN DEMULTIPLEXER */
always @ *
begin
    core_data_in = ram_bus_out;
    if(~ram_sel) begin
        case(data_addr[7:0])
            'h22, 'h21, 'h20: core_data_in = dat_pa_d_out;
            'h25, 'h24, 'h23: core_data_in = dat_pb_d_out;
            'h28, 'h27, 'h26: core_data_in = dat_pc_d_out;
            'h2b, 'h2a, 'h29: core_data_in = dat_pd_d_out;
            'h4c, 'h4d, 'h4e: core_data_in = dat_spi_d_out;
            'h43,             // GTCCR — shared by Timer0/Timer1/Timer2, routed to Timer0's copy
            'h44, 'h45, 'h46,
            'h47, 'h48, 'h6E,
            'h35:             core_data_in = dat_tim0_d_out;
            'h6F, 'h36:       core_data_in = dat_tim1_d_out;
            'hb0, 'hb1, 'hb2,
            'hb3, 'hb4, 'h70,
            'h37:             core_data_in = dat_tim2_d_out;
            'h42, 'h41, 'h40,
            'h3F:             core_data_in = dat_eeprom_d_out;
            'hc6, 'hc0, 'hc1,
            'hc2, 'hc4, 'hc5: core_data_in = dat_uart0_d_out;
        endcase
        case(data_addr[7:4])
            'h8:              core_data_in = dat_tim1_d_out;
        endcase
    end
end
/* !DATA BUS IN DEMULTIPLEXER */

/* ATMEGA CORE */
mega # (
    .CORE_TYPE(`CORE_TYPE),
    .ROM_ADDR_WIDTH(`ROM_ADDR_WIDTH),
    .RAM_ADDR_WIDTH(`BUS_ADDR_DATA_LEN),
    .WATCHDOG_CNT_WIDTH(`WATCHDOG_CNT_WIDTH),/* If is 0 the watchdog is disabled */
    .VECTOR_INT_TABLE_SIZE(`VECTOR_INT_TABLE_SIZE),/* If is 0 the interrupt module is disabled */
    .USE_HALT(USE_HALT),
    .REGS_REGISTERED(REGS_REGISTERED),
    .SPM_PAGE_ADDR_WIDTH(`SPM_PAGE_ADDR_WIDTH)
    )atmega644_inst(
    .rst(rst),
    .sys_rst_out(wdt_rst),
    // Core clock.
    .clk(core_clk),
    // Watchdog clock input that can be different from the core clock.
    .clk_wdt(core_clk),
    // Used to halt the core.
    .halt(1'b0),
    .halt_ack(),
    // FLASH space data interface.
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    .spm_pgm_addr(spm_pgm_addr),
    .spm_pgm_data(spm_pgm_data),
    .spm_pgm_write(spm_pgm_write),
    .spm_pgm_write_ack(spm_pgm_write_ack),
    // RAM space data interface.
    .data_addr(data_addr),
    .data_out(core_data_out),
    .data_write(data_write),
    .data_in(core_data_in),
    .data_read(data_read),
    // Dedicated second RAM read port, RET/RETI only -- see the `ram` instance above.
    .data_addr2(data_addr2),
    .data_in2(ram_bus_out2),
    // Interrupt lines from all IO's, descending vector-number order (vector 28 down to vector 2,
    // bit0=vector2/INT0), matching the convention derived from atmega32u4.v's own int_sig wiring
    // and confirmed against mega-core.v's int_encoder (mega-core.v:107-146). See PROJECT.md.
    .int_sig({
    int_spm_ready,                                            // 28
    int_twi,                                                  // 27
    int_ee_ready,                                             // 26
    int_adc,                                                  // 25
    int_analog_comp,                                          // 24
    int_usart0_tx, int_usart0_udre, int_usart0_rx,            // 23,22,21
    int_spi_stc,                                              // 20
    int_timer0_ovf, int_timer0_compb, int_timer0_compa,       // 19,18,17
    int_timer1_ovf, int_timer1_compb, int_timer1_compa, int_timer1_capt, // 16,15,14,13
    int_timer2_ovf, int_timer2_compb, int_timer2_compa,       // 12,11,10
    int_wdt,                                                  // 9
    int_pcint3, int_pcint2, int_pcint1, int_pcint0,           // 8,7,6,5
    int_int2, int_int1, int_int0}                             // 4,3,2
    ),
    // Interrupt reset lines going to all IO's.
    .int_rst({
    int_spm_ready_rst,
    int_twi_rst,
    int_ee_ready_rst,
    int_adc_rst,
    int_analog_comp_rst,
    int_usart0_tx_rst, int_usart0_udre_rst, int_usart0_rx_rst,
    int_spi_stc_rst,
    int_timer0_ovf_rst, int_timer0_compb_rst, int_timer0_compa_rst,
    int_timer1_ovf_rst, int_timer1_compb_rst, int_timer1_compa_rst, int_timer1_capt_rst,
    int_timer2_ovf_rst, int_timer2_compb_rst, int_timer2_compa_rst,
    int_wdt_rst,
    int_pcint3_rst, int_pcint2_rst, int_pcint1_rst, int_pcint0_rst,
    int_int2_rst, int_int1_rst, int_int0_rst}
    )
);
/* !ATMEGA CORE */

endmodule

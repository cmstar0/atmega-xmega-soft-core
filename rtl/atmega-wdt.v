/*
 * This IP is the ATMEGA Watchdog Timer implementation.
 *
 * Companion to the MEGA/XMEGA peripheral IP Copyright (C) Iulian Gheorghiu
 * (morgoth@devboard.tech), GPLv2-or-later. This file is new, written against the
 * ATmega16U4/32U4 datasheet (Atmel-7766J, Tables 8-4 and 8-5) and the ATmega644
 * datasheet (doc2593O, Tables 9-1 and 9-2). WDTCSR is at 0x60 with identical bits,
 * prescaler and unlock sequence on both parts; only the vector number differs, and
 * that belongs to the chip top.
 *
 * Interrupt mode only. WDE is stored, gates the WDIE auto-clear on vector entry and
 * keeps the counter running, but drives no system reset.
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

module atmega_wdt # (
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter WDTCSR_ADDR = 'h60,
    // Core clocks per tick of the 128kHz WDT oscillator. The instantiating chip
    // top expresses that, since only it knows its own clock.
    parameter OSC_PRESCALER = 1,
    parameter OSC_PRESCALER_WIDTH = 1
)(
    input rst,
    input clk,

    input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [7:0]bus_dat_in,
    output reg [7:0]bus_dat_out,

    input wdr,

    output int_out,
    input int_rst
    );

localparam WDIF = 7;
localparam WDIE = 6;
localparam WDP3 = 5;
localparam WDCE = 4;
localparam WDE  = 3;

reg [7:0]WDTCSR;
reg [2:0]wdce_cnt;
reg [OSC_PRESCALER_WIDTH-1:0]osc_cnt;
reg [19:0]wdt_cnt;
reg tap_del;

wire [3:0]wdp = {WDTCSR[WDP3], WDTCSR[2:0]};
// WDP 1010-1111 are reserved; hold the longest documented period.
wire [3:0]wdp_sel = (wdp > 4'd9) ? 4'd9 : wdp;
wire running = WDTCSR[WDE] | WDTCSR[WDIE];
wire osc_tick = (osc_cnt == (OSC_PRESCALER[OSC_PRESCALER_WIDTH-1:0] - 1'b1));
// Table 8-5: 2K cycles at WDP=0, doubling per step. Bit 10+WDP falls once per
// period, first fall a full period after a reset or WDR.
wire tap = wdt_cnt[10 + wdp_sel];
wire timeout = tap_del & ~tap;

assign int_out = WDTCSR[WDIF] & WDTCSR[WDIE];

always @ *
begin
    bus_dat_out = 8'h00;
    if(rd_dat)
    begin
        case(addr_dat)
            WDTCSR_ADDR: bus_dat_out = WDTCSR;
        endcase
    end
end

always @ (posedge clk)
begin
    if(rst)
    begin
        WDTCSR <= 8'h00;
        wdce_cnt <= 3'h0;
        osc_cnt <= 'h0;
        wdt_cnt <= 20'h0;
        tap_del <= 1'b0;
    end
    else
    begin
        if(|wdce_cnt)
        begin
            wdce_cnt <= wdce_cnt - 1'b1;
            if(wdce_cnt == 3'h1)
                WDTCSR[WDCE] <= 1'b0;
        end

        if(running)
        begin
            tap_del <= tap;
            if(osc_tick)
            begin
                osc_cnt <= 'h0;
                wdt_cnt <= wdt_cnt + 1'b1;
            end
            else
                osc_cnt <= osc_cnt + 1'b1;
            if(timeout)
                WDTCSR[WDIF] <= 1'b1;
        end
        else
        begin
            osc_cnt <= 'h0;
            wdt_cnt <= 20'h0;
            tap_del <= 1'b0;
        end

        if(wdr)
        begin
            osc_cnt <= 'h0;
            wdt_cnt <= 20'h0;
            tap_del <= 1'b0;
        end

        if(int_rst)
        begin
            WDTCSR[WDIF] <= 1'b0;
            // Interrupt and System Reset mode: the vector disarms WDIE.
            if(WDTCSR[WDE])
                WDTCSR[WDIE] <= 1'b0;
        end

        if(wr_dat)
        begin
            case(addr_dat)
                WDTCSR_ADDR:
                begin
                    if(bus_dat_in[WDIF])
                        WDTCSR[WDIF] <= 1'b0;
                    WDTCSR[WDIE] <= bus_dat_in[WDIE];
                    if(&{bus_dat_in[WDCE], bus_dat_in[WDE]})
                    begin
                        WDTCSR[WDCE] <= 1'b1;
                        WDTCSR[WDE] <= 1'b1;
                        wdce_cnt <= 3'h4;
                    end
                    else if(|wdce_cnt)
                    begin
                        WDTCSR[WDCE] <= 1'b0;
                        WDTCSR[WDE] <= bus_dat_in[WDE];
                        WDTCSR[WDP3] <= bus_dat_in[WDP3];
                        WDTCSR[2:0] <= bus_dat_in[2:0];
                        wdce_cnt <= 3'h0;
                    end
                    else if(bus_dat_in[WDE])
                        WDTCSR[WDE] <= 1'b1;
                end
            endcase
        end
    end
end

endmodule

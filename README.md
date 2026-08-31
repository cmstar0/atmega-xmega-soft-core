# atmega-xmega-soft-core

AVR (ATmega/ATxmega) soft core in Verilog. Fork of
[MorgothCreator/atmega-xmega-soft-core](https://github.com/MorgothCreator/atmega-xmega-soft-core)
by Iulian Gheorghiu, maintained here with a focus on **instruction-set completeness and
cycle accuracy** against the AVR Instruction Set Manual and the ATmega datasheets.

## What changed from upstream

`rtl/` replaces upstream's `core/`, `mega_io/` and `customized/` wholesale. It is **not** an
incremental evolution of that code: it is the substantially reworked descendant that ships in
the MiSTer `Arduboy_MiSTer` core, plus a series of instruction-accuracy fixes made here. Upstream
stopped at 2020-02-21 and the two trees diverged well before this fork existed, so there is no
merge path back.

`common/` and `custom_io/` are upstream's, untouched.

## Layout

```
rtl/
  mega-core.v  mega-alu.v  mega-def.v  mega-ram.v  mega-reg.v   the CPU
  mega-rom.v                                                    generic program ROM
  atmega-pio.v  atmega-eep.v  atmega-spi-m.v                    peripherals
  atmega-tim-8bit.v  atmega-tim-16bit.v  atmega-tim-10bit.v
  atmega-uart.v  atmega-pll.v
  atmega32u4.v                                                  worked example chip
```

`atmega32u4.v` shows how the engine and peripherals wire together for one real AVR. It is the
MiSTer core's file with its MiSTer-specific ADC removed — that block instantiated an
Altera-licensed reference design (`unstable_counters.v`), which is not GPL and is therefore not
carried here. Reads of `ADCL`/`ADCH` fall through to the bus default like any other unimplemented
I/O address.

The 8-bit and 16-bit timers instantiate Altera's `lpm_mux` library primitive, so they currently
need an Altera/Intel synthesis environment. The CPU itself does not.

## Verification

Fixes are developed here, then proven on real hardware by building them into `Arduboy_MiSTer`
and running actual games on a MiSTer FPGA. A clean compile or a passing simulation is not
treated as evidence that a fix works.

## Licence

GPLv2 or later, unchanged from upstream. Copyright of the original design remains with
Iulian Gheorghiu; see the per-file headers and `LICENSE`.

---

# Upstream README (Iulian Gheorghiu)

# atmega-xmega-soft-core

 Mega/Xmega soft core RTL design.

 A preety complete implementation of ATmega/ATxmega soft core.

 This design include all most used IO's, priority interrupt module and watchdog module, except UART that is in development.

  # V00.02.16:

 ```
  -Fix SBIW instruction due to wrong description in oficial documentation.
  -Add simple UART interface.
  -Optimize core code and make it more readable.

 TO DO:
 
 Observed some issues with TIM3 on 'arduboy-rtl-emulator' project, so is needed to 
 "Fix situations where on random times at core reset the TIM3 prescaller is setup at 
 wrong value ( at /64 instead of /8 core clock )" need to check in what situation 
 this issue is manifesting.
 ```

 # V00.02.10:

 ```
 Initial commit.
 Tested on Digilent Nexis Video board.
 ```

  If you like my work, you can help further development by donating as little as 1 EUR.
  
 [![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=CZM6JXDVMFXHS&source=url)

 Or you can send some crypto:

 BTC: 3CFRp6day6ZRgpXw8n1QGvXfmk5gf8XK3e

 LBRY: bbVdwfTsVkFhA3qcq2znyD7juuuDnUdMT1

 MONERO: 8ALzMJESPVrdCmrQuwssrZVvdg4wBvtt6DXigYxZ33ZuQVHQBXNpHpoCZVR4smKLHhYPsSgsH4BvYCXdBNdZzFH8AB5z8vs

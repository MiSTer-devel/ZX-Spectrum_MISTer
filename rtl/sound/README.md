# HQ audio

With **HQ Audio** on (Audio page), every sound source is mixed on one bus at
a fixed balance and band-limited; with it off the legacy mix is used
unchanged.

## Signal path

All processing runs on `clk_aud` at the PSG generator rate, 218.75 kHz
(16 T-states). Samples are Q4.28 with 1.0 = digital full scale.

```
YM2149 5-bit levels ─ ay_dac ─ ay_stereo_mixer ─ ay_dc_filter ─ ay_fir_decimator ─ ay_voicing ─ ay_punch_enhancer ─ ay_room_crossfeed ─┐
   (turbosound.sv, both chips)                                                                                                     │
FM words, GS, Covox, SAA1099, beeper/tape ─ 16-T integrate ─ hq_fir_stereo ───────────────────────────────────────────────────────┤
                                                                hq_mix: + ─ hq_dc ─ x1.664 ─ look-ahead compressor ─ soft limiter ─ first-order hold ─ AUDIO
```

- **AY/YM:** exact AY or YM DAC table (PSG Model), both chips summed without
  saturation. Pan 0.9/0.5/0.1 per side for ABC or ACB, divided by 3. Then a
  5 Hz DC blocker and a 96-tap Kaiser 20 kHz FIR, both at the full rate.
  Voicing, punch and room are applied to the AY only.
- **Other sources:** step outputs, integrated over each 16-T window, then a
  stereo FIR with the same coefficients on one MAC, with block RAM history.
  Gains in full-scale units:
  - FM: `word / 32768 * 0.703` per chip, mono.
  - General Sound: `(s - 128) * vol * 1.9226 / 32768` per channel, then
    `L = (l + r/2)/2`.
  - Covox: `(v - 128) / 256`.
  - Beeper: EAR/MIC levels 0 / 1600 / 15400 / 16000 out of 32768.
  - SAA1099: its legacy weight against one AY channel.
- **Master:**
  - 5 Hz DC blocker built from shifts.
  - Makeup gain x1.664 (+4.42 dB), so HQ is as loud as the legacy mix,
    whose `compr()` doubles small signals.
  - Look-ahead peak compressor, stereo-linked. Above T = 0.7071 FS it applies
    gain = T / peak: the target is held for the 256-sample (1.17 ms) delay,
    the attack is 1/32 per sample, and the release is 150 ms. Below T the mix
    passes untouched.
  - A soft limiter curve as a safety net: linear below 0.75 FS,
    `knee + span * (1 - exp(-(|x| - knee) / span))` above it, with a ceiling
    of 32000/32768. Because the compressor holds peaks at T, the limiter
    curve is normally never reached.
  - A first-order hold then brings the samples to the 3.5 MHz CE rate.

`hq_limiter_rom.sv` and `hq_comp_rom.sv` are generated: `python3 gen_roms.py`.

## Balance

FM full scale is about 4.7 times one PSG channel on its loud side, as measured
on TSFM hardware.

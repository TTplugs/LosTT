WIP - LosTT LV2 tape/media coloration effect inspired by Generation Loss MKII behavior, adapted for S2400-friendly LV2 deployment.

## Overview
- URI: `urn:ttlab:lv2:lostt`
- Bundle: `LosTT.lv2`
- Type: LV2 stereo effect (audio in/out, no GUI)
- `model` is continuous float `0.0..11.0` with named scale points
- Values between integer models morph continuously between adjacent emulations

## 12 Models (requested set)
- `0` VHS
- `1` Porter Studio
- `2` Camcorder
- `3` Dictaphones
- `4` Kids' Toy Tape Machine
- `5` Sony Walkman
- `6` Broken
- `7` Reel-To-Reel
- `8` VCR 2
- `9` Cassette 2
- `10` Cassette 3
- `11` Digital

## Technical behavior mapping
- `wow`: slow random pitch drift
- `flutter`: fast random pitch + amplitude jitter
- `saturate`: stateful tape-like saturation / compression behavior
- `failure`: combined drops, snags, and crinkle/pop layer
- `aux_mode` + `aux_active`: Stop / Filter / Fail performance behavior
- `dry`: None/Small/Unity clean blend states
- `noise`: None/Mild/Heavy base noise states
- `spread`: failure-based stereo decorrelation
- hidden/dip-style controls:
  - `aux_onset`, `dry_type`, `drop_bypass`, `snag_bypass`, `hum_bypass`
  - `hiss_level`, `mech_level`, `mech_type`, `crinkle_pop_level`
  - `input_gain_mode` (Line / Instrument / High Gain)

## Build / Check
```bash
cd /home/asier/LosTT
make clean
make
make check
file LosTT.lv2/LosTT.so
readelf -d LosTT.lv2/LosTT.so | grep NEEDED
strings -a LosTT.lv2/LosTT.so | grep -E 'GLIBC_|GLIBCXX_|GCC_' | sort -V | uniq
nm -D LosTT.lv2/LosTT.so | grep lv2_descriptor
```

## Deploy
```bash
scp -r LosTT.lv2 asier@[IP]:~/.lv2/
```

## Port order
- `0` in_l
- `1` in_r
- `2` out_l
- `3` out_r
- `4` model
- `5` wow
- `6` flutter
- `7` saturate
- `8` failure
- `9` volume
- `10` dry
- `11` noise
- `12` aux_mode
- `13` aux_active
- `14` aux_onset
- `15` spread
- `16` dry_type
- `17` drop_bypass
- `18` snag_bypass
- `19` hum_bypass
- `20` hiss_level
- `21` mech_level
- `22` mech_type
- `23` crinkle_pop_level
- `24` input_gain_mode

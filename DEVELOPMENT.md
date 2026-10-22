# Development status

Current firmware: **1.2.11**

## Current state

The system is running stably with:

- GNSS/PPS time reference
- 10 MHz OCXO measurement
- DS3231M RTC aging calibration
- async web UI
- standalone logger
- OCXO trend and Allan deviation diagnostics
- PCNT read-race diagnostics

Recent long-run observations show:

- GPS lock stable
- RTC aging calibration converging near zero residual rate
- long-term OCXO mean close to nominal
- rare PCNT read inconsistencies isolated to the 10 MHz counter path
- 32 kHz counter path does not show the same raw read failures

## Known diagnostic issue

Rare unrepairable PCNT snapshots occur on the 10 MHz path.

Current diagnostics capture:

- A/B/C raw PCNT values
- A/B/C timing relationship
- whether the 32 kHz PCNT failed at the same time
- flash/logger timing context
- RTC calibration state at the first operating-phase fault

Version 1.2.11 additionally records the execution duration of each individual `pcnt_unit_get_count()` read to distinguish a true counter inconsistency from a timestamp/preemption artifact.

The measurement baseline is not advanced by rejected snapshots.

## Planned 1.3.0

### True OCXO holdover

During trusted GNSS operation:

- derive a slow rolling OCXO frequency estimate from valid long-window measurements
- prefer long windows such as 1000 s over short-term values
- learn the expected holdover uncertainty from observed long-term OCXO/reference behavior

At GNSS loss:

- freeze the last trusted UTC/PPS anchor and OCXO frequency model
- derive time from OCXO ticks rather than the ESP timer
- do not learn new frequency corrections without a trusted reference
- use RTC as an independent plausibility monitor
- grow reported uncertainty as holdover duration increases

NTP source identity:

- GNSS locked: **Stratum 1**, RefID `GNSS`
- validated OCXO holdover: **Stratum 2**, RefID `HOLD`
- no trustworthy reference: **Stratum 16**

The GNSS PPS/RMC second assignment has been checked continuously against external NTP and does not show a one-second offset.

### Switchable RTC calibration

Add a web checkbox for RTC calibration.

When enabled:

- current aging calibration and GPS synchronization behavior remains active

When disabled:

- do not modify the aging register
- do not resynchronize RTC seconds from GPS
- let the battery-backed RTC free-run

Display:

- free-run duration
- accumulated RTC minus GPS phase/error since free-run began

After restart, the battery-backed RTC remains available as a time source. If used for NTP before GNSS/OCXO validation, it must be advertised with appropriately worse trust/uncertainty than GNSS or OCXO holdover.

## Rule for changes

Do not modify working subsystems unless there is a measured fault or an explicit requirement. Development changes should remain narrow and diagnosable.

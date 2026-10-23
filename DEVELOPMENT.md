# Development status

Current firmware: **1.3.0**

## 1.2.12 LCD latency change

Runtime LCD output is now buffered and incremental.

- the four 16-character rows are composed in RAM
- unchanged characters are not sent over I2C
- at most four contiguous changed characters are written per loop iteration
- the LCD does not wait for the shared RTC/LCD I2C mutex; if the RTC owns the bus, the LCD update is deferred
- page changes are spread over multiple short transfers instead of one full 64-character refresh
- boot messages still use the original immediate writer before the measurement task starts

The existing `lcdMaxUs` diagnostic now measures one compose/service loop rather than a full 4-row refresh. Additional raw status fields expose LCD chunk duration, chunks, characters, and bus-busy skips.

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

## 1.3.0 implementation

1.2.12 was accepted as the stable base after about 16.5 hours of operation with all operating-phase PCNT error counters still at zero. The LCD/Core0 change remains unchanged.

### True OCXO holdover

Normal GPS-locked measurement remains PPS-only. 1.3.0 does **not** add extra PCNT reads while GPS is trusted.

During trusted GNSS operation:

- a slow rolling OCXO bias is learned from valid 300 s / 1000 s windows
- 1000 s is preferred whenever available
- learning is disabled during the 30-minute startup observation phase
- bias and residual variance are persisted together with the existing temperature model
- the learned model is frozen automatically once GPS lock is lost

At GNSS loss:

- Core1 starts independent 10 MHz PCNT polling
- the last trusted PPS/UTC + PCNT read becomes the holdover anchor
- 64-bit accumulated tick deltas provide the long-term time axis
- the ESP timer is used only to interpolate the short interval since the latest OCXO publish
- implausible PCNT deltas are retried once and rejected instead of moving the holdover baseline
- if no valid OCXO update is available for more than 2 seconds, OCXO holdover loses trusted status

Reported uncertainty separates:

- deterministic learned-frequency uncertainty
- residual/random frequency scatter

The random term uses a square-root-time accumulation model rather than multiplying raw sigma by total holdover time.

NTP source identity:

- GNSS locked: **Stratum 1**, RefID `GNSS`
- validated OCXO holdover: **Stratum 2**, RefID `OCXO`
- ESP fallback / no trustworthy reference: **Stratum 16**

The GNSS PPS/RMC second assignment has been checked continuously against the existing NTP comparisons and does not show a one-second offset.

### Switchable RTC calibration / free-run

The web configuration now includes a persistent RTC calibration checkbox.

When enabled:

- existing GPS synchronization remains active
- existing adaptive Aging calibration remains active

When disabled:

- no GPS time is written to the RTC
- no Aging register value is changed
- SQW vs GPS measurement continues
- an unwrapped accumulated free-run drift value is maintained
- free-run duration and drift are shown in the web status

The setting is stored in NVS and survives restart/firmware updates.

### New diagnostics

The status endpoint exposes:

- holdover learned bias
- holdover residual sigma
- holdover learning count
- holdover PCNT read rejects
- holdover readiness / counter health / age
- RTC AUTO vs FREE-RUN
- RTC free-run duration and accumulated drift

## Rule for changes

Do not modify working subsystems unless there is a measured fault or an explicit requirement. Development changes should remain narrow and diagnosable.

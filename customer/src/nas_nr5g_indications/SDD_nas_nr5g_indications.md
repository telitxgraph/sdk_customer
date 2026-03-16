# nas_nr5g_indications — Software Design Description

| Item       | Detail                       |
|------------|------------------------------|
| Module     | nas_nr5g_indications (TNS)   |
| Platform   | SDX72 / OpenWrt / aarch64    |
| QMI IDL    | network_access_service_v01   |
| Version    | 1.0.0                        |
| Date       | 2026-03-06                   |

---

## 1. Requirements

Retrieve NR5G SIB9 time synchronization data (UTC time, GPS time, SFN, NTA, leap seconds) from the modem via QMI NAS and deliver it to the application layer.

| ID   | Requirement                                                                      |
|------|----------------------------------------------------------------------------------|
| R-01 | Monitor NR5G service availability via `QMI_NAS_SYS_INFO_IND`                    |
| R-02 | Configure sync pulse generation via `QMI_NAS_SET_NR5G_SYNC_PULSE_GEN_REQ` only after NR5G service is confirmed |
| R-03 | Receive SIB9 time data via `QMI_NAS_NR5G_TIME_SYNC_PULSE_REPORT_IND`           |
| R-04 | Detect sync loss via `QMI_NAS_NR5G_LOST_FRAME_SYNC_IND` and log the reason     |
| R-05 | Allow CLI configuration of `pulse_period`, `start_sfn`, `report_period`          |
| R-06 | Graceful shutdown: stop pulse generation before releasing QMI clients            |

---

## 2. Design Approach

### 2.1 Architecture

Two QMI NAS client instances on separate threads:

```
Main Thread ──┬── NAS Thread (Client #1)
              │     Register: sys_info, serving_system
              │     On NR5G srv_status==0x02 → signal condvar
              │
              └── Sync Pulse Thread (Client #2)
                    Wait on condvar → SET_NR5G_SYNC_PULSE_GEN
                    Receive: TIME_SYNC_PULSE_REPORT_IND
                    Receive: LOST_FRAME_SYNC_IND
```

Rationale for two clients: each has its own indication callback, cleanly separating NAS monitoring from sync pulse data reception.

### 2.2 Synchronization

- NAS Thread sets `g_nr5g_ready = 1` and calls `pthread_cond_signal()` when NR5G service is available.
- Sync Pulse Thread blocks on `pthread_cond_timedwait()` (5s timeout) until signaled.
- `g_running` flag (set to 0 on SIGINT/SIGTERM/ENTER) terminates all threads.

### 2.3 QMI Messages

| Direction   | Message                                          | Purpose                     |
|-------------|--------------------------------------------------|-----------------------------|
| App → Modem | `QMI_NAS_INDICATION_REGISTER_REQ`                | Subscribe to indications    |
| App → Modem | `QMI_NAS_SET_NR5G_SYNC_PULSE_GEN_REQ`           | Start/stop pulse generation |
| Modem → App | `QMI_NAS_SYS_INFO_IND`                           | NR5G service status         |
| Modem → App | `QMI_NAS_SERVING_SYSTEM_IND`                     | Registration state, PLMN    |
| Modem → App | `QMI_NAS_NR5G_TIME_SYNC_PULSE_REPORT_IND`       | SIB9 time sync data         |
| Modem → App | `QMI_NAS_NR5G_LOST_FRAME_SYNC_IND`              | Frame sync lost reason      |

---

## 3. Implementation

### 3.1 Source Files

| File                            | Role                                      |
|---------------------------------|-------------------------------------------|
| `nas_nr5g_indications.c`        | QMI init, indication callbacks, main loop |
| `nas_nr5g_indications.h`        | Types, logging macros, constants          |
| `nas_nr5g_indications_config.c` | Default sync pulse config values          |

### 3.2 Initialization Sequence

```
main()
  ├── tns_config_set_defaults()
  ├── CLI input (pulse_period, start_sfn, report_period)
  ├── pthread_create → tns_nas_qmi_start()
  │     ├── qmi_client_init_instance()          // NAS Client #1
  │     └── tns_register_nas_indications()
  │           → sys_info, sig_info, serving_system, operator_name
  ├── pthread_create → tns_sync_pulse_qmi_start()
  │     ├── qmi_client_init_instance()          // NAS Client #2
  │     ├── tns_register_sync_pulse_indications()
  │     │     → nr5g_time_sync_pulse_report, nr5g_lost_sync_frame
  │     ├── Wait for g_nr5g_ready (condvar)
  │     └── tns_set_nr5g_sync_pulse()           // retry x3
  └── Wait for ENTER / signal → tns_qmi_release()
```

### 3.3 Sync Pulse Parameters

| Parameter             | CLI | Range  | Unit   | Default | Description                        |
|-----------------------|-----|--------|--------|---------|------------------------------------|
| `pulse_period`        | Y   | 0–128  | x10 ms | 10      | Pulse period. 0 = stop.           |
| `start_sfn`           | Y   | 0–1024 | SFN    | 1024    | Start SFN. 1024 = next available. |
| `report_period`       | Y   | 0–128  | x10 ms | 10      | Report period. 0 = disabled.      |
| `pulse_align_type`    | N   | 0–1    | enum   | 0       | 0=NR5G frame, 1=UTC second.       |
| `pulse_trigger_action`| N   | 0–1    | enum   | 0       | 0=Trigger, 1=Skip.                |
| `pulse_get_cxo_count` | N   | 0–1    | bool   | 0       | 1=Include CXO count in report.    |

### 3.4 Received Sync Pulse Report Fields

| Field         | Type   | Description                          |
|---------------|--------|--------------------------------------|
| `sfn`         | uint32 | System Frame Number                  |
| `nta`         | int32  | Timing Advance                       |
| `nta_offset`  | uint32 | NTA offset                           |
| `leapseconds` | uint32 | UTC leap seconds                     |
| `utc_time`    | uint64 | UTC time in nanoseconds (from SIB9)  |
| `gps_time`    | uint64 | GPS time in nanoseconds              |
| `cxo_count`   | uint64 | CXO counter (if requested)           |

### 3.5 Lost Frame Sync Reasons

| Reason          | Trigger                       |
|-----------------|-------------------------------|
| `RLF`           | Radio Link Failure            |
| `HANDOVER`      | Handover in progress          |
| `RESELECTION`   | Cell reselection              |
| `OOS`           | Out of Service                |
| `STALE_SIB9`    | SIB9 data expired             |
| `NO_SIB9`       | Cell does not broadcast SIB9  |

### 3.6 Error Handling

| Scenario                  | Action                                            |
|---------------------------|---------------------------------------------------|
| QMI init / register fail  | Log, thread exits                                 |
| Sync pulse config fail    | Retry 3x (3s interval)                            |
| QMI service error in CB   | Log only (no `qmi_client_release` in CB context)  |
| Indication decode fail    | Log, skip                                         |
| NR5G service lost         | Reset `g_nr5g_ready`, wait for re-signal          |
| SIGINT / SIGTERM          | `g_running = 0`, graceful shutdown                |

### 3.7 Build

```bash
make package/nas_nr5g_indications/compile V=s
```

Linked libraries: `libqmiidl`, `libqmiservices`, `libqmi_cci`, `libqmi_client_qmux`, `libdiag`, `libpthread`, `librt`

---

## 4. Results

### 4.1 Execution

```bash
adb shell /usr/bin/nas_nr5g_indications
```

### 4.2 Expected Output

**NR5G service detected:**
```
[INFO ] [NR5G] Service Status: 2 (0=NoSrv,1=Limited,2=Srv)
[INFO ] NR5G service is available, signaling sync pulse thread
```

**Sync pulse configured:**
```
[INFO ] NR5G sync pulse generation configured successfully
```

**SIB9 time sync report received:**
```
[INFO ] === NR5G Time Sync Pulse Report ===
[INFO ] INFO: sfn = 512
[INFO ] INFO: nta = -1234
[INFO ] INFO: leapseconds = 18
[INFO ] INFO: utc_time = 1741267200000000000
[INFO ] INFO: gps_time = 1741267218000000000
[INFO ] ===================================
```

**Frame sync lost:**
```
[ERROR] NR5G Lost Frame Sync: reason=HANDOVER (1)
```

### 4.3 Verification Checklist

| #  | Test Case                       | Expected Result                                           | Pass/Fail |
|----|---------------------------------|-----------------------------------------------------------|-----------|
| 1  | App launch with valid input     | Both QMI threads initialized                             |           |
| 2  | NR5G service available          | `SYS_INFO_IND` decoded, sync pulse thread signaled       |           |
| 3  | Sync pulse configured           | `SET_NR5G_SYNC_PULSE_GEN` success                        |           |
| 4  | Pulse report received           | `TIME_SYNC_PULSE_REPORT_IND` with UTC/GPS time           |           |
| 5  | Frame sync lost                 | `LOST_FRAME_SYNC_IND` with reason code                   |           |
| 6  | NR5G service lost and recovered | `g_nr5g_ready` resets, re-signals on recovery            |           |
| 7  | Graceful shutdown (ENTER/Ctrl+C)| Pulse stopped (period=0), QMI clients released           |           |
| 8  | Config retry on failure         | Retries 3x with 3s interval                              |           |

### 4.4 Log Monitoring

```bash
logread -f | grep nas_nr5g_indications
```

Requires `FEATURE_ENABLE_LOGGING_TO_SYSLOG` at compile time.

---

**End of Document**

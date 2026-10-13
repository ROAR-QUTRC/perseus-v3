# can_logger

Logs every CAN frame seen on an interface to a single CSV file, for offline
analysis when something goes wrong.

```sh
ros2 run can_logger can_logger --iface can0
ros2 run can_logger can_logger --iface can0 --out /data/run1.csv
```

Default output is `~/can_logs/can_log_<YYYYmmdd_HHMMSS>.csv`. Frames are
buffered in memory (15000 lines) and written when the buffer fills, every 2 s,
and on Ctrl+C/SIGTERM, so a crash loses at most ~2 s of data.

Columns: `timestamp_us` (wall clock, µs since epoch), `id_hex`, the decoded
hi-can `system,subsystem,device,group,parameter` fields, `rtr`, `extended`,
`dlc`, `data_hex`.

```python
import pandas as pd
df = pd.read_csv("can_log.csv")
```

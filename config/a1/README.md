# A.1.1 approach configurations

Each JSON file is currently an exact copy of
`config/ics_clear_weather_pid.json`. The separate files make it possible to
tune and preserve PID gains independently for every A.1.1 run.

The wind and visibility conditions are configured in IOS; they are not stored
in these controller JSON files.

Assuming runway heading `64 deg`, use the following setup:

| Run | Configuration file | Condition | Wind Dir | Wind Speed | Relative aircraft |
| ---: | --- | --- | ---: | ---: | ---: |
| 1 | `01_calm.json` | Calm | `0 deg` | `0 kt` | N/A |
| 2 | `02_crosswind_left_5ms.json` | Crosswind from left, 5 m/s | `334 deg` | `9.7 kt` | `270 deg` |
| 3 | `03_crosswind_right_5ms.json` | Crosswind from right, 5 m/s | `154 deg` | `9.7 kt` | `90 deg` |
| 4 | `04_crosswind_left_10ms.json` | Crosswind from left, 10 m/s | `334 deg` | `19.4 kt` | `270 deg` |
| 5 | `05_crosswind_right_10ms.json` | Crosswind from right, 10 m/s | `154 deg` | `19.4 kt` | `90 deg` |
| 6 | `06_crosswind_left_15ms.json` | Crosswind from left, 15 m/s | `334 deg` | `29.2 kt` | `270 deg` |
| 7 | `07_crosswind_right_15ms.json` | Crosswind from right, 15 m/s | `154 deg` | `29.2 kt` | `90 deg` |
| 8 | `08_gust_left_7_to_12ms.json` | Gusts from left, 7 to 12 m/s | `334 deg` | `13.6` to `23.3 kt` | `270 deg` |
| 9 | `09_gust_right_7_to_12ms.json` | Gusts from right, 7 to 12 m/s | `154 deg` | `13.6` to `23.3 kt` | `90 deg` |
| 10 | `10_windshear_headwind_10_to_0ms.json` | Headwind shear, 10 to 0 m/s | `64 deg` | `19.4` to `0 kt` | `0 deg` |
| 11 | `11_headwind_10ms.json` | Headwind, 10 m/s | `64 deg` | `19.4 kt` | `0 deg` |
| 12 | `12_tailwind_5ms.json` | Tailwind, 5 m/s | `244 deg` | `9.7 kt` | `180 deg` |
| 13 | `13_low_visibility_calm.json` | RVR 300 m, calm | `0 deg` | `0 kt` | N/A |

Example for run A.1.1/2:

```powershell
python tools\run_ics_pid.py --config config\a1\02_crosswind_left_5ms.json --send --duration 300 --dashboard --check-a11-criteria
```

The criteria monitor stops control at `300 ft` radio altitude and reports
`PASS`, `FAIL`, or `INCOMPLETE`.

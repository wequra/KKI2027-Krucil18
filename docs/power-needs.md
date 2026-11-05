# Software power needs (for the electrical division)

Status: ESTIMATES, to be replaced with measured values once the hardware list is fixed.
Send to the electrical division by Fri 6 Nov.

| Item | Typical draw | Peak | Notes |
|---|---|---|---|
| Companion computer (Jetson class) | 10 W | 15-25 W | Depends on model and power mode, YOLO inference is the peak. TO CONFIRM model |
| Camera (USB/depth) | 2 W | 3.5 W | TO CONFIRM model (keep consistent with the electrical list) |
| Telemetry radio | 1 W | 2 W | TO CONFIRM band (avoid conflicting frequencies in the proposal) |
| Onboard Wi-Fi AP / router | 3 W | 5 W | |
| Flight controller + GPS | 2 W | 3 W | |
| **Total software-side load** | **~18 W** | **~38 W** | |

Questions for electrical:
1. Which rail voltages are available (5 V / 12 V / 19 V) for the companion computer?
2. Battery capacity target and mission length (assume 20-30 min run plus margin)?
3. Is the camera USB-powered from the companion computer or a separate rail?

Energy estimate: 38 W peak x 0.5 h = 19 Wh worst case for the software load (excluding propulsion).

# SSG-AI: SmartStreet Geo-Enforce AI — Simulation Data and Code

This repository contains the simulation scripts and output data supporting the results reported in:

> **Adnan Adel Bitar**, "SmartStreet Geo-Enforce AI (SSG-AI): A Secure, Safety-Gated, Token-Based Variable Speed Limit Enforcement Architecture with Multi-Scenario SUMO Validation," *Vehicular Communications*, Elsevier, submitted June 2026.

All simulations were run using [SUMO (Simulation of Urban MObility)](https://eclipse.dev/sumo/) with the TraCI Python interface.

---

## Repository Structure

```
SSG-AI-Simulation/
│
├── scenario_A/                          ← Urban 80 km/h VSL (Table II in paper)
│   ├── scenario_a_urban.py              ← Simulation script — Run 1 (15% aggressive)
│   ├── setup_urban_files.py             ← Network setup helper script
│   ├── urban_a_log_test1.csv            ← Per-vehicle log — Run 1 (15% aggressive)
│   └── urban_a_log_testC_realistic.csv  ← Per-vehicle log — Run 2 (25% aggressive)
│
├── scenario_B/                          ← Highway Fog VSL, Normal Communications (Table III)
│   ├── vsl_highway_dynamic_B.py         ← Simulation script (PACKET_LOSS_RATE = 0.0)
│   ├── vsl_highway_log_B.csv            ← Per-vehicle, per-step log
│   ├── tcc_decision_log_B.csv           ← Per-step AI-TCC decision log
│   └── tcc_run_summary_B.txt            ← Full run summary
│
├── scenario_C/                          ← Highway Fog VSL, 50% Packet Loss (Table IV)
│   ├── vsl_highway_dynamic_C.py         ← Simulation script (PACKET_LOSS_RATE = 0.50)
│   ├── vsl_highway_log_C.csv            ← Per-vehicle, per-step log
│   ├── tcc_decision_log_C.csv           ← Per-step AI-TCC decision log
│   └── tcc_run_summary_C.txt            ← Full run summary
│
└── README.md
```

---

## Scenario Descriptions

### Scenario A — Urban 80 km/h VSL (Table II)
A 5 km urban road with 100 vehicles under a fixed 80 km/h VSL geofence. Two driver-composition runs:
- **Run 1** (15% aggressive mix): `urban_a_log_test1.csv`
- **Run 2** (25% aggressive mix): `urban_a_log_testC_realistic.csv`

### Scenario B — Highway Fog VSL, Normal Communications (Table III)
A highway with 200 vehicles. The AI-TCC monitors live fog severity and highway occupancy each simulation step and issues `ENFORCE_SPEED_TOKEN` updates in real time, reducing the speed limit from 140 km/h to a 60 km/h floor as fog thickens, then recovering back to 140 km/h as fog clears. No packet loss. 8 token updates issued across 2 episodes.

Key results: 0 collisions, CII = 100%, 53/100 non-compliant drivers forced into compliance.

### Scenario C — Highway Fog VSL, Adverse Communications (Table IV)
Identical setup to Scenario B with `PACKET_LOSS_RATE = 0.50` and `MAX_LATENCY_STEPS = 5`. Reproduces the 50% token packet loss adversarial communications test.

Key results: 0 collisions, CII = 100%, 31/100 non-compliant drivers forced into compliance, 128/279 tokens lost (45.9% actual loss rate).

---

## How to Run

### Requirements
- [SUMO](https://eclipse.dev/sumo/) (tested with SUMO 1.18+)
- Python 3.8+
- Python packages: `traci`, `pandas`, `numpy`

### Running Scenarios B and C

1. Place the script in the same folder as `highway_vsl.sumocfg` and the network XML files.
2. To run **Scenario B** (normal communications), ensure at the top of the script:
```python
   PACKET_LOSS_RATE = 0.0
   MAX_LATENCY_STEPS = 0
```
3. To run **Scenario C** (50% packet loss), set:
```python
   PACKET_LOSS_RATE = 0.50
   MAX_LATENCY_STEPS = 5
```
4. Run:
```bash
   python vsl_highway_dynamic_B.py   # or vsl_highway_dynamic_C.py
```
5. Outputs (`vsl_highway_log.csv`, `tcc_decision_log.csv`, `tcc_run_summary.txt`) are written to the same folder as the script.

### Running Scenario A
```bash
cd scenario_A
python scenario_a_urban.py          # Run 1 (15% aggressive)
python setup_urban_files.py         # Run 2 setup, then scenario_a_urban.py
```

---

## Output File Descriptions

| File | Description |
|------|-------------|
| `vsl_highway_log_*.csv` | Per-vehicle, per-step log: vehicle ID, speed, overspeed flag, TTC check result, decel intent sent, non-compliant flag |
| `tcc_decision_log_*.csv` | Per-step AI-TCC log: fog severity, highway occupancy, computed target speed, enforced edge speed, token issued flag |
| `tcc_run_summary_*.txt` | Full run summary: token issuance steps, three-phase speed statistics, compliance count, packet loss rate, CII |
| `urban_a_log_*.csv` | Per-vehicle, per-step log for urban scenario: speed, overspeed flag, TTC check, decel intent, vehicle type |

---

## Key Results Summary

| Scenario | Vehicles | Collisions | CII | Forced Compliance | Tokens Lost |
|----------|----------|-----------|-----|-------------------|-------------|
| A — Run 1 (15% aggressive) | 100 | 0 | 100% | 4 / 100 | N/A |
| A — Run 2 (25% aggressive) | 100 | 0 | 100% | 14 / 100 | N/A |
| B — Normal comms | 200 | 0 | 100% | 53 / 100 | 0 / 275 |
| C — 50% packet loss | 200 | 0 | 100% | 31 / 100 | 128 / 279 (45.9%) |

---

## Citation

If you use this data or code, please cite the associated paper (full citation will be updated upon publication).

## Contact

Adnan Adel Bitar — adnan.bitar.4m@gmail.com  
Applied Technology Schools, Al Ain, Abu Dhabi, UAE

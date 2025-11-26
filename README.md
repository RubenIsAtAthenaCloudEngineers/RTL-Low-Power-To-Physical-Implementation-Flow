# RTL-Low-Power-To-Physical-Implementation-Flow
This is a low power design flow from the perspective of RTL correlated to physical implementation.

````markdown
# TSMC N3 Low-Power Design Flow — Eye-Tracking Feature Engine (AR/VR)

This repository contains a complete **RTL → Power Analysis → Physical Implementation → Signoff** flow for the **Eye-Tracking Feature Engine** used in AR/VR applications.

The flow demonstrates industry-standard low-power techniques using:

- Synopsys **RTL Architect**
- Synopsys **PrimePower-RTL**
- Synopsys **Fusion Compiler**
- Synopsys **PrimeTime-PX**
- Synopsys **VCS**
- Synopsys **Verdi**
- **Python 3** for automation/post-processing

Target technology:

- **Foundry:** TSMC
- **Node:** N3 (3 nm)
- **VDD:** 0.8 V (nominal)
- **Temperature:** 25 °C
- **Core frequency:** 500 MHz

Power budgets for the Eye-Tracking Feature Engine:

| Mode       | Max Power Target            |
|------------|-----------------------------|
| ACTIVE     | ≤ 80 mW                     |
| IDLE       | ≤ 10 mW                     |
| DEEP_SLEEP | < 1 mW (dominated by leak) |

---

## 1. Block Description

### Eye-Tracking Feature Engine

- **Function:**  
  Takes camera frames and extracts pupil/eye features to generate a gaze vector.

- **Specifications:**
  - Core frequency: **500 MHz**
  - Supply voltage: **0.8 V**
  - Modes:
    - `ACTIVE`
    - `IDLE`
    - `DEEP_SLEEP`
  - Interfaces:
    - AXI(-Lite) for configuration
    - Camera pixel stream input
    - Gaze vector (x, y) output

---

## 2. Repository Structure

```text
eyetrack_n3_lowpower_flow/
├── rtl/
│   ├── eyetrack_top.sv
│   ├── feature_engine.sv
│   ├── axi_regs.sv
│   └── filelist_rtl.f
├── tb/
│   └── tb_eyetrack.sv
├── constraints/
│   ├── eyetrack.sdc
│   └── eyetrack_n3_500MHz.sdc
├── power/
│   └── eyetrack.upf
├── lib/
│   ├── tsmcN3_0p80V_tt_25c.lib
│   ├── tsmcN3_0p72V_ff_0c.lib
│   └── tsmcN3_0p88V_ss_125c.lib
├── sim/
│   ├── eyetrack_rtl.fsdb
│   ├── eye_active_rtl.saif
│   ├── eye_idle_rtl.saif
│   ├── eye_sleep_rtl.saif
│   ├── eyetrack_gate.fsdb
│   ├── eye_active_gl.saif
│   ├── eye_idle_gl.saif
│   └── eye_sleep_gl.saif
├── impl/
│   ├── eyetrack_fc_final.v
│   ├── eyetrack_fc_final.def
│   └── eyetrack_route.spef
├── reports/
│   ├── pp_rtl_*.rpt
│   ├── rtlarch_*_summary.rpt
│   ├── rtlarch_*_power_hier.rpt
│   ├── fc_route_active_power.rpt
│   ├── fc_route_active_timing_pba.rpt
│   ├── ptpx_active_power.rpt
│   ├── ptpx_idle_power.rpt
│   ├── ptpx_sleep_power.rpt
│   ├── ptpx_active_timing.rpt
│   ├── rtlarch_sweep_summary.csv
│   └── ptpx_budget_summary.txt
└── scripts/
    ├── rtl_arch_sweep.tcl
    ├── fc_run_eyetrack.tcl
    ├── ptpx_run_eyetrack.tcl
    ├── summarize_rtlarch.py
    └── check_ptpx_budgets.py
````

---

## 3. Key Input Collateral

### 3.1 RTL (SystemVerilog)

Location: `rtl/`

Top-level: `eyetrack_top.sv` (simplified example)

```systemverilog
module eyetrack_top #(
  parameter AXI_ADDR_WIDTH = 32,
  parameter AXI_DATA_WIDTH = 64
)(
  input  logic                     clk,        // core clock (N3, 0.8 V)
  input  logic                     rst_n,

  // 00=ACTIVE, 01=IDLE, 10=DEEP_SLEEP
  input  logic [1:0]               mode_sel,

  // AXI-lite control (simplified)
  input  logic                     s_axi_aclk,
  input  logic                     s_axi_aresetn,
  input  logic                     s_axi_awvalid,
  input  logic [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
  input  logic                     s_axi_wvalid,
  input  logic [AXI_DATA_WIDTH-1:0] s_axi_wdata,
  output logic                     s_axi_awready,
  output logic                     s_axi_wready,

  // Camera pixel in
  input  logic                     cam_valid,
  input  logic [15:0]              cam_pixel,

  // Gaze vector out
  output logic                     gaze_valid,
  output logic [15:0]              gaze_x,
  output logic [15:0]              gaze_y
);

  // Mode decode
  logic active_mode, idle_mode, sleep_mode;
  assign active_mode = (mode_sel == 2'b00);
  assign idle_mode   = (mode_sel == 2'b01);
  assign sleep_mode  = (mode_sel == 2'b10);

  // Datapath enable – candidate for automatic clock gating
  logic dp_en;
  assign dp_en = active_mode & ~sleep_mode;

  logic [15:0] pixel_reg;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pixel_reg <= '0;
    end else if (dp_en && cam_valid) begin
      pixel_reg <= cam_pixel;
    end
  end

  logic        gaze_valid_int;
  logic [15:0] gaze_x_int, gaze_y_int;

  feature_engine u_feature_engine (
    .clk        (clk),
    .rst_n      (rst_n),
    .enable     (dp_en),
    .pixel_in   (pixel_reg),
    .gaze_valid (gaze_valid_int),
    .gaze_x     (gaze_x_int),
    .gaze_y     (gaze_y_int)
  );

  // DEEP_SLEEP: clamp outputs
  assign gaze_valid = sleep_mode ? 1'b0 : gaze_valid_int;
  assign gaze_x     = sleep_mode ? 16'd0 : gaze_x_int;
  assign gaze_y     = sleep_mode ? 16'd0 : gaze_y_int;

endmodule
```

### 3.2 SDC (Synopsys Design Constraints)

Location: `constraints/eyetrack.sdc`

```tcl
# Core clock – 500 MHz (2.0 ns) at 0.8 V
create_clock -name core_clk -period 2.0 [get_ports clk]

# AXI-lite clock – 200 MHz
create_clock -name axi_clk -period 5.0 [get_ports s_axi_aclk]

# Basic I/O delays
set_input_delay  0.3 -clock core_clk \
  [get_ports {cam_valid cam_pixel[*]}]
set_output_delay 0.3 -clock core_clk \
  [get_ports {gaze_valid gaze_x[*] gaze_y[*]}]

# Operating condition (N3 TT, 0.8 V, 25 C)
set_operating_conditions -analysis_type on_chip_variation \
  [get_operating_conditions tsmcN3_0p80V_tt_25c]

# Simple CDC exception
set_false_path -from [get_clocks axi_clk] -to [get_clocks core_clk]
```

### 3.3 UPF (Unified Power Format)

Location: `power/eyetrack.upf`

```tcl
create_supply_port VDD
create_supply_port VSS
create_supply_net  VDD
create_supply_net  VSS
connect_supply_net VDD -ports VDD
connect_supply_net VSS -ports VSS

create_power_domain PD_TOP      -include_scope eyetrack_top
create_power_domain PD_EYE_CORE -elements {eyetrack_top/u_feature_engine}

create_supply_set SS_EYE_CORE \
  -primary_power_net VDD -primary_ground_net VSS
set_domain_supply_net PD_EYE_CORE -primary SS_EYE_CORE

create_power_state ACTIVE     -domain PD_EYE_CORE -values {SS_EYE_CORE=on}
create_power_state IDLE       -domain PD_EYE_CORE -values {SS_EYE_CORE=on}
create_power_state DEEP_SLEEP -domain PD_EYE_CORE -values {SS_EYE_CORE=off}

set_isolation ISO_EYE_OUT \
  -domain PD_EYE_CORE \
  -applies_to outputs \
  -isolation_signal sleep_mode \
  -clamp_value 0
```

---

## 4. Toolchain

Synopsys tools (2024/2025 W-series, example):

* **VCS** – RTL & gate-level simulation, FSDB + SAIF
* **Verdi** – waveform debug and low-power analysis using FSDB/UPF
* **PrimePower-RTL** – early RTL dynamic/leakage power estimation
* **RTL Architect** – RTL-level, physically-aware PPA (timing/power/area)
* **Fusion Compiler** – unified synthesis & place/route with in-design power optimization
* **PrimeTime-PX** – STA + dynamic/leakage power signoff

Non-Synopsys:

* **Python 3** – combines multiple reports into CSV/txt summaries (budget checks, sweeps).

---

## 5. High-Level Flow

### 5.1 End-to-End Low-Power Flow (TSMC N3, 0.8 V)

```text
RTL + UPF + SDC
      │
      ▼
(1) VCS + Verdi (RTL Simulation)
      │ FSDB + RTL SAIF (ACTIVE / IDLE / SLEEP)
      ▼
(2) PrimePower-RTL
      │ Early RTL dynamic/leak power
      ▼
(3) RTL Architect
      │ RTL PPA sweeps (freq / Vdd / Vt)
      │ Low-power guidance (clock gating, operand isolation)
      ▼
(4) Fusion Compiler
      │ N3 implementation + opt_power
      ▼
(5) VCS + Verdi (Gate-Level Simulation)
      │ Gate FSDB + Gate SAIF
      ▼
(6) PrimeTime-PX
      │ Timing + power signoff (ACTIVE / IDLE / SLEEP)
      ▼
(7) Python scripts
      │ Budget summary vs 80 / 10 / < 1 mW
      ▼
Final PASS / FAIL report
```

### 5.2 Optimization Loop

```text
      +--------------------+
      |  RTL Architect     |
      |  (RTL PPA, N3)     |
      +---------+----------+
                |
                v
      +---------+----------+
      |  Fusion Compiler   |
      |  (impl, opt_power) |
      +---------+----------+
                |
                v
      +---------+----------+
      |  VCS + Verdi       |
      |  gate SAIF / FSDB  |
      +---------+----------+
                |
                v
      +---------+----------+
      |  PrimeTime-PX      |
      |  timing + power    |
      +---------+----------+
                |
                v
      +---------+----------+
      |  Python summary    |
      |  80 / 10 / <1 mW   |
      +---------+----------+
                |
         PASS <-+-> FAIL
                 |
                 v
        Back to RTL / UPF /
        implementation tweaks
```

---

## 6. Step-by-Step Flow

### Step 1 — RTL Simulation (VCS + Verdi)

**Goal:**
Generate:

* `eyetrack_rtl.fsdb` for debug
* `eye_active_rtl.saif`, `eye_idle_rtl.saif`, `eye_sleep_rtl.saif` for RTL power

Example compile/run (bash):

```bash
vcs -full64 -sverilog -debug_access+all \
    -f rtl/filelist_rtl.f \
    -top tb_eyetrack \
    -o sim_rtl

./sim_rtl +mode=ACTIVE +define=MODE_ACTIVE
./sim_rtl +mode=IDLE   +define=MODE_IDLE
./sim_rtl +mode=SLEEP  +define=MODE_SLEEP
```

Verdi debug:

```bash
verdi -ssf sim/eyetrack_rtl.fsdb -top tb_eyetrack &
```

---

### Step 2 — Early RTL Power (PrimePower-RTL)

**Goal:**
Estimate **dynamic + leakage power** at RTL per mode using SAIF.

Run (example):

```bash
primepower -f scripts/pp_rtl_run.tcl
```

Outputs:

* `reports/pp_rtl_active_power.rpt`
* `reports/pp_rtl_idle_power.rpt`
* `reports/pp_rtl_sleep_power.rpt`

---

### Step 3 — RTL PPA Sweeps (RTL Architect)

**Goal:**

* Predict timing, power, area, congestion at RTL.
* Sweep frequency and/or Vdd/Vt.
* Identify RTL-level low-power opportunities.

Run:

```bash
dc_shell -f scripts/rtl_arch_sweep.tcl
```

Outputs:

* `reports/rtlarch_*_summary.rpt`
* `reports/rtlarch_*_power_hier.rpt`

Low-power actions informed here:

* Improve clock gating around `dp_en`.
* Add operand isolation for inactive datapaths.
* Pipeline or restructure hot blocks inside `feature_engine`.

---

### Step 4 — Summarize RTL Sweeps (Python)

**Goal:**
Produce a compact table of:

* Scenario (corner, frequency)
* Mode (ACTIVE / IDLE / SLEEP)
* Total power (mW)
* WNS (ns)
* PASS/FAIL vs budgets

Run:

```bash
python3 scripts/summarize_rtlarch.py
```

Output:

* `reports/rtlarch_sweep_summary.csv`

Use this to select the **nominal N3 0.8 V, 500 MHz** scenario and finalize RTL.

---

### Step 5 — Implementation (Fusion Compiler)

**Goal:**

* N3 implementation (synthesis + place + CTS + route)
* Low-power optimization (`opt_power`, clock gating refinement, MBFF, glitch reduction)

Run:

```bash
fc_shell -f scripts/fc_run_eyetrack.tcl
```

Outputs:

* `impl/eyetrack_fc_final.v`
* `impl/eyetrack_fc_final.def`
* `impl/eyetrack_route.spef`
* `reports/fc_route_active_power.rpt`
* `reports/fc_route_active_timing_pba.rpt`

---

### Step 6 — Gate-Level Simulation (VCS + Verdi)

**Goal:**

* Generate **gate-level FSDB** and **gate-level SAIF** for final signoff.

Run:

```bash
vcs -full64 -sverilog -debug_access+all \
    -f filelist_gl.f \
    -top tb_eyetrack \
    -o sim_gate

./sim_gate +mode=ACTIVE +define=MODE_ACTIVE
./sim_gate +mode=IDLE   +define=MODE_IDLE
./sim_gate +mode=SLEEP  +define=MODE_SLEEP

verdi -ssf sim/eyetrack_gate.fsdb -top tb_eyetrack &
```

Outputs:

* `sim/eyetrack_gate.fsdb`
* `sim/eye_active_gl.saif`
* `sim/eye_idle_gl.saif`
* `sim/eye_sleep_gl.saif`

---

### Step 7 — Timing + Power Signoff (PrimeTime-PX)

**Goal:**

* STA + dynamic + leakage power signoff at 500 MHz, 0.8 V.
* Verify compliance with 80 / 10 / <1 mW budgets.

Run:

```bash
pt_shell -f scripts/ptpx_run_eyetrack.tcl
```

Outputs:

* `reports/ptpx_active_timing.rpt`
* `reports/ptpx_active_power.rpt`
* `reports/ptpx_idle_power.rpt`
* `reports/ptpx_sleep_power.rpt`

---

### Step 8 — Final Budget Summary (Python)

**Goal:**

* Compare PrimeTime-PX power results to budgets.
* Mark each mode as PASS/FAIL.

Run:

```bash
python3 scripts/check_ptpx_budgets.py
```

Outputs:

* `reports/ptpx_budget_summary.txt`
* `reports/ptpx_budget_summary.csv`

This summary is what you would highlight in a design review.

---

## 7. How to Use This Repo in an Interview Context

You can present this repository as:

1. A concrete **TSMC N3 low-power design flow** for an AR/VR Eye-Tracking block.
2. Proof that you understand:

   * RTL → SAIF generation (VCS/Verdi)
   * RTL power estimation (PrimePower-RTL)
   * RTL PPA and low-power architecture (RTL Architect)
   * Physical implementation with low-power options (Fusion Compiler)
   * Gate-level SAIF + FSDB for debug (VCS/Verdi)
   * Timing + power signoff (PrimeTime-PX)
   * Automation using Python for sweeps and budget checking

You can share the GitHub URL and point reviewers directly to:

* `README.md` for the full story.
* `scripts/` for automation.
* `reports/ptpx_budget_summary.txt` for final PASS/FAIL vs 80 / 10 / <1 mW.

---

## 8. License

Internal engineering educational reference.
No proprietary EDA install scripts or PDK contents are included in this example.

```
::contentReference[oaicite:0]{index=0}
```

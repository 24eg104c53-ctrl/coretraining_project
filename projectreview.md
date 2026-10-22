
# Fixed-Time Two-Road Traffic Light Controller: RTL-to-GDSII Project Report


## 1. Project Overview

This project implements a digital traffic light controller for a two-road intersection consisting of a North-South road and an East-West road. The design follows the complete RTL-to-GDSII flow, including Verilog RTL design, simulation, synthesis, gate-level simulation, physical design, sign-off checks, timing analysis, and final GDSII layout generation.

The design uses standard cells only, without macros, memories, SRAM/ROM, IP blocks, or analog components. It targets a 100 µm × 100 µm die using the SkyWater Sky130 `sky130_fd_sc_hd` process.

**Project summary:** The traffic light controller takes a clock and reset as inputs and generates six traffic-light outputs. It uses a four-state finite state machine (FSM) and a timer counter to control the traffic signal sequence.

## 2. Real-World Application

**Problem:** At a junction where two roads cross, giving green signals to both roads simultaneously can cause collisions. The controller must ensure that when one road has a green or yellow signal, the other road remains red. The signals must change in a fixed and safe order.

**Users and place of use:** This type of controller can be used at small road junctions, colony roads, industrial-area gates, and other locations where fixed-time traffic signalling is sufficient.

**Why dedicated hardware:** The safety rule that both roads must never receive green signals simultaneously is implemented directly through the FSM output logic. Each light is decoded from a particular state, ensuring that North-South green and East-West green cannot be active together.

**Example scenario:** When a bus arrives from the north and an auto arrives from the east, the North-South green state allows the bus to proceed while the auto waits at red. The North-South yellow state allows vehicles to clear the junction. The East-West green state then allows the auto to proceed while the bus waits at red. The East-West yellow state prepares the controller to return to North-South green.

After reset, the controller returns to North-South green and East-West red.

**Scope:** The project implements fixed-time traffic-light sequencing with a hardware-enforced safety rule. Vehicle sensors, pedestrian phases, and lamp-driver circuits are outside the current scope.

## 3. Specification

### Inputs and Outputs

| Signal | Direction | Width | Purpose |
|---|---|---|---|
| `clk` | Input | 1 | Clock for the FSM and counter |
| `rst` | Input | 1 | Synchronous, active-high reset |
| `ns_red` | Output | 1 | North-South red light |
| `ns_yellow` | Output | 1 | North-South yellow light |
| `ns_green` | Output | 1 | North-South green light |
| `ew_red` | Output | 1 | East-West red light |
| `ew_yellow` | Output | 1 | East-West yellow light |
| `ew_green` | Output | 1 | East-West green light |

The module has eight ports in total.

### Timing Parameters

- `GREEN_TIME = 5` clock cycles.
- `YELLOW_TIME = 2` clock cycles.
- One complete traffic cycle contains \(2 \times 5 + 2 \times 2 = 14\) clock cycles.

Both timing values are parameters and can be changed before synthesis.

### Safety Requirements

1. North-South green and East-West green must never be active simultaneously.
2. When North-South is green or yellow, East-West must be red.
3. When East-West is green or yellow, North-South must be red.
4. After reset, the controller must start in North-South green and East-West red.

## 4. Design and RTL

The design is a Moore finite state machine with a state register, next-state logic, output decoding, and a timer counter.

### FSM States

| State | Code | North-South | East-West | Duration |
|---|---|---|---|---|
| S0: NS_GREEN | `00` | Green | Red | `GREEN_TIME` |
| S1: NS_YELLOW | `01` | Yellow | Red | `YELLOW_TIME` |
| S2: EW_GREEN | `10` | Red | Green | `GREEN_TIME` |
| S3: EW_YELLOW | `11` | Red | Yellow | `YELLOW_TIME` |

The state sequence is:

`S0 → S1 → S2 → S3 → S0`

The FSM advances when the timer reaches its terminal count. Otherwise, it remains in the current state.

### Counter Operation

The counter counts upward during each state. The terminal count is `GREEN_TIME - 1` for green states and `YELLOW_TIME - 1` for yellow states.

When the counter reaches its terminal count, the FSM advances and the counter clears to zero. The counter width is calculated automatically from the larger timing parameter.

### Reset Operation

The reset is synchronous and active-high. When `rst = 1` at a rising clock edge, the state register returns to S0 and the counter clears to zero. The outputs become North-South green and East-West red.

### RTL Code (`traffic_light_controller.v`)

```verilog
`timescale 1ns/1ps

module traffic_light_controller #(
    parameter GREEN_TIME  = 5,
    parameter YELLOW_TIME = 2
)(
    input  wire clk,
    input  wire rst,

    output wire ns_red,
    output wire ns_yellow,
    output wire ns_green,
    output wire ew_red,
    output wire ew_yellow,
    output wire ew_green
);

    localparam [1:0] S0_NS_GREEN  = 2'b00,
                     S1_NS_YELLOW = 2'b01,
                     S2_EW_GREEN  = 2'b10,
                     S3_EW_YELLOW = 2'b11;

    localparam MAX_TIME =
        (GREEN_TIME > YELLOW_TIME) ? GREEN_TIME : YELLOW_TIME;

    localparam CNT_W =
        (MAX_TIME > 2) ? $clog2(MAX_TIME) : 1;

    reg [1:0] state, next_state;
    reg [CNT_W-1:0] cnt;

    wire state_is_green = ~state[0];

    wire [CNT_W-1:0] last_count =
        state_is_green ? (GREEN_TIME - 1) : (YELLOW_TIME - 1);

    wire timer_done = (cnt == last_count);

    always @(*) begin
        next_state = state;

        if (timer_done) begin
            case (state)
                S0_NS_GREEN:
                    next_state = S1_NS_YELLOW;

                S1_NS_YELLOW:
                    next_state = S2_EW_GREEN;

                S2_EW_GREEN:
                    next_state = S3_EW_YELLOW;

                S3_EW_YELLOW:
                    next_state = S0_NS_GREEN;

                default:
                    next_state = S0_NS_GREEN;
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= S0_NS_GREEN;
            cnt   <= {CNT_W{1'b0}};
        end
        else begin
            state <= next_state;

            if (timer_done)
                cnt <= {CNT_W{1'b0}};
            else
                cnt <= cnt + 1'b1;
        end
    end

    assign ns_green  = (state == S0_NS_GREEN);
    assign ns_yellow = (state == S1_NS_YELLOW);
    assign ns_red    = (state == S2_EW_GREEN) |
                       (state == S3_EW_YELLOW);

    assign ew_green  = (state == S2_EW_GREEN);
    assign ew_yellow = (state == S3_EW_YELLOW);
    assign ew_red    = (state == S0_NS_GREEN) |
                       (state == S1_NS_YELLOW);

endmodule
```

## 5. RTL Simulation and Verification

A self-checking Verilog testbench generates a 10 ns clock, applies reset, and samples the six traffic-light outputs on every falling clock edge.

The testbench checks:

- Both roads are never green simultaneously.
- North-South green or yellow implies East-West red.
- East-West green or yellow implies North-South red.
- Each road displays exactly one light.
- The decoded state matches the DUT state register.
- The state sequence and timing durations are correct.
- Reset returns the controller to North-South green.

The testbench runs four complete traffic cycles, applies a reset during East-West green, verifies the reset response, and runs another full cycle.

### RTL Simulation Results

The source document reports:

- 735 of 735 checks passed.
- Zero failures.
- Simulation ended at 841 ns.
- Additional tests using `GREEN_TIME / YELLOW_TIME = 1/1` and `3/1` also passed with zero failures.



<img width="1920" height="923" alt="trafficsimu" src="https://github.com/user-attachments/assets/0341c087-c4e2-41ea-9004-2c72f7c7e643" />


*Figure 1: RTL waveform. The state steps through 00 → 01 → 10 → 11 → 00. Both the initial reset and the mid-run reset return the design to NS_GREEN.*

### Testbench Code (`traffic_light_controller_tb.v`)

```verilog
//============================================================================
// traffic_light_controller_tb.v - self-checking testbench
//============================================================================
`timescale 1ns/1ps

module traffic_light_controller_tb;

    localparam GREEN_TIME  = 5;
    localparam YELLOW_TIME = 2;
    localparam CYCLE_LEN   = 2*GREEN_TIME + 2*YELLOW_TIME;
    localparam NUM_CYCLES  = 4;

    reg clk;
    reg rst;

    wire ns_red, ns_yellow, ns_green;
    wire ew_red, ew_yellow, ew_green;

    traffic_light_controller #(
        .GREEN_TIME(GREEN_TIME),
        .YELLOW_TIME(YELLOW_TIME)
    ) dut (
        .clk(clk),
        .rst(rst),

        .ns_red(ns_red),
        .ns_yellow(ns_yellow),
        .ns_green(ns_green),

        .ew_red(ew_red),
        .ew_yellow(ew_yellow),
        .ew_green(ew_green)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    integer total_tests, passed_tests, failed_tests;
    integer safety_fail, timing_fail, fsm_fail;
    integer cyc;
    integer cur_code, prev_code, run_len, exp_len;

    reg check_en;

    reg [8*9:1] sname;
    reg [8*6:1] nsc, ewc;

    task chk;
        input cond;
        input integer cat;
        begin
            total_tests = total_tests + 1;

            if (cond)
                passed_tests = passed_tests + 1;
            else begin
                failed_tests = failed_tests + 1;

                case (cat)
                    0: safety_fail = safety_fail + 1;
                    1: timing_fail = timing_fail + 1;
                    default: fsm_fail = fsm_fail + 1;
                endcase

                $display(
                    "** FAIL (category %0d) at time %0t, cycle %0d",
                    cat, $time, cyc
                );
            end
        end
    endtask

    always @(negedge clk) begin
        if (check_en) begin
            cyc = cyc + 1;

            if (ns_green && ew_red &&
                !ns_yellow && !ns_red &&
                !ew_yellow && !ew_green)
                cur_code = 0;

            else if (ns_yellow && ew_red &&
                     !ns_green && !ns_red &&
                     !ew_yellow && !ew_green)
                cur_code = 1;

            else if (ew_green && ns_red &&
                     !ew_yellow && !ew_red &&
                     !ns_yellow && !ns_green)
                cur_code = 2;

            else if (ew_yellow && ns_red &&
                     !ew_green && !ew_red &&
                     !ns_yellow && !ns_green)
                cur_code = 3;

            else
                cur_code = -1;

            case (cur_code)
                0: sname = "NS_GREEN ";
                1: sname = "NS_YELLOW";
                2: sname = "EW_GREEN ";
                3: sname = "EW_YELLOW";
                default: sname = "ILLEGAL  ";
            endcase

            nsc = ns_green  ? "GREEN " :
                  ns_yellow ? "YELLOW" :
                  ns_red    ? "RED   " : "??????";

            ewc = ew_green  ? "GREEN " :
                  ew_yellow ? "YELLOW" :
                  ew_red    ? "RED   " : "??????";

            $display(
                "%s NS=%s EW=%s (time=%0t, cycle=%0d)",
                sname, nsc, ewc, $time, cyc
            );

            chk(!(ns_green && ew_green), 0);
            chk(!ns_green || ew_red, 0);
            chk(!ns_yellow || ew_red, 0);
            chk(!ew_green || ns_red, 0);
            chk(!ew_yellow || ns_red, 0);

            chk(
                ((ns_red + ns_yellow + ns_green) == 1) &&
                ((ew_red + ew_yellow + ew_green) == 1),
                0
            );

            chk(cur_code != -1, 2);
            chk(cur_code == dut.state, 2);

            if (prev_code == -1) begin
                chk(cur_code == 0, 2);
                run_len = 1;
            end
            else if (cur_code != prev_code) begin
                exp_len =
                    (prev_code == 0 || prev_code == 2)
                    ? GREEN_TIME : YELLOW_TIME;

                chk(run_len == exp_len, 1);
                chk(cur_code == ((prev_code + 1) % 4), 2);

                run_len = 1;
            end
            else begin
                run_len = run_len + 1;

                exp_len =
                    (cur_code == 0 || cur_code == 2)
                    ? GREEN_TIME : YELLOW_TIME;

                chk(run_len <= exp_len, 1);
            end

            prev_code = cur_code;
        end
    end

    initial begin
        $dumpfile("traffic_light_controller_tb.vcd");
        $dumpvars(0, traffic_light_controller_tb);

        total_tests = 0;
        passed_tests = 0;
        failed_tests = 0;

        safety_fail = 0;
        timing_fail = 0;
        fsm_fail = 0;

        cyc = 0;
        prev_code = -1;
        run_len = 0;
        cur_code = -1;
        exp_len = 0;

        check_en = 1'b0;
        rst = 1'b1;

        repeat (3) @(posedge clk);

        @(negedge clk);

        $display(
            "RESET NS=%s EW=%s",
            ns_green ? "GREEN " : "other ",
            ew_red ? "RED " : "other "
        );

        chk(
            ns_green && ew_red &&
            !ns_yellow && !ns_red &&
            !ew_yellow && !ew_green,
            2
        );

        @(posedge clk);
        #1 rst = 1'b0;
        check_en = 1'b1;

        repeat (NUM_CYCLES*CYCLE_LEN)
            @(posedge clk);

        while (cur_code != 2)
            @(posedge clk);

        #1 check_en = 1'b0;
        rst = 1'b1;

        $display("--- Reset applied during EW_GREEN ---");

        @(posedge clk);
        @(negedge clk);

        $display(
            "RESET NS=%s EW=%s",
            ns_green ? "GREEN " : "other ",
            ew_red ? "RED " : "other "
        );

        chk(
            ns_green && ew_red &&
            !ns_yellow && !ns_red &&
            !ew_yellow && !ew_green,
            2
        );

        @(posedge clk);
        #1 rst = 1'b0;

        prev_code = -1;
        run_len = 0;
        check_en = 1'b1;

        repeat (CYCLE_LEN)
            @(posedge clk);

        @(negedge clk);
        #1 check_en = 1'b0;

        $display("");
        $display(
            "SAFETY CHECK: %s",
            (safety_fail == 0) ? "PASS" : "FAIL"
        );

        $display(
            "TIMING CHECK: %s",
            (timing_fail == 0) ? "PASS" : "FAIL"
        );

        $display(
            "FSM CHECK: %s",
            (fsm_fail == 0) ? "PASS" : "FAIL"
        );

        $display("");
        $display("TOTAL TESTS : %0d", total_tests);
        $display("PASSED      : %0d", passed_tests);
        $display("FAILED      : %0d", failed_tests);
        $display("");

        if (failed_tests == 0)
            $display("TESTBENCH PASSED");
        else
            $display("TESTBENCH FAILED");

        $finish;
    end

    initial begin
        #200000;
        $display("TIMEOUT - TESTBENCH FAILED");
        $finish;
    end

endmodule
```

## 6. Synthesis

The RTL was synthesized using Yosys, targeting the `sky130_fd_sc_hd` typical library.

### Synthesis Results

| Item | Value |
|---|---:|
| Total cells | 24 |
| Flip-flops (`dfxtp_1`) | 5 |
| Memories | 0 |
| Ports | 8 |
| Reported chip area | 205.20 µm² |
| Sequential cell area | 100.10 µm² |
| Sequential area percentage | 48.78% |

The five flip-flops consist of two state bits and three counter bits. The synthesized circuit includes the counter, terminal-count comparator, next-state logic, and output decoding.

**Figure 2:** Yosys cell statistics showing 24 cells and a reported chip area of 205.20 µm².

<img width="1920" height="923" alt="statatisticstraffic" src="https://github.com/user-attachments/assets/afe9b1a7-e7fe-4281-9cd7-419b90078045" />


## 7. Gate-Level Simulation (GLS)

The synthesized netlist was simulated using the Sky130 cell models in functional mode, with a unit delay of 1 ns per cell. The testbench was adapted by removing the RTL internal-state comparison because the synthesized netlist does not preserve the same RTL register hierarchy.

### GLS Results

- 656 of 656 checks passed.
- Zero failures.
- Simulation ended at 841 ns.
- RTL and GLS cycle-by-cycle text traces were identical.

The GLS waveforms show approximately 1 ns of delay after clock edges because of the unit-delay cell models.

**Figure 3:** Gate-level simulation waveform.

<img width="1920" height="923" alt="glstraffic" src="https://github.com/user-attachments/assets/0d56e664-9435-49ad-8580-de66133cef5f" />


**Figure 4:** GLS terminal output and RTL-versus-GLS trace comparison.

<img width="1920" height="923" alt="trafficidentical" src="https://github.com/user-attachments/assets/e85f9654-1f66-4452-ae22-938aaea2cbbf" />


## 8. Pre-Layout Timing Analysis

Timing was checked on the synthesized netlist using OpenSTA with an ideal clock.

The constraints used were:

- Clock period: 10 ns (100 MHz).
- Input delay on `rst`: 2 ns.
- Output delay on all outputs: 2 ns.

### Pre-Layout STA Results

| Metric | Value |
|---|---:|
| WNS | 0.00 ns |
| TNS | 0.00 ns |
| Worst setup slack | +7.38 ns |
| Worst hold slack | +0.43 ns |

The reported hold path includes a flip-flop clock-to-Q delay of 0.30 ns and a NOR-gate delay of 0.09 ns. The data arrives at 0.39 ns against a library hold time of −0.04 ns, giving 0.43 ns of hold slack.

These results use an ideal clock and do not include the full effects of routed wire delays.

**Figure 5:** Pre-layout OpenSTA report.

<img width="1280" height="768" alt="prelayoutsta" src="https://github.com/user-attachments/assets/9f24ccd9-6f2e-4b74-a826-3a8906962b35" />


## 9. Physical Design

The physical design was performed using OpenLane v0.21 in its Docker environment with the Sky130A PDK.

The flow includes synthesis, floorplanning, power-grid generation, placement, clock-tree synthesis, routing, Magic DRC, Netgen LVS, antenna checking, electrical rule checking, and GDSII generation.

### Main Configuration

| Setting | Value |
|---|---|
| Design | `traffic_light_controller` |
| Clock port | `clk` |
| Clock period | 10 ns |
| Die area | 100 µm × 100 µm |
| Placement target density | 0.3 |
| Power-grid pitch | 40 µm |
| Power-grid offset | 5 µm |
| Cell library | `sky130_fd_sc_hd` |
| Macros | None |

### Main Physical-Design Results

| Item | Result |
|---|---|
| Flow status | Completed without fatal errors |
| Die area | 0.01 mm² |
| Placement utilization | 4% |
| Cells counted by OpenLane | 30 logic cells |
| Tap cells | 90 |
| End caps | 56 |
| Routed wire length | 1038 µm |
| Vias | 241 |
| Routing layer 2 usage | 2.14% |
| Routing layer 3 usage | 2.22% |
| TritonRoute violations | 0 |
| Magic DRC violations | 0 |
| LVS errors | 0 |
| Antenna violations | 0 |
| CVC errors | 0 |

The utilization is low because the 100 µm × 100 µm die has an area of 10,000 µm², while the logic requires approximately 200 µm². The flow reports 30 cells because its synthesis script produces a different cell count from the standalone Yosys run.

### 9.1 Floorplanning

Floorplanning establishes the die and core dimensions, creates standard-cell rows, positions I/O pins, and builds the power grid.

The die was configured as 100 µm × 100 µm using absolute sizing. The eight pins—clock, reset, and six light outputs—were placed around the die edges.

The power grid used a 40 µm pitch and a 5 µm offset. The flow added 90 tap cells and 56 end caps.

**Result:** The floorplan was accepted, and the flow continued.

### 9.2 Placement

Placement assigns physical positions to standard cells. Global placement estimates suitable locations, while detailed placement legalizes the cells and removes overlaps.

The target placement density was 0.3, and the reported placement utilization was 4%.

The logic cells form a small cluster near the bottom-centre of the die. Other rows contain physical-only cells, including tap, end-cap, and filler cells.

**Result:** The cells were placed without overlap, and substantial free space remained.

<img width="1920" height="922" alt="placementtt" src="https://github.com/user-attachments/assets/0cb82a1d-1cdf-4ce7-b3c4-79df51e2509d" />


### 9.3 Clock-Tree Synthesis (CTS)

Clock-tree synthesis builds the network that distributes the clock signal to the flip-flops while controlling clock skew and insertion delay.

The controller contains only five flip-flops, so its clock network is small. The post-route timing report shows approximately 0.21 ns of clock-tree arrival time at the flip-flops.

The post-route hold slack at the typical corner was +0.47 ns, compared with +0.43 ns before layout.

**Result:** The clock tree was built, and the reported timing requirements were met.
<img width="1920" height="922" alt="proof3" src="https://github.com/user-attachments/assets/6ffcaa61-731f-4d3c-84f8-71dadb637414" />


### 9.4 Routing

Routing creates the metal wires and vias that connect standard cells, the clock network, and the power grid.

Global routing estimates routing paths, while detailed routing uses TritonRoute to generate the actual interconnections.

The design completed detailed routing with zero reported routing violations. The total routed wire length was 1038 µm, with 241 vias.

The flow extracted parasitics into the SPEF file and used them for post-route timing analysis.

**Result:** All nets were routed without reported violations, and the routed layout proceeded to GDSII generation.
<img width="1920" height="922" alt="proof2" src="https://github.com/user-attachments/assets/66af1e27-5b0b-4afb-8a96-7120b1302c36" />
<img width="1920" height="922" alt="proof4" src="https://github.com/user-attachments/assets/5bd6c974-2874-40e5-bd6e-0fba8351cf45" />



## 10. Post-Route Timing Analysis

After routing, OpenSTA was run on the clock-tree netlist using a propagated clock and extracted parasitics. The analysis used the typical corner and 18 process-voltage-temperature (PVT) corners of the Sky130 library.

### Typical Corner Results

| Metric | Value |
|---|---:|
| Corner | TT, 25 °C, 1.8 V |
| WNS | 0.00 ns |
| TNS | 0.00 ns |
| Worst setup slack | +7.15 ns |
| Worst hold slack | +0.47 ns |
| Estimated total power | Approximately 123 µW |
| Clock-tree arrival time | Approximately 0.21 ns |

The power estimate was based on default switching activity. The report attributes approximately 32.5% to sequential cells and 67.5% to combinational cells.

### Results Across All Corners

The source document reports zero WNS and TNS at all analyzed corners, with:

- Worst setup slack: +0.16 ns at `ss_n40C_1v28`.
- Worst hold slack: +0.30 ns at the fast 1.95 V corners.
- Estimated power ranging from approximately \(5.3 \times 10^{-5}\) W to \(1.5 \times 10^{-4}\) W.

Setup slack decreases at slower corners, while hold slack tends to increase. This is consistent with the expected relationship between cell delays and process-voltage-temperature conditions.

**Figure 11:** Post-route timing summary across the 18 corners.

<img width="1920" height="922" alt="stareport" src="https://github.com/user-attachments/assets/59d22b53-d101-4d90-8693-202bb872e011" />


### Understanding the Timing Metrics

- **WNS:** Worst negative setup slack.
- **TNS:** Sum of negative setup slacks.
- **Setup slack:** The difference between the required data arrival time and the actual arrival time.
- **Hold slack:** The margin by which the data path satisfies the hold-time requirement.

Zero WNS and TNS indicate no reported setup violations in the summarized results. Positive setup and hold slack indicate that the analyzed paths meet the respective timing requirements.

## 11. Summary of Results

| Stage | Result |
|---|---|
| RTL simulation | 735/735 checks passed |
| Boundary timing tests | Passed for 1/1 and 3/1 |
| Synthesis | 24 cells, 5 flip-flops, 205.20 µm² |
| Gate-level simulation | 656/656 checks passed |
| RTL/GLS comparison | Traces identical |
| Pre-layout timing | WNS 0.00 ns, setup +7.38 ns, hold +0.43 ns |
| Floorplan | 100 µm × 100 µm die |
| Placement utilization | 4% |
| Placement, CTS, routing | Completed, zero routing violations |
| DRC/LVS/antenna/CVC | 0/0/0/0 |
| Post-route typical timing | Setup +7.15 ns, hold +0.47 ns |
| Post-route multi-corner timing | No reported violations |
| Output | GDSII file generated |

## 12. Limitations and Future Work

The controller uses fixed timing and does not include vehicle sensors or pedestrian phases. Real traffic signals also require lamp-driver circuits, which are outside this design's scope.

The simulation clock period is 10 ns, making the green interval only 50 ns in simulation. Real traffic signals require phase durations measured in seconds, so a clock divider or slower timing source would be required for practical hardware.

Because no clock period was originally specified, a 10 ns clock and 2 ns input/output delays were assumed. A faster clock was not tested.

The multi-corner analysis reused extracted parasitics across the analyzed libraries, so the corner results should be treated as estimates rather than a complete sign-off for every physical condition. Power estimates also use default switching activity.

A cross-check between GDS files generated by Magic and KLayout reported differences that were not investigated further. The Magic layout was used as the reference for the reported DRC, LVS, and antenna checks.

### Future Enhancements

1. Add vehicle-sensor inputs for traffic-responsive signal timing.
2. Introduce pedestrian crossing phases.
3. Add a clock divider to support realistic signal durations.
4. Add a fault-safe flashing state.
5. Extend the controller to manage multiple intersections.

## 13. Conclusion

A fixed-time two-road traffic light controller was designed in synthesizable Verilog and verified using a self-checking testbench in RTL and gate-level simulation.

The design was synthesized to 24 standard cells, with a reported area of 205.20 µm², and implemented on a 100 µm × 100 µm die using OpenLane and the Sky130 standard-cell library.

The physical-design flow completed with clean reported DRC, LVS, and antenna checks. Timing analysis was performed before and after routing, including analysis across multiple PVT corners. The flow generated the final GDSII layout.

Overall, the project demonstrates the complete RTL-to-GDSII methodology, including RTL design, simulation, synthesis, gate-level verification, static timing analysis, floorplanning, placement, clock-tree synthesis, routing, physical verification, and layout generation.

---

**End of Report**

// Rough RTL model of the S-LX peripheral board: a register file standing in
// for the BMP280/ICM20689 I2C sensors and PCA9685 servo driver, plus four PWM
// channels and the arm/status GPIO. This is connectivity-level, not a
// cycle-accurate I2C or PCA9685 implementation -- the Verilated C++ bridge
// loads it from POSIX shared memory supplied by the physics plant.
module slx_periph (
    input  logic             clk,
    input  logic             rst,
    input  logic             wr,
    input  logic             valid,
    input  logic [7:0]       addr,
    input  logic [63:0]      wdata,
    output logic [63:0]      rdata,
    output logic [3:0]       pwm,
    output logic             arm_led,
    output logic             fault_led,
    // Live peripheral/pin status mirrored into the bridge CSV for the GUI.
    output logic             pwm_en,
    output logic             i2c_busy,
    output logic             spi_busy,
    output logic             sd_cs_n,
    output logic             usart_tx,
    output logic             usart_rx,
    output logic             status_led,
    output logic             arm_gpio,
    output logic [7:0]       fault_code
);
    localparam logic [63:0] ZERO = 64'd0;

    // Register map (one 64-bit word each):
    //   0x00 control/status:
    //        bit0 PWM enable, bit1 SPI/SD logging, bit2 I2C activity,
    //        bit3 USART TX activity
    //   0x10 alt_m, 0x11 pressure_pa,
    //   0x12..0x14 velocity xyz, 0x15..0x17 attitude pitch/roll/yaw,
    //   0x18..0x1a rates, 0x1b..0x1d accel g, 0x1e..0x20 gyro dps,
    //   0x21 q_ratio, 0x30..0x33 servo pulse width in microseconds,
    //   0x38 arm GPIO (bit0), 0x39 fault code.
    logic [63:0] regs [0:63];
    integer i;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 64; i = i + 1)
                regs[i] <= ZERO;
        end else if (wr && valid) begin
            regs[addr[5:0]] <= wdata;
        end
    end

    always_comb begin
        rdata = regs[addr[5:0]];
    end

    // Servo pulse widths are held in microseconds. The bridge samples at
    // 100 Hz, so pwm[] reports channel enable/valid-pulse state rather than an
    // instantaneous 1 MHz comparator level (which would alias badly).
    logic [15:0] pwm_cmp [0:3];

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : gen_servo
            assign pwm_cmp[g] = regs[6'h30 + g][15:0];
            assign pwm[g] = regs[6'h00][0] &&
                            (pwm_cmp[g] >= 16'd900) && (pwm_cmp[g] <= 16'd2100);
        end
    endgenerate

    assign arm_led    = regs[6'h38][0];
    assign fault_led  = (regs[6'h39] != ZERO);

    assign pwm_en     = regs[6'h00][0];
    assign i2c_busy   = regs[6'h00][2];
    assign spi_busy   = regs[6'h00][1];
    assign sd_cs_n    = ~regs[6'h00][1];   // active low while logging
    assign usart_tx   = regs[6'h00][3];
    assign usart_rx   = 1'b1;              // idle high; flight has no RX command
    assign arm_gpio   = regs[6'h38][0];
    assign fault_code = regs[6'h39][7:0];
    assign status_led = pwm_en && (fault_code == 8'd0);
endmodule

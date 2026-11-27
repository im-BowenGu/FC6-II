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
    output logic             fault_led
);
    localparam logic [63:0] ZERO = 64'd0;

    // Register map (one 64-bit word each):
    //   0x00 control/status, 0x10 alt_m, 0x11 pressure_pa,
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

    // 20 ms servo period at a 1 MHz register clock: 0..20000 counts.
    logic [15:0] pwm_count;
    logic [15:0] pwm_cmp [0:3];
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            pwm_count <= 16'd0;
        else if (pwm_count >= 16'd19999)
            pwm_count <= 16'd0;
        else
            pwm_count <= pwm_count + 16'd1;
    end

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : gen_servo
            assign pwm_cmp[g] = regs[8'h30 + g][15:0];
            assign pwm[g] = (pwm_count < pwm_cmp[g]) && regs[8'h00][0];
        end
    endgenerate

    assign arm_led   = regs[8'h38][0];
    assign fault_led = (regs[8'h39] != ZERO);
endmodule

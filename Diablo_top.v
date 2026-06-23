//==========================================================================================================
// Diablo 2315 Archiver
// Top Level definition
// File Name: Diablo_top.v
//
// Designed for reading 2315 on Diablo by Carl Claunch
//
//==========================================================================================================

`include "bus_disk_read.v"
`include "sector_and_index.v"
`include "seek_to_cylinder.v"
`include "dram_controller.v"
`include "archiver.v"
`include "uploader.v"

//================================= TOP LEVEL INPUT-OUTPUT DEFINITIONS =====================================
module Diablo_top (

// Inputs

    // this is Diablo READY TO SEEK, READ OR WRITE
    input wire READY_SEEK_L,    // Seek ready and on-cylinder
    
    // this is Diablo READ DATA
    input wire RD_DATA_L,       // Read data pulses, 160 ns pulse
    
    // this is Diablo READ CLOCK
    input wire RD_CLK_L,        // Read clock pulses, 160 ns pulse
    
    // this is Diablo SECTOR MARKS
    input wire SECTOR_MARK_L,        // 160 us negative pulse each time a sector slot passes the transducer
    
    // this is Diablo INDEX MARKS
    input wire INDEX_MARK_L,         // 160 us negative pulse for each revolution of the disk, 600 μs after the sector pulse
    
    // this is Diablo ADDRESS ACKNOWLEDGE
    input wire ADDR_ACK,

    // this is Diablo LOGICAL ADDRESS INTERNLOCK used only to light error LED
    input wire LOG_ADDR_INLK,

    // this is Diablo SEEK INCOMPLETE used only to light error LED
    input wire SEEK_INCOMP,
    
    // ignore PSEUDO_SM, WRITE CHECK, HIGH DENSITY AND SECTOR ADDRESSS signals

// Clock external pin
    input wire CLK100MHZ,                 // board 100MHz clock

// external reset button
    input wire ck_rst,                    // reset button or USB connection does reset low

// pushbutton
    input wire button0,                   // button 0

// slide switch
    input wire slideswitch,               // slideswitch 0

// DRAM PORTS
    inout [15:0]    ddr3_dq,
    inout [1:0]     ddr3_dqs_n,
    inout [1:0]     ddr3_dqs_p,
    output [13:0]   ddr3_addr,
    output [2:0]    ddr3_ba,
    output          ddr3_ras_n,
    output          ddr3_cas_n,
    output          ddr3_reset_n,
    output [0:0]    ddr3_cs_n,
    output          ddr3_we_n,
    output [0:0]    ddr3_ck_p,
    output [0:0]    ddr3_ck_n,
    output [0:0]    ddr3_cke,
    output [1:0]    ddr3_dm,
    output [0:0]    ddr3_odt,

// Outputs

    // this is Diablo STROBE
    output wire STROBE_L,                // Strobe to enable movement    
    
    // this is Diablo HEAD SELECT
    output wire HEAD_SELECT_L,  // Head Select
    
    // this is Diablo READ GATE
    output wire RD_GATE_L,      // Read gate, when active enables read circuitry
    
    // this is Diablo TRACK ADDRESS
    output wire [7:0] TRACK,
    
    // ignore and leave high Diablo RESTORE, WRITE GATE, WRITE CLOCK AND DATA, SELECT[3:0]

// USB communications channel
    output wire uart_rxd_out,
  
// LEDs
    output wire led0_b,                 // LD blue
    output wire led0_g,                 // LD green
    output wire led0_r,                 // LD red
    output wire led1_b,                 // LD blue
    output wire led1_g,                 // LD green
    output wire led1_r,                 // LD red
    output wire led2_b,                 // LD blue
    output wire led2_g,                 // LD green
    output wire led2_r,                 // LD red
    output wire led3_b,                 // LD blue
    output wire led3_g,                 // LD green
    output wire led3_r,                 // LD red
    output wire led1,                   // LD 7
    output wire led2,                   // LD 6
    output wire led3,                   // LD 5
    output wire led4                    // LD 4

);

//============================ Internal Connections ==================================

wire        reset;
wire [7:0]  Cylinder_Address;
wire        Head_Select;
wire [1:0]  Sector_Address;
wire [8:0]  Word_Number;
wire [7:0]  dramCylinder_Address;
wire        dramHead_Select;
wire [1:0]  dramSector_Address;
wire [8:0]  dramWord_Number;
wire        calib_complete;
wire        requestread;
wire        clkenbl_sector;
wire [15:0] readdata;
wire        readdone;
wire        grabword;
wire        strobe_move;
wire        echoword;
wire        alldone;
wire        dataready;
wire [15:0] readback;
wire [7:0]  cyl_address;
wire [1:0]  sector_address;
wire        head_address;
wire [8:0]  word_address;
wire        dumpdone;
wire        txd_out;
wire        extract_cartridge;
wire        upload_cartridge;

//============================ MISC TOP LEVEL LOGIC ==================================

assign dramCylinder_Address = slideswitch == 1'b0
                              ?  Cylinder_Address
                              :  cyl_address;

assign dramHead_Select = slideswitch == 1'b0
                         ?  Head_Select
                         :  head_address;

assign dramSector_Address = slideswitch == 1'b0
                            ?  Sector_Address
                            :  sector_address;

assign dramWord_Number = slideswitch == 1'b0
                         ?  Word_Number
                         :  word_address;

assign HEAD_SELECT_L = Head_Select;

assign extract_cartridge = button0 && 
                           READY_SEEK_L == 1'b0 && 
                           slideswitch == 1'b0; 

assign upload_cartridge =  button0 && slideswitch == 1'b1; 

assign led1 = calib_complete;
assign led2 = slideswitch == 1'b0
                         ?  ~LOG_ADDR_INLK
                         :  dumpdone;
assign led3 = ~SEEK_INCOMP;
assign led4 = alldone;

assign led0_b = dramSector_Address[1:1];
assign led0_g = dramSector_Address[0:0];
assign led0_r = Head_Select;
assign led1_b = (dramCylinder_Address < 23)                                  ? 1'b1 : 1'b0;
assign led1_g = (dramCylinder_Address > 22)  && (dramCylinder_Address < 46)  ? 1'b1 : 1'b0;
assign led1_r = (dramCylinder_Address > 45)  && (dramCylinder_Address < 69)  ? 1'b1 : 1'b0;
assign led2_b = (dramCylinder_Address > 68)  && (dramCylinder_Address < 92)  ? 1'b1 : 1'b0;
assign led2_g = (dramCylinder_Address > 91)  && (dramCylinder_Address < 115) ? 1'b1 : 1'b0;
assign led2_r = (dramCylinder_Address > 114) && (dramCylinder_Address < 148) ? 1'b1 : 1'b0;
assign led3_b = (dramCylinder_Address > 147) && (dramCylinder_Address < 171) ? 1'b1 : 1'b0;
assign led3_g = (dramCylinder_Address > 170) && (dramCylinder_Address < 194) ? 1'b1 : 1'b0;
assign led3_r = (dramCylinder_Address > 193)                                 ? 1'b1 : 1'b0;


assign uart_rxd_out = txd_out;

//================================= MODULES =====================================


// ======== Module ======== bus_disk_read =====
bus_disk_read i_bus_disk_read (
    // Inputs
    .clock (CLK100MHZ),
    .reset (reset),
    .requestread (requestread),
    .clkenbl_sector (clkenbl_sector),
    .RD_DATA_L (RD_DATA_L),
    .RD_CLK_L (RD_CLK_L),

    // Outputs
    .RD_GATE_L (RD_GATE_L),
    .readdata (readdata),
    .readdone (readdone),
    .Word_Number (Word_Number),
    .grabword (grabword)
);


// ======== Module ======== sector_and_index =====
sector_and_index i_sector_and_index (
    // Inputs
    .clock (CLK100MHZ),
    .reset (reset),
    .SECTOR_MARK_L (SECTOR_MARK_L),
    .INDEX_MARK_L (INDEX_MARK_L),

    // Outputs
    .clkenbl_sector (clkenbl_sector),
    .Sector_Address (Sector_Address)
);

// ======== Module ======== seek_to_cylinder =====
seek_to_cylinder i_seek_to_cylinder (
    // Inputs
    .clock (CLK100MHZ),
    .reset (reset),
    .strobe_move (strobe_move),
    .ADDR_ACK (ADDR_ACK),
    .READY_SEEK_L (READY_SEEK_L),

    // Outputs
    .STROBE_L (STROBE_L),
    .TRACK (TRACK),
    .Cylinder_Address (Cylinder_Address)
);

dram_controller i_dram_controller (
    // inputs
    .CLK100MHZ (CLK100MHZ),
    .grabword (grabword),
    .echoword (echoword),
    .Word_Number (dramWord_Number),
    .Sector_Address (dramSector_Address),
    .Head_Select (dramHead_Select),
    .Cylinder_Address (dramCylinder_Address),
    .readdata (readdata),
    .requestread (requestread),
    .clkenbl_sector (clkenbl_sector),
    .ck_rst (ck_rst),

    // DDR3 ports
    .ddr3_dq(ddr3_dq),
    .ddr3_dqs_n(ddr3_dqs_n),
    .ddr3_dqs_p(ddr3_dqs_p),
    .ddr3_addr(ddr3_addr),
    .ddr3_ba(ddr3_ba),
    .ddr3_ras_n(ddr3_ras_n),
    .ddr3_cas_n(ddr3_cas_n),
    .ddr3_we_n(ddr3_we_n),
    .ddr3_ck_p(ddr3_ck_p),
    .ddr3_cs_n(ddr3_cs_n),
    .ddr3_reset_n(ddr3_reset_n),
    .ddr3_ck_n(ddr3_ck_n),
    .ddr3_cke(ddr3_cke),
    .ddr3_dm(ddr3_dm),
    .ddr3_odt(ddr3_odt),

    // outputs
    .reset (reset),
    .dataready (dataready),
    .init_calib_complete (calib_complete),
    .readback (readback)
);

archiver i_archiver (
    // inputs
    .clock (CLK100MHZ),
    .reset (reset),
    .extract_cartridge (extract_cartridge),
    .Sector_Address (Sector_Address),
    .readdone (readdone),
    .READY_SEEK_L (READY_SEEK_L),

    // outputs
    .requestread (requestread),
    .Head_Select (Head_Select),
    .strobe_move (strobe_move),
    .alldone (alldone)
);

uploader i_uploader (
    // inputs
    .clock (CLK100MHZ),
    .reset (reset),
    .upload_cartridge (upload_cartridge),
    .dataready (dataready),
    .readback (readback),

    // outputs
    .echoword (echoword),
    .cyl_address (cyl_address),
    .head_address (head_address),
    .sector_address (sector_address),
    .word_address (word_address),
    .dumpdone (dumpdone),
    .txd_out (txd_out)
);

endmodule // Diablo_top

//==========================================================================================================
// Diablo Archiver
// File Name: dram_controller.v
//
// Written for reading 2315 on Diablo by Carl Claunch

// Functions: 
// will store words in DRAM as we archive the cartridge
// also allows fetching over USB
// makes use of a memory IP module from Vivado
//
//==========================================================================================================

module dram_controller(
    input wire CLK100MHZ,                   // master clock 100 MHz
    input wire grabword,                 // request to store a word from the sector read
    input wire echoword,                 // request from USB to read out a word
    input wire [8:0] Word_Number,        // specifies which word of the sector we want
    input wire [1:0] Sector_Address,     // specifies which sector
    input wire Head_Select,              // head selection (upper or lower)
    input wire [7:0] Cylinder_Address,   // cylinder address
    input wire [15:0] readdata,          // 16-bit sector data to write to DRAM controller
    input wire requestread,              // watch for when sector read starts
    input wire clkenbl_sector,           // with request then sector pulse; lock sector address
    input wire ck_rst,                   // async reset from hardware
    inout [15:0]    ddr3_dq,
    inout [1:0]     ddr3_dqs_n,
    inout [1:0]     ddr3_dqs_p,
    output [13:0]   ddr3_addr,
    output [2:0]    ddr3_ba,
    output          ddr3_ras_n,
    output          ddr3_reset_n,
    output          ddr3_cas_n,
    output          ddr3_we_n,
    output [0:0]    ddr3_ck_p,
    output [0:0]    ddr3_ck_n,
    output [0:0]    ddr3_cs_n,
    output [0:0]    ddr3_cke,
    output [1:0]    ddr3_dm,
    output [0:0]    ddr3_odt,
    output wire reset,                   // sync reset for all logic
    output reg dataready,                // flag that our word is in readback
    output wire init_calib_complete, 
    output reg [15:0] readback		 // 16-bit data from DRAM to write to USB
);

//============================ Internal Connections ==================================
  `define CMD_READ 3'b001
  `define CMD_WRITE 3'b000

    wire         locked;
    wire         clk200;
    reg [27:0]   app_addr;           // 28 bit address, we use 20
    reg [2:0]    app_cmd;
    reg          app_en;
    wire [127:0] app_wdf_data;       // eight 16 bit words wide
    reg          app_wdf_end;
    wire [15:0]  app_wdf_mask;       // which section of data is written
    reg          app_wdf_wren;
    wire [127:0] app_rd_data;
    wire         app_rd_data_end;
    wire         app_rd_data_valid;
    wire         app_rdy;
    wire         app_wdf_rdy;
    wire         ui_reset;
    wire          ui_clk;

    wire [15:0]  din;
    reg          wr_en;
    reg          rd_en;
    wire [15:0]  dout;
    wire         full;
    wire         empty;
    reg [15:0]   rdin;
    reg          rwr_en;
    reg          rrd_en;
    wire [15:0]  rdout;
    wire         rfull;
    wire         rempty;
    wire         valid;
    wire         rvalid;

    reg [1:0]    Saved_Sector;
    reg [1:0]    edgedetect;
    reg [1:0]    sectordetect;

`define FW0  0
`define FW1  1
`define FW2  2
`define FW3  3
    reg [1:0]  w_state;
    // FW0 idle
    // FW1 write to w-FIFO
    // FW2 pull from r-FIFO
    // FW3 latch output

`define FU0 0
`define FU1 1
`define FU2 2
`define FU3 3
`define FU4 4
`define FU5 5
`define FU6 6
`define FU7 7
    // FU0 idle
    // FU1 rec0
    // FU2 rec1
    // FU3 rec2
    // FU4 rec3
    // FU5 play0
    // FU6 play1
    // FU7 play2
    reg [2:0]   u_state;

`define L0 0
`define L1 1
`define L2 2
    // L0 idle waiting for read request
    // L1 got readrequest, wait for sector
    // L2 latch the sector value
    reg [1:0]    l_state;

    reg [7:0]    metacylinder [3:0];
    reg          metahead     [3:0];
    reg [1:0]    metasector   [3:0];
    reg [1:0]    metasaved    [3:0];
    reg [8:0]    metaword     [3:0];
    reg [3:0]    metaecho;
    reg [3:0]    metagrab;

// ============================ Start of Code =========================================
     
// when readrequest rising edge, wait for clkenbl_sector and latch Sector_address, 
//    Cylinder_Address, Head_Select part of address
//    full address formed from latched parts plus Word_Number
//
// anytime we see grabword rising edge, we capture readdata and begin a archive cycle
// first we push the data on the write FIFO then wait for a response on read FIFO
// read cycle ends raising readdone
//
// if we see echoword we instead will begin a fetch cycle
// wait for data on read FIFO and latch as output 
// echoword should not be issued until all archiving is done
// 
// CLK100MHZ domain does writes to wFIFO and waits for reads from rFIFO
// the ui-clk domain waits for data in wFIFO and writes it to RAM
// ui_clk domain writes responses to rFIFO
// 
// read cycle will initialize Cyl, Head, Sec and Word to 0 at first
// loop for 203x2x4 times:
//     emit header with addresses first
//     grab each word from RAM and push into r-FIFO in UI
//     pull each word from r-FIFO in CLK100MHZ when it appears
//     bumping Word_Number, and writing to USB
//     grab 322 word and format errors output
// 

// concurrent assignments
  assign app_wdf_data  = {dout, 112'b0};
  assign app_wdf_mask  = 16'b0;
  assign din = readdata;
  assign reset = (ui_reset) || (~init_calib_complete);
    

// main pump to and from the UI of the DDR3 RAM
//
// grabword is on when we are asked to write a word
// which pushes a word into w-FIFO and we see count > 0
//
// echoword is on when we are asked to retrieve a word
// and after we pass between clock domains, we start a read
always @ (posedge ui_clk)
begin : UIPUMP // block name
  if(reset) begin
      app_en            <= 1'b0;
      app_cmd           <= `CMD_WRITE;
      app_addr          <= 28'd0;
      app_wdf_wren      <= 1'b0;
      app_wdf_end       <= 1'b0;
      rd_en             <= 1'b0;
      rwr_en            <= 1'b0;
      rdin              <= 16'd0;
      metaecho          <= 4'd0;
      metagrab          <= 4'd0;
      metacylinder[3]   <= 8'd0;
      metacylinder[2]   <= 8'd0;
      metacylinder[1]   <= 8'd0;
      metacylinder[0]   <= 8'd0;
      metahead[3]       <= 1'd0;
      metahead[2]       <= 1'd0;
      metahead[1]       <= 1'd0;
      metahead[0]       <= 1'd0;
      metasector[3]     <= 2'd0;
      metasector[2]     <= 2'd0;
      metasector[1]     <= 2'd0;
      metasector[0]     <= 2'd0;
      metasaved[3]      <= 2'd0;
      metasaved[2]      <= 2'd0;
      metasaved[1]      <= 2'd0;
      metasaved[0]      <= 2'd0;
      metaword[3]       <= 9'd0;
      metaword[2]       <= 9'd0;
      metaword[1]       <= 9'd0;
      metaword[0]       <= 9'd0;
      u_state           <= `FU0;
  end
  else begin

      // handle clock domain crossing for these signals
      metacylinder[3] <= metacylinder[2];
      metacylinder[2] <= metacylinder[1];
      metacylinder[1] <= metacylinder[0];
      metacylinder[0] <= Cylinder_Address;
      metahead[3]     <= metahead[2];
      metahead[2]     <= metahead[1];
      metahead[1]     <= metahead[0];
      metahead[0]     <= Head_Select;
      metasector[3]   <= metasector[2];
      metasector[2]   <= metasector[1];
      metasector[1]   <= metasector[0];
      metasector[0]   <= Sector_Address;
      metasaved[3]    <= metasaved[2];
      metasaved[2]    <= metasaved[1];
      metasaved[1]    <= metasaved[0];
      metasaved[0]    <= Saved_Sector;
      metaword[3]     <= metaword[2];
      metaword[2]     <= metaword[1];
      metaword[1]     <= metaword[0];
      metaword[0]     <= Word_Number;
      metaecho[3:0]   <= {metaecho[2:0], echoword};
      metagrab[3:0]   <= {metagrab[2:0], grabword};

      app_en          <= 1'b0;
      app_wdf_wren    <= 1'b0;
      app_wdf_end     <= 1'b0;

      app_cmd         <= `CMD_WRITE;
      app_addr        <= 28'd0;

      case (u_state)
// idle
    `FU0: begin

          app_cmd       <= `CMD_WRITE;
          u_state       <= (init_calib_complete == 1'b1 && valid == 1'b1)
                           ? `FU1  // wrt0
                           : (init_calib_complete == 1'b1 && metaecho[3] == 1'b1)
                              ? `FU6   //read1
                              : `FU0;  // idle

          app_addr      <= metagrab[3] == 1'b0 && metaecho[3] == 1'b0
                           ? 28'd0
                           : app_addr;

          // drop app_en, rd_en, app_wdf_wren, app_wdf_end and rwr_en
          app_en       <= 1'b0;
          rwr_en       <= 1'b0;
          rd_en        <= 1'b0;
          app_wdf_wren <= 1'b0;
          app_wdf_end  <= 1'b0;

          end

// wrt0
    `FU1: begin
          
          // move forward to wrt2
          u_state       <= `FU3;

          // pull word from FIFO now
          rd_en        <= 1'b1;

          end

// wrt1 NEVER HAPPENS
    `FU2: begin

          // now ask for the write
          app_en        <= 1'b1;
          app_cmd       <= `CMD_WRITE;

          // proceed to wrt2
          u_state       <= `FU3;

          // no more from FIFO
          rd_en        <= 1'b0;

          end

// wrt2
    `FU3: begin

          // hold app_en until accepted then go to wrt3
          u_state       <= app_wdf_rdy == 1'b1
                           ? `FU4
                           : `FU3;

          // present the address we want for the write and latch it
          app_addr      <= {metacylinder[3], metahead[3], metasaved[3], metaword[3]}; 

          // drop FIFO pull request
          rd_en         <= 1'b0;

          // drop app_wdf_wren and app_wdf_end
          app_wdf_wren  <=  1'b1;
          app_wdf_end   <= 1'b1;

          // hold app_en until accepted
          app_en        <= 1'b1;

          // hold write command until accepted
          app_cmd       <= `CMD_WRITE;

          end

// wrt3
    `FU4: begin

          // if done, go back to idle
          u_state       <= app_rdy == 1'b1
                           ? `FU0
                           : `FU4;

          // drop app_en
          app_en        <= 1'b0;

          // hold write command until accepted
          app_cmd       <= `CMD_WRITE;

         end

// read0 NOT USED
    `FU5: begin

          // go to read1
          u_state       <= `FU6;

          // raise app_en and a read request
          app_en        <= 1'b1;
          app_cmd       <= `CMD_READ;

          end

// read1
    `FU6: begin
       
           // hold app_en until ready
           app_en <= 1'b1;

           // emit the desired address
           app_addr <= {metacylinder[3], metahead[3], metasector[3], metaword[3]};

           // assert read command
           app_cmd <= `CMD_READ;

           if ((app_rdy == 1'b1) && (app_en == 1'b1)) begin

             // when request accepted, go to read2
             u_state <= `FU7;

             // and turn off the enable
             app_en <= 1'b0;

             end

           end

// read2
    `FU7: begin

        // drop enable because it was accepted
        app_en <= 1'b0;

        // default write enable low
        rwr_en <= 1'b0;

        if (app_rd_data_valid) begin

          // push into FIFO when data is returned
          rwr_en          <= 1'b1;

          // output the data to the FIFO
          rdin            <= app_rd_data[127:112];

        end

        // wait to go idle until echoword has dropped
        u_state <= (metaecho[3] == 1'b0)
                   ? `FU0
                   : `FU7;

      end

    default: begin
          u_state            <= `FU0;
          end
    endcase
    end
end

// operate in our main clock domain
// 
// when we see grabword, this pushes the data
// into the w-FIFO so that the ui_clk side can grab it
//
// when echoword was received, ui_clk side did a read
// and pushed the results into r-FIFO so we
// grab it when r-FIFO empty state goes false
always @ (posedge CLK100MHZ)
begin : MAINCLOCKFUNCTIONS // block name
  if(reset) begin
      wr_en          <= 1'b0;
      w_state        <= `FW0;
      rrd_en         <= 1'b0;
      readback       <= 16'd0;
      dataready      <= 1'b0;
  end
  else begin

      case (w_state)
// idle waiting for a dram request
    `FW0: begin  
 
          // write to the w-FIFO when grabword goes on
          // otherwise if r-FIFO has contents, pull it
          w_state   <= grabword == 1'b1
                       ? `FW1
                       : rvalid == 1'b1
                         ? `FW2
                         : `FW0;

          // push into the w-FIFO
          wr_en     <= grabword == 1'b1
                       ? 1'b1
                       : 1'b0;
 
          // request to pull from r-FIFO
          rrd_en    <= rvalid == 1'b1
                       ? 1'b1
                       : 1'b0;

          // turn off finished alert
          dataready <= 1'b0;

     end

// pump a word request
    `FW1: begin   

          // go to idle after this
          w_state   <= `FW0;

          // turn off write into w-FIFO
          wr_en     <= 1'b0;

     end

// pull a read result
    `FW2: begin

          // go to idle after this
          w_state    <= `FW3;

          // turn off request to pull from r-FIFO
          rrd_en     <= 1'b0;

          // save the output of FIFO
          readback  <= rdout;
     end

// return the result
    `FW3: begin

          // go to idle when request dropped
          w_state    <= echoword == 1'b0
                        ? `FW0
                        : `FW3;

          // drop request
          rrd_en    <= 1'b0;

          // signal completion
          dataready <= 1'b1;

    end

    default: begin
          w_state   <= `FW0;
    end

    endcase
    end
end // End of Block MAINCLOCKFUNCTIONS

// process to latch the sector address 
always @ (posedge CLK100MHZ)
begin : LATCHSECTOR // block name
  if(reset) begin
     Saved_Sector       <= 2'd0;
     l_state            <= `L0;
     edgedetect         <= 2'd0;
     sectordetect       <= 2'd0;
  end
  else begin

    edgedetect[1:0] <= {edgedetect[0],requestread};
    sectordetect[1:0] <= {sectordetect[0],clkenbl_sector};

    case (l_state)
// idle waiting for a read request
    `L0: begin  

        l_state          <= edgedetect[1] == 1'b0 && edgedetect[0] == 1'b1
                            ? `L1
                            : `L0;
    end

// wait for sector to arrive
    `L1: begin

        l_state          <= sectordetect[1] == 1'b0 && sectordetect[0] == 1'b1

                            ? `L2
                            : `L1;
      
    end

// latch in the sector value
    `L2: begin

        // go back to idle
        l_state          <= `L0;

        // grab the value
        Saved_Sector     <= Sector_Address;

    end

    default: begin
      l_state            <= `L0;
    end

    endcase
  end
end


 mig_7series_1 u_mig_7series_0
      (
// Memory interface ports
       .ddr3_addr                      (ddr3_addr),
       .ddr3_ba                        (ddr3_ba),
       .ddr3_cas_n                     (ddr3_cas_n),
       .ddr3_ck_n                      (ddr3_ck_n),
       .ddr3_ck_p                      (ddr3_ck_p),
       .ddr3_cs_n                      (ddr3_cs_n),
       .ddr3_cke                       (ddr3_cke),
       .ddr3_ras_n                     (ddr3_ras_n),
       .ddr3_we_n                      (ddr3_we_n),
       .ddr3_dq                        (ddr3_dq),
       .ddr3_dqs_n                     (ddr3_dqs_n),
       .ddr3_dqs_p                     (ddr3_dqs_p),
       .ddr3_reset_n                   (ddr3_reset_n),
       .init_calib_complete            (init_calib_complete),
       .ddr3_dm                        (ddr3_dm),
       .ddr3_odt                       (ddr3_odt),
// Application interface ports
       .app_addr                       (app_addr),
       .app_cmd                        (app_cmd),
       .app_en                         (app_en),
       .app_wdf_data                   (app_wdf_data),
       .app_wdf_end                    (app_wdf_end),
       .app_wdf_wren                   (app_wdf_wren),
       .app_wdf_mask                   (app_wdf_mask),
       .app_rd_data                    (app_rd_data),
       .app_rd_data_end                (app_rd_data_end),
       .app_rd_data_valid              (app_rd_data_valid),
       .app_rdy                        (app_rdy),
       .app_wdf_rdy                    (app_wdf_rdy),
       .app_sr_req                     (1'b0),
       .app_ref_req                    (1'b0),
       .app_zq_req                     (1'b0),
       .app_sr_active                  (),
       .app_ref_ack                    (),
       .app_zq_ack                     (),
       .ui_clk                         (ui_clk),
       .ui_clk_sync_rst                (ui_reset),
// System Clock Ports
       .sys_clk_i                      (clk166),
// Reference Clock Ports
       .clk_ref_i                      (clk200),
       .device_temp                    (),
       .sys_rst                        ((ck_rst && locked))
       );

   clk_wiz_1 clocks
   (
    // Clock out ports
    .clk_out1(clk200),     // output clk_out1
    .clk_out2(clk166),    // output clk_out2
    // Status and control signals
    .reset(~ck_rst),        // input reset
    .locked(locked),       // output locked
   // Clock in ports
    .clk_in1(CLK100MHZ));  // input clk_in1

   fifo_generator_A w_FIFO
   (
      .rst (reset),
      .wr_clk (CLK100MHZ),
      .rd_clk (ui_clk),
      .din (din),
      .wr_en (wr_en),
      .rd_en (rd_en),
      .dout (dout),
      .full (full),
      .empty (empty),
      .valid (valid)
   );

   fifo_generator_B r_FIFO
   (
      .rst (reset),
      .wr_clk (ui_clk),
      .rd_clk (CLK100MHZ),
      .din (rdin),
      .wr_en (rwr_en),
      .rd_en (rrd_en),
      .dout (rdout),
      .full (rfull),
      .empty (rempty),
      .valid (rvalid)
   );


endmodule // End of Module dram_controller

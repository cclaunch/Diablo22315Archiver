//==========================================================================================================
// Diablo 2315 Archiver
// File Name: uploader.v
//
// Written for 2315 on Diablo by Carl Claunch
//
//==========================================================================================================

// This routine will loop through all sectors from RAM
//
// Set up Cyl, Head, Sector and reset Word address to 0 for each sector 
//
// pulse echoword and grab the data in readback when dataready == 1
// we advance the word address after each read has finished 
// we interlock waiting for dataready from dram_controller
//
// sector read saved the syncerror and ECCerror status in word 322 of DRAM
// thus we read this to grab the status
//
// for each sector begun, emit text string with cylinder, head, and sector
// format and send each word of sector as a line
// format text string for error conditions at end
// send blank line to separate sectors
//
// This is started by a pushbutton on the FPGA board

module uploader (

// Inputs
    input wire clock,                     // system clock
    input wire reset,                     // system reset
    input wire upload_cartridge,          // ask us to fetch the contents of the RAM
    input wire dataready,                 // indication RAM contents is ready
    input wire [15:0] readback,           // data to be written 

// Outputs
    output reg echoword,                  // trigger dram_controller to read a word
    output reg [7:0] cyl_address,         // which RAM cylinder to read
    output reg       head_address,        // which RAM head to read
    output reg [1:0] sector_address,      // which RAM sector to read
    output reg [8:0] word_address,        // which RAM sector word to read
    output reg dumpdone,                  // indicates the uploading is complete
    output txd_out                        // our serial bitstream

);

//============================ Internal Connections ==================================

//============================ Logic =================================================

// simple nested loop
//   cyl 0 to 202
//     head 0 to 1
//       sector 0 to 3
//         word 0 to 321
//
// formatting controlled by state machine  

`define AR0 3'd0
`define AR1 3'd1
`define AR2 3'd2
`define AR3 3'd3
`define AR4 3'd4
`define AR5 3'd5
`define AR6 3'd6
`define AR7 3'd7
// 0 is idle
// 1 write sector information on serial port
// 2 grab each of 321 words of sector and word 322
// 3 dump each word on serial port
// 4 bumps for next word or ends sector
// 5 write sync or ECC error status to serial port
// 6 bump for next sector
// 7 is end wait
reg [2:0]        upload_state;    // archive machine state variable
reg              data_write_stb;
reg              ascii_write_stb;
reg [15:0]       data_in;
reg [7:0]        ascii_in;
wire             busy;
`define          m_idle 3'd0
`define          m_load 3'd1
`define          m_wait 3'd2
`define          m_wr0  3'd3
`define          m_wr1  3'd4
`define          m_end  3'd5
                 // m_idle  0  - initial state
                 // m_load  1  - set up for string to send
                 // m_wait  2  - wait until not busy
                 // m_wr0   3  - write step 1
                 // m_wr1   4  - write step 2
                 // m_end   5  - finished state
reg [2:0]        c_state;
reg [4:0]        char_cnt;
reg [4:0]        string_size;
`define          msg0_size   21
`define          msg1_size   15
reg              msg_sent_flag;   
reg [7:0]        msg0 [20:0];
`define          cylinder1    4 
`define          cylinder2    5 
`define          cylinder3    6
`define          head1       15 
`define          sector1     17 
`define          sector2     18 
reg  [7:0]       msg1 [14:0];
`define          syncerror1  5
`define          ECCerror1  12
reg [1:0]        msg_ptr;
reg              send_msg_request;   

reg [3:0]        metabutton;     // metastability chain for button 0 input 

//=============================== start of code ======================================//
always @ (posedge clock)
begin : UPLOAD // block name
  if(reset) begin
    upload_state      <= `AR0;
    metabutton        <= 4'b0000;
    cyl_address       <= 8'd0;
    head_address      <= 1'b0;
    sector_address    <= 2'd0;
    word_address      <= 9'd0;
    echoword          <= 1'b0;
    dumpdone          <= 1'b0;
    data_write_stb    <= 1'b0;
    data_in           <= 16'h0000;
    string_size       <= 5'd0;
    msg_ptr           <= 2'd0;
    send_msg_request  <= 1'b0;
    // CYL ### SECTOR ### cr nl
    msg0[0]           <= 8'h43; 
    msg0[1]           <= 8'h59; 
    msg0[2]           <= 8'h4c; 
    msg0[3]           <= 8'h20; 
    msg0[`cylinder1]  <= 8'h20;
    msg0[`cylinder2]  <= 8'h20;
    msg0[`cylinder3]  <= 8'h20;
    msg0[7]           <= 8'h20; 
    msg0[8]           <= 8'h53; 
    msg0[9]           <= 8'h45; 
    msg0[10]          <= 8'h43; 
    msg0[11]          <= 8'h54; 
    msg0[12]          <= 8'h4f; 
    msg0[13]          <= 8'h52; 
    msg0[14]          <= 8'h20; 
    msg0[`head1]      <= 8'h20;
    msg0[16]          <= 8'h20;
    msg0[`sector1]    <= 8'h20;
    msg0[`sector2]    <= 8'h20;
    msg0[19]          <= 8'h0d; 
    msg0[20]          <= 8'h0a; 
    // SYNC # DATA # cr nl
    msg1[0]           <= 8'h53;
    msg1[1]           <= 8'h59;
    msg1[2]           <= 8'h4e;
    msg1[3]           <= 8'h43;
    msg1[4]           <= 8'h20;
    msg1[`syncerror1] <= 8'h20;
    msg1[6]           <= 8'h20;
    msg1[7]           <= 8'h44;
    msg1[8]           <= 8'h41;
    msg1[9]           <= 8'h54;
    msg1[10]          <= 8'h41;
    msg1[11]          <= 8'h20;
    msg1[`ECCerror1]  <= 8'h20;
    msg1[13]          <= 8'h0d;
    msg1[14]          <= 8'h0a;
  end
  else begin

     // button has bounce and is not in our clock domain, so deal with metastability
     metabutton[3:0] <= {metabutton[2:0], upload_cartridge};

     case(upload_state)

// Sitting around waiting to begin our work when button 0 pushed
    `AR0: begin  

       // next state determination
       // must be at home to start archive while pushbutton is pressed
       upload_state <=  (metabutton[3] == 1'b0) && (metabutton[2] == 1'b1)
                        ? `AR1
                        : `AR0;

       // housekeeping
       dumpdone         <= 1'b0;
       data_write_stb   <= 1'b0;
       data_in          <= 16'h0000;
       send_msg_request <= 1'b0;

    end   

// Emitting the header for each sector on USB
    `AR1: begin  

       // next state determination
       upload_state <= msg_sent_flag == 1'b0
                       ? `AR1
                       : `AR2;

       // write the header to USB serial link
       send_msg_request <= msg_sent_flag == 1'b0
                           ? 1'b1
                           : 1'b0;

       // our data
       msg0[`cylinder1] <=  8'h58;
       msg0[`cylinder2] <=  cyl_address[7:4] == 4'b0000
                            ? 8'h30
                            : cyl_address[7:4] == 4'b0001
                              ? 8'h31
                              : cyl_address[7:4] == 4'b0010
                                ? 8'h32
                                : cyl_address[7:4] == 4'b0011
                                  ? 8'h33
                                  : cyl_address[7:4] == 4'b0100
                                    ? 8'h34
                                    : cyl_address[7:4] == 4'b0101
                                      ? 8'h35
                                      : cyl_address[7:4] == 4'b0110
                                        ? 8'h36
                                        : cyl_address[7:4] == 4'b0111
                                          ? 8'h37
                                          : cyl_address[7:4] == 4'b1000
                                            ? 8'h38
                                            : cyl_address[7:4] == 4'b1001
                                              ? 8'h39
                                              : cyl_address[7:4] == 4'b1010
                                                ? 8'h41
                                                : cyl_address[7:4] == 4'b1011
                                                  ? 8'h42
                                                  : cyl_address[7:4] == 4'b1100
                                                    ? 8'h43
                                                    : cyl_address[7:4] == 4'b1101
                                                      ? 8'h44
                                                      : cyl_address[7:4] == 4'b1110
                                                        ? 8'h45
                                                        : 8'h46;
       msg0[`cylinder3] <=  cyl_address[3:0] == 4'b0000
                            ? 8'h30
                            : cyl_address[3:0] == 4'b0001
                              ? 8'h31
                              : cyl_address[3:0] == 4'b0010
                                ? 8'h32
                                : cyl_address[3:0] == 4'b0011
                                  ? 8'h33
                                  : cyl_address[3:0] == 4'b0100
                                    ? 8'h34
                                    : cyl_address[3:0] == 4'b0101
                                      ? 8'h35
                                      : cyl_address[3:0] == 4'b0110
                                        ? 8'h36
                                        : cyl_address[3:0] == 4'b0111
                                          ? 8'h37
                                          : cyl_address[3:0] == 4'b1000
                                            ? 8'h38
                                            : cyl_address[3:0] == 4'b1001
                                              ? 8'h39
                                              : cyl_address[3:0] == 4'b1010
                                                ? 8'h41
                                                : cyl_address[3:0] == 4'b1011
                                                  ? 8'h42
                                                  : cyl_address[3:0] == 4'b1100
                                                    ? 8'h43
                                                    : cyl_address[3:0] == 4'b1101
                                                      ? 8'h44
                                                      : cyl_address[3:0] == 4'b1110
                                                        ? 8'h45
                                                        : 8'h46;
       msg0[`head1] <= head_address == 1'b1
                       ? 8'h31
                       : 8'h30;
       msg0[`sector1] <= sector_address == 2'b00
                         ? 8'h30
                         : sector_address == 2'b01
                           ? 8'h30
                           : sector_address == 2'b10
                             ? 8'h31
                             : 8'h31; 
       msg0[`sector2] <= sector_address == 2'b00
                         ? 8'h30
                         : sector_address == 2'b01
                           ? 8'h31
                           : sector_address == 2'b10
                             ? 8'h30
                             : 8'h31; 

       msg_ptr         <= 2'd0;
       string_size     <= 5'd`msg0_size;

       // housekeeping
       dumpdone        <= 1'b0;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;

    end   

// Request one of 321 words of sector
    `AR2: begin  

       // next state determination
       // for each, request RAM word and wait
       upload_state <= (dataready == 1'b1)
                       ? word_address == 9'd321
                         ? `AR5
                         : `AR3
                       : `AR2;

       // raise request for data word and drop when result returned
       echoword        <=  dataready == 1'b1
                           ? 1'b0
                           : 1'b1;

       // housekeeping
       dumpdone        <= 1'b0;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;
       send_msg_request <= 1'b0;

    end   

// Dump each word on serial port
    `AR3: begin  

       // next state determination
       // wait for USB link to finish
       upload_state    <=  busy == 1'b1
                           ? `AR3
                           : `AR4;

       // write the data word to USB serial link for data part
       data_write_stb  <=  busy == 1'b0
                           ? 1'b1
                           : 1'b0;
      
       // our data to write
       data_in         <= readback;

       // housekeeping
       dumpdone        <= 1'b0;
       send_msg_request <= 1'b0;

    end   

// Bumps for next word or ends sector
    `AR4: begin  

       // next state determination
       // walk through 321 words of the sector
       // plus final word for errors encountered
       upload_state <= busy == 1'b0
                       ? word_address == 9'd321
                         ? `AR6
                         : `AR2
                       : `AR4;

       // bump relevant values
       word_address <= busy == 1'b1
                       ? word_address
                       : word_address == 9'd321
                         ? 9'd0
                         : word_address + 1;

       // housekeeping
       dumpdone        <= 1'b0;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;
       send_msg_request <= 1'b0;

    end   

// Write sync or ECC error status to serial port
    `AR5: begin  

       // next state determination
       // we report on any errors while reading sector
       upload_state <= msg_sent_flag == 1'b1
                       ? `AR6
                       : `AR5;

       // write the trailer to USB serial link
       send_msg_request <= msg_sent_flag == 1'b0
                           ? 1'b1
                           : 1'b0;

       // set up message
       msg_ptr      <= 2'd1;
       msg1[`syncerror1] <= readback[15] == 1'b1
                            ? 8'h31
                            : 8'h20;
       msg1[`ECCerror1]  <= readback[14] == 1'b1
                            ? 8'h31
                            : 8'h20;
       string_size       <= 5'd`msg1_size;


       // housekeeping
       dumpdone        <= 1'b0;
       word_address    <= 9'd0;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;

    end   

// Bump for next sector
    `AR6: begin  

       // next state determination
       // we move from sector 00 to 03
       // then move upper track to lower track
       // for another four sectors before advancing
       // then from cylinder to cylinder until complete
       upload_state <= cyl_address == 202 && head_address == 1'b1 && sector_address == 2'b11
                       ? `AR7
                       : msg_sent_flag == 1'b1
                         ? `AR6
                         : `AR1;

       // bump relevant values
       sector_address <= msg_sent_flag == 1'b1
                         ? sector_address
                         : sector_address == 2'b11
                           ? 2'b00
                           : sector_address + 1;

       head_address <= msg_sent_flag == 1'b1
                       ? head_address
                       : sector_address == 2'b11
                         ? ~head_address
                         : head_address;

       cyl_address <= msg_sent_flag == 1'b1
                      ? cyl_address
                      : sector_address == 2'b11 && head_address == 1'b1
                        ? cyl_address + 1
                        
: cyl_address;

       // housekeeping
       dumpdone        <= 1'b0;
       word_address    <= 9'd0;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;
       send_msg_request <= 1'b0;

    end   

// wrap up after uploading complete cartridge
    `AR7: begin  

       // next state determination
       // lock ourselves here for duration
       upload_state <= `AR7;

       // inform we are done with the archive
       dumpdone <= 1'b1;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;
       send_msg_request <= 1'b0;

    end   

    default: begin
       upload_state <= `AR0;
       dumpdone        <= 1'b0;
       data_write_stb  <= 1'b0;
       data_in         <= 16'h0000;
       send_msg_request <= 1'b0;
    end

    endcase

  end
end // End of Block UPLOAD

// code borrowed from William Carter, Eclektek LLC
// and rewritten in Verilog from the original VHDL
// see license allowing borrowing in UART_DEBUGGER2.vhd file

// A state machine to write text strings
always @ (posedge clock)
begin : TEXT_SM 
  if (reset) begin
     c_state                <= `m_idle;
     char_cnt               <= 5'd0;
     ascii_in               <= 8'h20;
     ascii_write_stb        <= 1'b0;
     msg_sent_flag          <= 1'b0;
  end
  else begin

     case(c_state)

// 0 waiting to be asked to send a string
    `m_idle: begin  

       // next state if msg is requested is m_load
       c_state              <= send_msg_request == 1'b1
                               ?  `m_load
                               :  `m_idle;

       char_cnt             <= 5'd0;
       ascii_write_stb      <= 1'b0;
       msg_sent_flag        <= 1'b0;

    end   

// 1 set up for string
    `m_load: begin  

       // next state 
       c_state             <= char_cnt < string_size
                              ? `m_wait
                              : `m_end;

       ascii_write_stb  <= 1'b0;
       
       ascii_in            <= char_cnt < string_size && msg_ptr == 0
                              ? msg0[char_cnt]
                              : char_cnt < string_size && msg_ptr == 1
                                ? msg1[char_cnt]
                                : 8'h20;
    end   

// 2 hold until we are not busy
    `m_wait: begin 

       // next state write0 when not busy
       c_state            <= busy == 1'b0
                             ? `m_wr0
                             : `m_wait;

       ascii_write_stb  <= 1'b0;

    end   

// 3 first stage of write
    `m_wr0: begin  

       // next state is write 1
       c_state             <= `m_wr1;

       ascii_write_stb     <= 1'b1;
       char_cnt            <= char_cnt + 1;

    end   

// 4 second stage of write
    `m_wr1: begin  

       // next state is m_load once we go busy
       c_state            <= busy == 1'b0
                             ? `m_wr1
                             : `m_load;

       ascii_write_stb     <= 1'b0;

    end   

// 5 we are finished
    `m_end: begin  

       // next state is idle
       c_state             <= send_msg_request == 1'b0 && busy == 1'b0
                              ? `m_idle
                              : `m_end;

       ascii_write_stb     <= 1'b0;
       msg_sent_flag       <= busy == 1'b0
                              ? 1'b1
                              : 1'b0;

    end   

// should not happen
    default: begin

      c_state              <= `m_idle;

    end

    endcase

  end
end // end of block TEXT_SM


UART_DEBUGGER2 #(.CLK_RATE(100000000),
                 .BAUD_RATE(115200),
                 .DATA_CHARS(4)
                ) 
    i_UART_DEBUGGER2 (
        .clk (clock),
        .areset (reset),
        .data_write_stb (data_write_stb),
        .data_in (data_in),
        .ascii_in (ascii_in),
        .ascii_write_stb (ascii_write_stb),
        .busy (busy),
        .txd_out (txd_out)
   );

endmodule // uploader
